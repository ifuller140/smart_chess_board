#!/usr/bin/env python3
"""
Stepper Driver Node for A4988 Drivers + NEMA 11 Motors.

Uses pigpio DMA wave chains for jitter-free, hardware-timed step pulses.
Both motors always step in a single wave — no inter-motor lag.

Requires:
- pigpio library: pip install pigpio
- pigpio daemon running: sudo pigpiod
"""

import time

import pigpio
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, String


# Pin defaults (BCM) — can be overridden via ROS parameters
DEFAULT_MOTOR_A_DIR_PIN = 27
DEFAULT_MOTOR_A_STEP_PIN = 22
DEFAULT_MOTOR_B_DIR_PIN = 6
DEFAULT_MOTOR_B_STEP_PIN = 5
DEFAULT_MOTOR_ENABLE_PIN = 17

# Timing
DIR_SETUP_US = 5
STEP_PULSE_US = 10

# Speed parameters (steps per second)
MAX_SPEED = 1200
MIN_SPEED = 250
ACCEL_STEPS = 100

# Wave chain limits
MAX_UNIQUE_WAVES = 180


class StepperDriverNode(Node):
    """ROS2 node for controlling A4988 stepper drivers via pigpio DMA."""

    def __init__(self):
        super().__init__('stepper_driver_node')

        # Parameters
        self.declare_parameter('motorA_dir_pin', DEFAULT_MOTOR_A_DIR_PIN)
        self.declare_parameter('motorA_step_pin', DEFAULT_MOTOR_A_STEP_PIN)
        self.declare_parameter('motorB_dir_pin', DEFAULT_MOTOR_B_DIR_PIN)
        self.declare_parameter('motorB_step_pin', DEFAULT_MOTOR_B_STEP_PIN)
        self.declare_parameter('motor_enable_pin', DEFAULT_MOTOR_ENABLE_PIN)
        self.declare_parameter('hold_torque_when_idle', True)

        self.motorA_dir = self.get_parameter('motorA_dir_pin').value
        self.motorA_step = self.get_parameter('motorA_step_pin').value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').value
        self.motorB_step = self.get_parameter('motorB_step_pin').value
        self.motor_enable = self.get_parameter('motor_enable_pin').value
        self.hold_torque_when_idle = self.get_parameter('hold_torque_when_idle').value

        self.emergency_stop = False

        # Connect to pigpio daemon
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod daemon. "
                "Start it with: sudo pigpiod"
            )
            raise RuntimeError("pigpiod not running")

        # GPIO setup
        for pin in [
            self.motorA_dir, self.motorA_step,
            self.motorB_dir, self.motorB_step,
            self.motor_enable,
        ]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # Disable motors initially (A4988 ENABLE is active LOW)
        self.pi.write(self.motor_enable, 1)

        self.get_logger().info(
            f"pigpio initialized: A(DIR={self.motorA_dir}, STEP={self.motorA_step}), "
            f"B(DIR={self.motorB_dir}, STEP={self.motorB_step}), "
            f"EN={self.motor_enable}"
        )

        # ROS interfaces
        self.command_sub = self.create_subscription(
            Point,
            '/stepper/command',
            self.command_callback,
            10,
        )
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, '/stepper/status', 10)

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP triggered')
            self.stop_motors()

    def stop_motors(self):
        try:
            self.pi.wave_tx_stop()
        except Exception:
            pass
        self.pi.write(self.motorA_step, 0)
        self.pi.write(self.motorB_step, 0)
        self.pi.write(self.motor_enable, 1)

    def enable_motors(self):
        self.pi.write(self.motor_enable, 0)
        time.sleep(0.001)

    def command_callback(self, msg: Point):
        if self.emergency_stop:
            self.get_logger().warn('Cannot move: emergency stop active')
            return

        steps_a = int(msg.x)
        steps_b = int(msg.y)
        speed_percent = msg.z if msg.z > 0 else 50.0

        self.status_pub.publish(String(data='MOVING'))
        self.get_logger().info(
            f"Command: A={steps_a}, B={steps_b}, speed={speed_percent:.1f}%"
        )

        self.move_motors(steps_a, steps_b, speed_percent)
        self.status_pub.publish(String(data='IDLE'))

    def _speed_to_steps_per_sec(self, speed_percent: float) -> int:
        """Convert speed percentage to steps per second."""
        speed = max(0.0, min(100.0, speed_percent))
        return int(MIN_SPEED + (speed / 100.0) * (MAX_SPEED - MIN_SPEED))

    def _calculate_speed_profile(self, total_steps, speed_percent):
        """Calculate trapezoidal speed profile."""
        if total_steps <= 0:
            return []

        target_speed = self._speed_to_steps_per_sec(speed_percent)

        if total_steps <= ACCEL_STEPS * 2:
            delay_us = int(1_000_000 / MIN_SPEED)
            return [(total_steps, delay_us)]

        accel_steps = min(ACCEL_STEPS, total_steps // 3)
        decel_steps = accel_steps
        cruise_steps = total_steps - accel_steps - decel_steps

        profile = []

        for i in range(accel_steps):
            t = (i + 1) / accel_steps
            speed = MIN_SPEED + t * (target_speed - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))

        if cruise_steps > 0:
            delay_us = int(1_000_000 / target_speed)
            profile.append((cruise_steps, delay_us))

        for i in range(decel_steps):
            t = (i + 1) / decel_steps
            speed = target_speed - t * (target_speed - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))

        return profile

    def move_motors(self, steps_a: int, steps_b: int, speed_percent: float):
        """
        Move both motors simultaneously using a DMA wave chain.

        Both motors' step edges are in the same pulse — truly synchronous
        CoreXY motion.
        """
        if steps_a == 0 and steps_b == 0:
            return

        # Set directions
        self.pi.write(self.motorA_dir, 1 if steps_a >= 0 else 0)
        self.pi.write(self.motorB_dir, 1 if steps_b >= 0 else 0)
        time.sleep(DIR_SETUP_US / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)

        if max_steps == 0:
            return

        profile = self._calculate_speed_profile(max_steps, speed_percent)

        self.enable_motors()

        try:
            self._build_and_send_wave_chain(abs_a, abs_b, max_steps, profile)
        finally:
            if not self.hold_torque_when_idle:
                self.pi.write(self.motor_enable, 1)

    def _build_and_send_wave_chain(self, abs_a, abs_b, max_steps, profile):
        """Build and execute a DMA wave chain for the entire move."""
        # Flatten profile into per-step delays
        per_step_delays = []
        for step_count, delay_us in profile:
            per_step_delays.extend([delay_us] * step_count)

        # Bresenham interpolation
        step_plan = []
        err_a = 0
        err_b = 0
        for idx in range(max_steps):
            if self.emergency_stop:
                return

            do_a = False
            do_b = False

            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                do_a = True

            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                do_b = True

            if do_a or do_b:
                step_plan.append((do_a, do_b, per_step_delays[idx]))

        if not step_plan:
            return

        # Group consecutive identical patterns
        groups = []
        current_pattern = (step_plan[0][0], step_plan[0][1], step_plan[0][2])
        count = 1
        for entry in step_plan[1:]:
            pattern = (entry[0], entry[1], entry[2])
            if pattern == current_pattern:
                count += 1
            else:
                groups.append((current_pattern, count))
                current_pattern = pattern
                count = 1
        groups.append((current_pattern, count))

        # Create unique waves
        wave_ids = []
        pattern_to_wid = {}

        try:
            self.pi.wave_clear()

            for pattern, _ in groups:
                if pattern in pattern_to_wid:
                    continue

                do_a, do_b, delay_us = pattern
                wait_us = max(1, delay_us - STEP_PULSE_US)

                set_mask = 0
                clear_mask = 0

                if do_a:
                    set_mask |= (1 << self.motorA_step)
                    clear_mask |= (1 << self.motorA_step)
                if do_b:
                    set_mask |= (1 << self.motorB_step)
                    clear_mask |= (1 << self.motorB_step)

                pulses = [
                    pigpio.pulse(set_mask, 0, STEP_PULSE_US),
                    pigpio.pulse(0, clear_mask, wait_us),
                ]
                self.pi.wave_add_generic(pulses)
                wid = self.pi.wave_create()

                if wid < 0:
                    raise RuntimeError(
                        f"pigpio wave_create failed (code {wid})"
                    )

                pattern_to_wid[pattern] = wid
                wave_ids.append(wid)

            # Build chain
            chain = []
            for pattern, cnt in groups:
                wid = pattern_to_wid[pattern]
                if cnt == 1:
                    chain.append(wid)
                else:
                    loop_lo = cnt & 0xFF
                    loop_hi = (cnt >> 8) & 0xFF
                    chain.extend([255, 0, wid, 255, 1, loop_lo, loop_hi])

            # Execute and wait
            self.pi.wave_chain(chain)

            while self.pi.wave_tx_busy():
                if self.emergency_stop:
                    self.pi.wave_tx_stop()
                    break
                time.sleep(0.001)

        finally:
            for wid in wave_ids:
                try:
                    self.pi.wave_delete(wid)
                except Exception:
                    pass

    def destroy_node(self):
        self.stop_motors()
        try:
            self.pi.wave_clear()
        except Exception:
            pass
        if self.pi.connected:
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StepperDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
