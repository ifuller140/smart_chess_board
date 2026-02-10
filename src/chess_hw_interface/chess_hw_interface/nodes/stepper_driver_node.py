#!/usr/bin/env python3
"""
Stepper Driver Node for A4988 Drivers + NEMA 11 Motors.

Uses pigpio DMA wave chains for jitter-free, hardware-timed step pulses.
Both motors always step in a single wave — no inter-motor lag.

Supports TWO control modes:
  1. Point-to-point: `/stepper/command` (Point) — move N steps then stop
  2. Velocity:       `/stepper/velocity` (Twist) — continuous joystick control
     with smooth acceleration/deceleration

Requires:
- pigpio library: pip install pigpio
- pigpio daemon running: sudo pigpiod
"""

import math
import threading
import time

import pigpio
import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


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
MIN_SPEED = 100

# Velocity control
DEFAULT_MAX_VELOCITY = 800      # steps/sec max for velocity mode
DEFAULT_ACCELERATION = 2000     # steps/sec² ramp rate
VELOCITY_TICK_MS = 20           # velocity loop period (50Hz)

# Wave chain limits
MAX_UNIQUE_WAVES = 180


class StepperDriverNode(Node):
    """
    ROS2 node for controlling A4988 stepper drivers via pigpio DMA.

    Two modes:
      - Point-to-point: receives step counts on /stepper/command
      - Velocity: receives velocity on /stepper/velocity for joystick control
    """

    def __init__(self):
        super().__init__('stepper_driver_node')

        # ---- Parameters ----
        self.declare_parameter('motorA_dir_pin', DEFAULT_MOTOR_A_DIR_PIN)
        self.declare_parameter('motorA_step_pin', DEFAULT_MOTOR_A_STEP_PIN)
        self.declare_parameter('motorB_dir_pin', DEFAULT_MOTOR_B_DIR_PIN)
        self.declare_parameter('motorB_step_pin', DEFAULT_MOTOR_B_STEP_PIN)
        self.declare_parameter('motor_enable_pin', DEFAULT_MOTOR_ENABLE_PIN)
        self.declare_parameter('hold_torque_when_idle', True)
        self.declare_parameter('max_velocity', DEFAULT_MAX_VELOCITY)
        self.declare_parameter('acceleration', DEFAULT_ACCELERATION)

        self.motorA_dir = self.get_parameter('motorA_dir_pin').value
        self.motorA_step = self.get_parameter('motorA_step_pin').value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').value
        self.motorB_step = self.get_parameter('motorB_step_pin').value
        self.motor_enable_pin = self.get_parameter('motor_enable_pin').value
        self.hold_torque = self.get_parameter('hold_torque_when_idle').value
        self.max_velocity = float(self.get_parameter('max_velocity').value)
        self.acceleration = float(self.get_parameter('acceleration').value)

        self.emergency_stop = False

        # ---- pigpio connection ----
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod daemon. Start it with: sudo pigpiod"
            )
            raise RuntimeError("pigpiod not running")

        for pin in [self.motorA_dir, self.motorA_step,
                    self.motorB_dir, self.motorB_step, self.motor_enable_pin]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)

        # Disable motors initially (A4988 ENABLE is active LOW)
        self.pi.write(self.motor_enable_pin, 1)

        self.get_logger().info(
            f"pigpio initialized: A(DIR={self.motorA_dir}, STEP={self.motorA_step}), "
            f"B(DIR={self.motorB_dir}, STEP={self.motorB_step}), "
            f"EN={self.motor_enable_pin}"
        )

        # ---- Velocity state ----
        self._vel_lock = threading.Lock()
        self._desired_vel_x = 0.0      # desired X velocity (steps/sec)
        self._desired_vel_y = 0.0      # desired Y velocity (steps/sec)
        self._current_vel_x = 0.0      # actual ramping X velocity
        self._current_vel_y = 0.0      # actual ramping Y velocity
        self._motors_enabled = False
        self._moving = False
        self._idle_since = time.time()

        # ---- ROS interfaces ----

        # Point-to-point mode
        self.command_sub = self.create_subscription(
            Point, '/stepper/command', self.command_callback, 10)

        # Velocity (joystick) mode
        self.velocity_sub = self.create_subscription(
            Twist, '/stepper/velocity', self.velocity_callback, 10)

        # Emergency stop
        self.stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.stop_callback, 10)

        # Status publisher
        self.status_pub = self.create_publisher(String, '/stepper/status', 10)

        # Speed/accel parameter update subscribers
        self.speed_sub = self.create_subscription(
            Float32, '/stepper/set_max_velocity', self._set_max_velocity_cb, 10)
        self.accel_sub = self.create_subscription(
            Float32, '/stepper/set_acceleration', self._set_acceleration_cb, 10)

        # Velocity control timer (50Hz)
        self._vel_timer = self.create_timer(
            VELOCITY_TICK_MS / 1000.0, self._velocity_tick)

        self.get_logger().info(
            f"Stepper driver ready. max_vel={self.max_velocity}, accel={self.acceleration}")

    # ------------------------------------------------------------------
    # Parameter update callbacks
    # ------------------------------------------------------------------

    def _set_max_velocity_cb(self, msg: Float32):
        self.max_velocity = max(50.0, min(2000.0, msg.data))
        self.get_logger().info(f"Max velocity set to {self.max_velocity}")

    def _set_acceleration_cb(self, msg: Float32):
        self.acceleration = max(100.0, min(10000.0, msg.data))
        self.get_logger().info(f"Acceleration set to {self.acceleration}")

    # ------------------------------------------------------------------
    # Motor control helpers
    # ------------------------------------------------------------------

    def _enable_motors(self):
        if not self._motors_enabled:
            self.pi.write(self.motor_enable_pin, 0)
            self._motors_enabled = True
            time.sleep(0.001)

    def _disable_motors(self):
        if self._motors_enabled:
            self.pi.write(self.motor_enable_pin, 1)
            self._motors_enabled = False

    def stop_motors(self):
        """Emergency stop — kill wave and disable."""
        try:
            self.pi.wave_tx_stop()
        except Exception:
            pass
        self.pi.write(self.motorA_step, 0)
        self.pi.write(self.motorB_step, 0)
        with self._vel_lock:
            self._desired_vel_x = 0.0
            self._desired_vel_y = 0.0
            self._current_vel_x = 0.0
            self._current_vel_y = 0.0
        self._disable_motors()
        self._moving = False

    # ------------------------------------------------------------------
    # Emergency stop
    # ------------------------------------------------------------------

    def stop_callback(self, msg: Bool):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP triggered')
            self.stop_motors()

    # ------------------------------------------------------------------
    # VELOCITY MODE — joystick control
    # ------------------------------------------------------------------

    def velocity_callback(self, msg: Twist):
        """
        Set desired velocity for joystick control.

        twist.linear.x = desired X velocity (steps/sec, signed)
        twist.linear.y = desired Y velocity (steps/sec, signed)
        """
        if self.emergency_stop:
            return

        with self._vel_lock:
            self._desired_vel_x = msg.linear.x
            self._desired_vel_y = msg.linear.y

    def _velocity_tick(self):
        """
        Timer callback (50Hz) — smoothly ramp velocity and generate steps.

        This is the core joystick loop:
        1. Ramp current velocity toward desired velocity (smooth accel/decel)
        2. Convert Cartesian velocity to CoreXY motor velocities
        3. Generate a small DMA wave for this tick's worth of steps
        """
        if self.emergency_stop:
            return

        dt = VELOCITY_TICK_MS / 1000.0

        with self._vel_lock:
            desired_x = self._desired_vel_x
            desired_y = self._desired_vel_y

        # --- Smooth ramp toward desired velocity ---
        self._current_vel_x = self._ramp_toward(
            self._current_vel_x, desired_x, self.acceleration * dt)
        self._current_vel_y = self._ramp_toward(
            self._current_vel_y, desired_y, self.acceleration * dt)

        vel_x = self._current_vel_x
        vel_y = self._current_vel_y

        # If effectively stopped, disable motors after a brief hold
        speed = math.sqrt(vel_x * vel_x + vel_y * vel_y)
        if speed < 1.0:
            self._current_vel_x = 0.0
            self._current_vel_y = 0.0
            if self._moving:
                self._moving = False
                self._idle_since = time.time()
                self.status_pub.publish(String(data='IDLE'))
            # Disable motors after 2 seconds idle (if not holding torque)
            if not self.hold_torque and (time.time() - self._idle_since) > 2.0:
                self._disable_motors()
            return

        # --- CoreXY mapping ---
        # Cartesian → motor velocities
        # +X (right): A+, B-   →  vel_a = vel_x + vel_y
        # +Y (up):    A+, B+   →  vel_b = vel_y - vel_x
        vel_a = vel_x + vel_y
        vel_b = vel_y - vel_x

        # How many steps this tick?
        steps_a_float = vel_a * dt
        steps_b_float = vel_b * dt

        steps_a = int(round(steps_a_float))
        steps_b = int(round(steps_b_float))

        if steps_a == 0 and steps_b == 0:
            return

        if not self._moving:
            self._moving = True
            self.status_pub.publish(String(data='MOVING'))

        self._enable_motors()

        # Set directions — INVERTED per user's hardware
        # (positive step count → DIR pin LOW for this hardware layout)
        self.pi.write(self.motorA_dir, 0 if steps_a >= 0 else 1)
        self.pi.write(self.motorB_dir, 0 if steps_b >= 0 else 1)
        time.sleep(DIR_SETUP_US / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)

        # For velocity ticks, steps are small enough to use a simple wave
        # (no acceleration needed within a single tick)
        self._send_tick_wave(abs_a, abs_b, speed)

    def _send_tick_wave(self, abs_a: int, abs_b: int, speed: float):
        """Generate a small DMA wave for one velocity tick."""
        max_steps = max(abs_a, abs_b)
        if max_steps == 0:
            return

        # Calculate delay from current speed
        delay_us = max(100, int(1_000_000 / max(speed, 1.0)))

        # Bresenham for this small batch
        step_plan = []
        err_a = 0
        err_b = 0
        for _ in range(max_steps):
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
                step_plan.append((do_a, do_b))

        if not step_plan:
            return

        wait_us = max(1, delay_us - STEP_PULSE_US)

        # Build a single wave for all steps in this tick
        # (they all share the same delay, so one wave + loop)
        # Determine unique step patterns
        patterns = set(step_plan)
        wave_ids = []

        try:
            self.pi.wave_clear()

            pattern_to_wid = {}
            for do_a, do_b in patterns:
                set_mask = 0
                clear_mask = 0
                if do_a:
                    set_mask |= (1 << self.motorA_step)
                    clear_mask |= (1 << self.motorA_step)
                if do_b:
                    set_mask |= (1 << self.motorB_step)
                    clear_mask |= (1 << self.motorB_step)

                self.pi.wave_add_generic([
                    pigpio.pulse(set_mask, 0, STEP_PULSE_US),
                    pigpio.pulse(0, clear_mask, wait_us),
                ])
                wid = self.pi.wave_create()
                if wid < 0:
                    return  # Silently fail on resource exhaustion
                pattern_to_wid[(do_a, do_b)] = wid
                wave_ids.append(wid)

            # Group consecutive identical patterns
            groups = []
            cur = step_plan[0]
            cnt = 1
            for s in step_plan[1:]:
                if s == cur:
                    cnt += 1
                else:
                    groups.append((cur, cnt))
                    cur = s
                    cnt = 1
            groups.append((cur, cnt))

            chain = []
            for pattern, c in groups:
                wid = pattern_to_wid[pattern]
                if c == 1:
                    chain.append(wid)
                else:
                    chain.extend([255, 0, wid, 255, 1, c & 0xFF, (c >> 8) & 0xFF])

            self.pi.wave_chain(chain)

            # Wait for this tick's wave (should be very short)
            while self.pi.wave_tx_busy():
                time.sleep(0.0005)

        finally:
            for wid in wave_ids:
                try:
                    self.pi.wave_delete(wid)
                except Exception:
                    pass

    @staticmethod
    def _ramp_toward(current: float, target: float, max_delta: float) -> float:
        """Smoothly ramp current value toward target by at most max_delta."""
        diff = target - current
        if abs(diff) <= max_delta:
            return target
        return current + math.copysign(max_delta, diff)

    # ------------------------------------------------------------------
    # POINT-TO-POINT MODE — discrete step commands
    # ------------------------------------------------------------------

    def command_callback(self, msg: Point):
        """
        Execute a point-to-point move.

        msg.x = steps for motor A
        msg.y = steps for motor B
        msg.z = speed percent (1-100)
        """
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
        speed = max(0.0, min(100.0, speed_percent))
        return int(MIN_SPEED + (speed / 100.0) * (MAX_SPEED - MIN_SPEED))

    def _calculate_speed_profile(self, total_steps, speed_percent):
        """Calculate trapezoidal speed profile for point-to-point moves."""
        if total_steps <= 0:
            return []

        target_speed = self._speed_to_steps_per_sec(speed_percent)
        accel_steps = min(100, total_steps // 3)

        if total_steps <= accel_steps * 2:
            delay_us = int(1_000_000 / MIN_SPEED)
            return [(total_steps, delay_us)]

        decel_steps = accel_steps
        cruise_steps = total_steps - accel_steps - decel_steps

        profile = []
        for i in range(accel_steps):
            t = (i + 1) / accel_steps
            speed = MIN_SPEED + t * (target_speed - MIN_SPEED)
            profile.append((1, int(1_000_000 / speed)))

        if cruise_steps > 0:
            profile.append((cruise_steps, int(1_000_000 / target_speed)))

        for i in range(decel_steps):
            t = (i + 1) / decel_steps
            speed = target_speed - t * (target_speed - MIN_SPEED)
            profile.append((1, int(1_000_000 / speed)))

        return profile

    def move_motors(self, steps_a: int, steps_b: int, speed_percent: float):
        """Move both motors via DMA wave chain (point-to-point)."""
        if steps_a == 0 and steps_b == 0:
            return

        # INVERTED DIR pins per user's hardware
        self.pi.write(self.motorA_dir, 0 if steps_a >= 0 else 1)
        self.pi.write(self.motorB_dir, 0 if steps_b >= 0 else 1)
        time.sleep(DIR_SETUP_US / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        if max_steps == 0:
            return

        profile = self._calculate_speed_profile(max_steps, speed_percent)
        self._enable_motors()

        try:
            self._build_and_send_wave_chain(abs_a, abs_b, max_steps, profile)
        finally:
            if not self.hold_torque:
                self._disable_motors()

    def _build_and_send_wave_chain(self, abs_a, abs_b, max_steps, profile):
        """Build and execute a DMA wave chain for a full point-to-point move."""
        per_step_delays = []
        for step_count, delay_us in profile:
            per_step_delays.extend([delay_us] * step_count)

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

        groups = []
        cur = (step_plan[0][0], step_plan[0][1], step_plan[0][2])
        cnt = 1
        for entry in step_plan[1:]:
            p = (entry[0], entry[1], entry[2])
            if p == cur:
                cnt += 1
            else:
                groups.append((cur, cnt))
                cur = p
                cnt = 1
        groups.append((cur, cnt))

        wave_ids = []
        p2w = {}

        try:
            self.pi.wave_clear()
            for pattern, _ in groups:
                if pattern in p2w:
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

                self.pi.wave_add_generic([
                    pigpio.pulse(set_mask, 0, STEP_PULSE_US),
                    pigpio.pulse(0, clear_mask, wait_us),
                ])
                wid = self.pi.wave_create()
                if wid < 0:
                    raise RuntimeError(f"wave_create failed ({wid})")
                p2w[pattern] = wid
                wave_ids.append(wid)

            chain = []
            for pattern, c in groups:
                wid = p2w[pattern]
                if c == 1:
                    chain.append(wid)
                else:
                    chain.extend([255, 0, wid, 255, 1, c & 0xFF, (c >> 8) & 0xFF])

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

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

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
