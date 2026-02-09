#!/usr/bin/env python3
"""
Stepper Driver Node for A4988 Drivers + NEMA 11 Motors.

Controls two stepper motors using A4988 driver boards with STEP/DIR control.
Each motor requires STEP, DIR, and a shared ENABLE pin.
"""

import time

import RPi.GPIO as GPIO
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import Bool, String


class StepperDriverNode(Node):
    """ROS2 node for controlling A4988 stepper drivers."""

    def __init__(self):
        super().__init__('stepper_driver_node')

        # Parameters
        self.declare_parameter('motorA_dir_pin', 27)
        self.declare_parameter('motorA_step_pin', 22)
        self.declare_parameter('motorB_dir_pin', 6)
        self.declare_parameter('motorB_step_pin', 5)
        self.declare_parameter('motor_enable_pin', 17)

        self.declare_parameter('dir_setup_us', 50)
        self.declare_parameter('step_pulse_us', 100)
        self.declare_parameter('min_step_delay_ms', 5.0)
        self.declare_parameter('max_step_delay_ms', 50.0)
        self.declare_parameter('hold_torque_when_idle', True)
        self.declare_parameter('accel_ramp_steps', 40)

        # Read parameters
        self.motorA_dir = self.get_parameter('motorA_dir_pin').value
        self.motorA_step = self.get_parameter('motorA_step_pin').value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').value
        self.motorB_step = self.get_parameter('motorB_step_pin').value
        self.motor_enable = self.get_parameter('motor_enable_pin').value

        self.dir_setup_us = self.get_parameter('dir_setup_us').value
        self.step_pulse_us = self.get_parameter('step_pulse_us').value
        self.min_step_delay_ms = self.get_parameter('min_step_delay_ms').value
        self.max_step_delay_ms = self.get_parameter('max_step_delay_ms').value
        self.hold_torque_when_idle = self.get_parameter('hold_torque_when_idle').value
        self.accel_ramp_steps = max(0, int(self.get_parameter('accel_ramp_steps').value))

        self.dir_setup_sec = self.dir_setup_us / 1_000_000.0
        self.step_pulse_sec = self.step_pulse_us / 1_000_000.0

        self.emergency_stop = False

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in [
            self.motorA_dir,
            self.motorA_step,
            self.motorB_dir,
            self.motorB_step,
            self.motor_enable,
        ]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)

        # A4988 enable is active LOW.
        GPIO.output(self.motor_enable, GPIO.HIGH)

        self.get_logger().info(
            f"GPIO initialized: A(DIR={self.motorA_dir}, STEP={self.motorA_step}), "
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
        GPIO.output(self.motorA_step, GPIO.LOW)
        GPIO.output(self.motorB_step, GPIO.LOW)
        GPIO.output(self.motor_enable, GPIO.HIGH)

    def enable_motors(self):
        GPIO.output(self.motor_enable, GPIO.LOW)
        time.sleep(0.001)

    def speed_to_delay(self, speed_percent: float) -> float:
        speed = max(0.0, min(100.0, speed_percent))
        delay_ms = self.max_step_delay_ms - (speed / 100.0) * (
            self.max_step_delay_ms - self.min_step_delay_ms
        )
        return delay_ms / 1000.0

    def command_callback(self, msg: Point):
        if self.emergency_stop:
            self.get_logger().warn('Cannot move: emergency stop active')
            return

        steps_a = int(msg.x)
        steps_b = int(msg.y)
        speed = msg.z if msg.z > 0 else 50.0
        step_delay = self.speed_to_delay(speed)

        self.status_pub.publish(String(data='MOVING'))
        self.get_logger().info(
            f"Command: A={steps_a}, B={steps_b}, speed={speed:.1f}%"
        )

        self.move_motors(steps_a, steps_b, step_delay)
        self.status_pub.publish(String(data='IDLE'))

    def move_motors(self, steps_a: int, steps_b: int, step_delay: float):
        dir_a = 1 if steps_a >= 0 else -1
        dir_b = 1 if steps_b >= 0 else -1

        GPIO.output(self.motorA_dir, GPIO.HIGH if dir_a > 0 else GPIO.LOW)
        GPIO.output(self.motorB_dir, GPIO.HIGH if dir_b > 0 else GPIO.LOW)
        time.sleep(self.dir_setup_sec)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)

        if max_steps == 0:
            return

        self.enable_motors()
        GPIO.output(self.motorA_step, GPIO.LOW)
        GPIO.output(self.motorB_step, GPIO.LOW)

        err_a = 0
        err_b = 0

        for step_index in range(max_steps):
            if self.emergency_stop:
                break

            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                GPIO.output(self.motorA_step, GPIO.HIGH)

            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                GPIO.output(self.motorB_step, GPIO.HIGH)

            time.sleep(self.step_pulse_sec)
            GPIO.output(self.motorA_step, GPIO.LOW)
            GPIO.output(self.motorB_step, GPIO.LOW)

            applied_delay = self._apply_ramp_delay(step_index, max_steps, step_delay)
            if applied_delay > self.step_pulse_sec:
                time.sleep(applied_delay - self.step_pulse_sec)

        if not self.hold_torque_when_idle:
            GPIO.output(self.motor_enable, GPIO.HIGH)

    def _apply_ramp_delay(self, step_index: int, total_steps: int, cruise_delay: float) -> float:
        """Apply a simple symmetric linear ramp to reduce missed steps at start/end."""
        if self.accel_ramp_steps <= 0 or total_steps < 3:
            return cruise_delay

        ramp = min(self.accel_ramp_steps, total_steps // 2)
        if ramp <= 0:
            return cruise_delay

        if step_index < ramp:
            ratio = (ramp - step_index) / ramp
        elif step_index >= (total_steps - ramp):
            ratio = (step_index - (total_steps - ramp - 1)) / ramp
        else:
            ratio = 0.0

        ramp_span = max(0.0, self.max_step_delay_ms / 1000.0 - cruise_delay)
        return cruise_delay + (ramp_span * ratio * 0.8)

    def destroy_node(self):
        self.stop_motors()
        GPIO.cleanup()
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
