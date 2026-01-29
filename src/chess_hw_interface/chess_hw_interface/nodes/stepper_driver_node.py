#!/usr/bin/env python3
"""
Stepper Driver Node for A4988 Drivers + NEMA 11 Motors.

Controls two stepper motors using A4988 driver boards with STEP/DIR control.
Each motor requires only 2 GPIO pins:
  - DIR: Direction control (HIGH/LOW)
  - STEP: Step pulse (rising edge triggers one step)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import time
import sys

import RPi.GPIO as GPIO


class StepperDriverNode(Node):
    """ROS2 node for controlling A4988 stepper drivers."""
    
    def __init__(self):
        super().__init__('stepper_driver_node')
        
        # Declare parameters with defaults matching pins.yaml
        self.declare_parameter('motorA_dir_pin', 27)
        self.declare_parameter('motorA_step_pin', 22)
        self.declare_parameter('motorB_dir_pin', 6)
        self.declare_parameter('motorB_step_pin', 5)
        self.declare_parameter('step_pulse_us', 10)
        self.declare_parameter('min_step_delay_us', 100)
        self.declare_parameter('max_step_delay_us', 5000)
        
        # Get parameter values
        self.motorA_dir = self.get_parameter('motorA_dir_pin').value
        self.motorA_step = self.get_parameter('motorA_step_pin').value
        self.motorB_dir = self.get_parameter('motorB_dir_pin').value
        self.motorB_step = self.get_parameter('motorB_step_pin').value
        self.step_pulse_us = self.get_parameter('step_pulse_us').value
        self.min_step_delay_us = self.get_parameter('min_step_delay_us').value
        self.max_step_delay_us = self.get_parameter('max_step_delay_us').value
        
        # Convert timing to seconds for time.sleep()
        self.step_pulse_sec = self.step_pulse_us / 1_000_000
        self.default_step_delay_sec = self.max_step_delay_us / 1_000_000  # Start slow
        
        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup all motor pins as outputs
        for pin in [self.motorA_dir, self.motorA_step, self.motorB_dir, self.motorB_step]:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        self.get_logger().info(
            f"GPIO Initialized. Motor A: DIR={self.motorA_dir}, STEP={self.motorA_step}; "
            f"Motor B: DIR={self.motorB_dir}, STEP={self.motorB_step}"
        )
        
        # Subscribers
        # Command format: x = steps motor A, y = steps motor B, z = speed (0-100, optional)
        self.subscription = self.create_subscription(
            Point,
            '/stepper/command',
            self.command_callback,
            10
        )
        
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10
        )
        
        # Publishers
        self.status_pub = self.create_publisher(String, '/stepper/status', 10)
        
        self.emergency_stop = False
        
    def stop_callback(self, msg):
        """Handle emergency stop signal."""
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP TRIGGERED")
            self.stop_motors()
            
    def stop_motors(self):
        """Set all motor pins low."""
        GPIO.output(self.motorA_step, GPIO.LOW)
        GPIO.output(self.motorB_step, GPIO.LOW)
        
    def speed_to_delay(self, speed_percent: float) -> float:
        """
        Convert speed percentage (0-100) to step delay in seconds.
        
        0 = slowest (max_step_delay_us)
        100 = fastest (min_step_delay_us)
        """
        speed = max(0.0, min(100.0, speed_percent))
        delay_us = self.max_step_delay_us - (speed / 100.0) * (self.max_step_delay_us - self.min_step_delay_us)
        return delay_us / 1_000_000
        
    def command_callback(self, msg):
        """Handle movement commands."""
        if self.emergency_stop:
            self.get_logger().warn("Cannot move: Emergency Stop Active")
            return
            
        steps_a = int(msg.x)
        steps_b = int(msg.y)
        speed = msg.z if msg.z > 0 else 50.0  # Default to 50% speed if not specified
        
        step_delay = self.speed_to_delay(speed)
        
        self.get_logger().info(f"Moving: A={steps_a}, B={steps_b}, Speed={speed}%")
        self.status_pub.publish(String(data="Moving"))
        
        self.move_motors(steps_a, steps_b, step_delay)
        
        self.status_pub.publish(String(data="Idle"))
        
    def step_single_motor(self, step_pin: int, dir_pin: int, direction: int):
        """
        Execute a single step on one motor.
        
        Args:
            step_pin: GPIO pin for STEP signal
            dir_pin: GPIO pin for DIR signal
            direction: 1 for forward, -1 for reverse
        """
        # Direction is already set in move_motors
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(self.step_pulse_sec)
        GPIO.output(step_pin, GPIO.LOW)
        
    def move_motors(self, steps_a: int, steps_b: int, step_delay: float):
        """
        Move both motors with synchronized stepping using Bresenham interpolation.
        
        Args:
            steps_a: Number of steps for motor A (negative = reverse)
            steps_b: Number of steps for motor B (negative = reverse)
            step_delay: Delay between steps in seconds
        """
        # Set directions
        dir_a = 1 if steps_a >= 0 else -1
        dir_b = 1 if steps_b >= 0 else -1
        
        GPIO.output(self.motorA_dir, GPIO.HIGH if dir_a > 0 else GPIO.LOW)
        GPIO.output(self.motorB_dir, GPIO.HIGH if dir_b > 0 else GPIO.LOW)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        if max_steps == 0:
            return
            
        # Bresenham-style interpolation for synchronized stepping
        err_a = 0
        err_b = 0
        
        for _ in range(max_steps):
            if self.emergency_stop:
                break
                
            # Step motor A if needed
            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                GPIO.output(self.motorA_step, GPIO.HIGH)
                
            # Step motor B if needed
            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                GPIO.output(self.motorB_step, GPIO.HIGH)
                
            # Hold pulse high for minimum pulse width
            time.sleep(self.step_pulse_sec)
            
            # Release both step pins
            GPIO.output(self.motorA_step, GPIO.LOW)
            GPIO.output(self.motorB_step, GPIO.LOW)
            
            # Wait for step delay (controls speed)
            time.sleep(step_delay)
            
    def destroy_node(self):
        """Cleanup on shutdown."""
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
