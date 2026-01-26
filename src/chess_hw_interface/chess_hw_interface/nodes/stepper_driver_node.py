#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import time
import sys

import RPi.GPIO as GPIO

class StepperDriverNode(Node):
    def __init__(self):
        super().__init__('stepper_driver_node')
        
        # Parameters
        self.declare_parameter('motorA_pins', [14, 4, 3, 2])
        self.declare_parameter('motorB_pins', [24, 23, 22, 27])
        self.declare_parameter('step_sequence', 'half')
        self.declare_parameter('step_delay_default', 0.001)
        
        self.motorA_pins = self.get_parameter('motorA_pins').value
        self.motorB_pins = self.get_parameter('motorB_pins').value
        self.seq_mode = self.get_parameter('step_sequence').value
        self.step_delay = self.get_parameter('step_delay_default').value

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        for pin in self.motorA_pins + self.motorB_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, 0)
        self.get_logger().info(f"GPIO Initialized. Motor A: {self.motorA_pins}, Motor B: {self.motorB_pins}")

        # Define Sequences
        if self.seq_mode == 'half':
            self.seq = [
                [1,0,0,0], [1,1,0,0], [0,1,0,0], [0,1,1,0],
                [0,0,1,0], [0,0,1,1], [0,0,0,1], [1,0,0,1]
            ]
        else: # full step
            self.seq = [
                [1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]
            ]
        self.step_count = len(self.seq)

        # Subscribers
        # Command format: x = steps motor A, y = steps motor B, z = unused
        self.subscription = self.create_subscription(
            Point,
            '/stepper/command',
            self.command_callback,
            10)
            
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10)
            
        # Publishers
        self.status_pub = self.create_publisher(String, '/stepper/status', 10)
        
        self.emergency_stop = False

    def stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP TRIGGERED")
            self.stop_motors()

    def stop_motors(self):
        for pin in self.motorA_pins + self.motorB_pins:
            GPIO.output(pin, 0)

    def command_callback(self, msg):
        if self.emergency_stop:
            self.get_logger().warn("Cannot move: Emergency Stop Active")
            return

        steps_a = int(msg.x)
        steps_b = int(msg.y)
        
        self.get_logger().info(f"Moving: A={steps_a}, B={steps_b}")
        self.move_motors(steps_a, steps_b)
        
        self.status_pub.publish(String(data="Idle"))

    def move_motors(self, steps_a, steps_b):
        # Determine direction
        dir_a = 1 if steps_a > 0 else -1
        dir_b = 1 if steps_b > 0 else -1
        
        steps_a = abs(steps_a)
        steps_b = abs(steps_b)
        
        max_steps = max(steps_a, steps_b)
        
        # Counters for Bresenham-like stepping if needed, 
        # but for now we just step them sequentially or interleaved.
        # Better approach for CoreXY: Step them as synchronously as possible.
        
        # Current step index in sequence
        idx_a = 0
        idx_b = 0
        
        for i in range(max_steps):
            if self.emergency_stop:
                break
                
            if i < steps_a:
                idx_a = (idx_a + dir_a) % self.step_count
                self.set_pins(self.motorA_pins, self.seq[idx_a])
                
            if i < steps_b:
                idx_b = (idx_b + dir_b) % self.step_count
                self.set_pins(self.motorB_pins, self.seq[idx_b])
                
            time.sleep(self.step_delay)
            
        # Turn off coils to save power/heat if holding torque not needed
        # For chess board, we might want to hold if there is tension, but usually 28BYJ hold well enough with friction
        # self.stop_motors() 

    def set_pins(self, pins, values):
        for pin, val in zip(pins, values):
            GPIO.output(pin, val)

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
