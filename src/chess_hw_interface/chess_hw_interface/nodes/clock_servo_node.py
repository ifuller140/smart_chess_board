#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
import time

import RPi.GPIO as GPIO

class ClockServoNode(Node):
    def __init__(self):
        super().__init__('clock_servo_node')
        
        # Parameters
        self.declare_parameter('clock_servo_pin', 18)
        self.declare_parameter('rest_pwm', 2.5)
        self.declare_parameter('hit_pwm', 7.5)
        self.declare_parameter('hit_duration', 0.3)
        
        self.servo_pin = self.get_parameter('clock_servo_pin').value
        self.rest_val = self.get_parameter('rest_pwm').value
        self.hit_val = self.get_parameter('hit_pwm').value
        self.hit_duration = self.get_parameter('hit_duration').value
        
        self.pwm = None
        self.current_state = "rest"
        self.emergency_stop = False

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50) # 50Hz standard
        self.pwm.start(0)
        self.set_servo(self.rest_val)
        self.get_logger().info(f"Clock Servo Initialized on Pin {self.servo_pin}")

        # Services
        self.hit_srv = self.create_service(Trigger, '/clock/hit', self.hit_callback)
        
        # Subscribers
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10)
            
    def stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP: Disabling Clock Servo")
            if self.pwm:
                self.pwm.ChangeDutyCycle(0)

    def set_servo(self, duty_cycle, duration=0.5):
        if self.emergency_stop:
            return False
            
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(duration)
        self.pwm.ChangeDutyCycle(0) # Stop sending pulses
            
        return True

    def hit_callback(self, request, response):
        self.get_logger().info("Hitting Clock")
        
        # Move to hit position
        success = self.set_servo(self.hit_val, self.hit_duration)
        
        if success:
            # Return to rest
            self.set_servo(self.rest_val, 0.5)
            response.success = True
            response.message = "Clock Hit Successful"
        else:
            response.success = False
            response.message = "Failed: Emergency Stop Active"
            
        return response

    def destroy_node(self):
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ClockServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
