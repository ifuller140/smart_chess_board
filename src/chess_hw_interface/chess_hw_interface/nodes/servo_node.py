#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String, Bool
import time

import RPi.GPIO as GPIO

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        
        # Parameters
        self.declare_parameter('servo_pin', 12)
        self.declare_parameter('engage_pwm', 2.5)
        self.declare_parameter('release_pwm', 7.5)
        self.declare_parameter('movement_time', 0.5)
        
        self.servo_pin = self.get_parameter('servo_pin').value
        self.engage_val = self.get_parameter('engage_pwm').value
        self.release_val = self.get_parameter('release_pwm').value
        self.move_time = self.get_parameter('movement_time').value
        
        self.pwm = None
        self.current_state = "unknown"

        # GPIO Setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.servo_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.servo_pin, 50) # 50Hz standard for servos
        self.pwm.start(0)
        self.get_logger().info(f"Servo Initialized on Pin {self.servo_pin}")

        # Services
        self.engage_srv = self.create_service(Trigger, '/servo/engage', self.engage_callback)
        self.release_srv = self.create_service(Trigger, '/servo/release', self.release_callback)
        
        # Subscribers
        self.stop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.stop_callback,
            10)
            
        # Publishers
        self.state_pub = self.create_publisher(String, '/servo/state', 10)
        
        self.emergency_stop = False

    def stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP: Disabling Servo")
            if self.pwm:
                self.pwm.ChangeDutyCycle(0)

    def set_servo(self, duty_cycle):
        if self.emergency_stop:
            return False
            
        self.pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(self.move_time)
        self.pwm.ChangeDutyCycle(0) # Stop sending pulses to prevent jitter
            
        return True

    def engage_callback(self, request, response):
        self.get_logger().info("Engaging Magnet (Down)")
        success = self.set_servo(self.engage_val)
        
        if success:
            self.current_state = "engaged"
            response.success = True
            response.message = "Magnet Engaged"
        else:
            response.success = False
            response.message = "Failed: Emergency Stop Active"
            
        self.state_pub.publish(String(data=self.current_state))
        return response

    def release_callback(self, request, response):
        self.get_logger().info("Releasing Magnet (Up)")
        success = self.set_servo(self.release_val)
        
        if success:
            self.current_state = "released"
            response.success = True
            response.message = "Magnet Released"
        else:
            response.success = False
            response.message = "Failed: Emergency Stop Active"
            
        self.state_pub.publish(String(data=self.current_state))
        return response

    def destroy_node(self):
        if self.pwm:
            self.pwm.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
