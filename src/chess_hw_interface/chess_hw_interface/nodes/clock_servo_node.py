#!/usr/bin/env python3
"""
Clock Servo Node — controls the servo that physically presses the chess clock button.

Uses pigpio for hardware-timed PWM (microsecond pulse widths).
Per pins.yaml: clock_servo_pin=18, rest_pwm=500µs, hit_pwm=1500µs.

Services:
  /clock/hit (Trigger) — move servo to hit position, then return to rest

Subscribes:
  /emergency_stop (Bool) — disable servo on emergency
"""
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import pigpio
import time


class ClockServoNode(Node):
    def __init__(self):
        super().__init__('clock_servo_node')

        # Parameters
        # Parameters
        self.declare_parameter('clock_servo_pin', 18)
        self.declare_parameter('rest_pulse_us', 500)       # µs pulse width at rest
        self.declare_parameter('hit_pulse_us', 1000)       # µs pulse width for hitting
        self.declare_parameter('hit_duration', 0.3)    # seconds to hold

        self.servo_pin = self.get_parameter('clock_servo_pin').value
        # Handle potential float parsing if user forgets to update YAML (though we fixed it)
        self.rest_pw = int(self.get_parameter('rest_pulse_us').value)
        self.hit_pw = int(self.get_parameter('hit_pulse_us').value)
        self.hit_duration = self.get_parameter('hit_duration').value

        self.emergency_stop = False

        # pigpio setup
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error('Cannot connect to pigpiod! Is it running?')
            raise RuntimeError('pigpiod not running')

        # Set servo to rest position
        self.pi.set_servo_pulsewidth(self.servo_pin, self.rest_pw)
        self.get_logger().info(
            f'Clock Servo initialized on pin {self.servo_pin} '
            f'(rest={self.rest_pw}µs, hit={self.hit_pw}µs)')

        # Services
        self.create_service(Trigger, '/clock/hit', self.hit_callback)

        # Subscribers
        self.create_subscription(
            Bool, '/emergency_stop', self.stop_callback, 10)

    def stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn('EMERGENCY STOP: disabling clock servo')
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)  # stop pulses

    def hit_callback(self, request, response):
        if self.emergency_stop:
            response.success = False
            response.message = 'Failed: Emergency Stop Active'
            return response

        self.get_logger().info('Hitting clock...')

        # Move to hit position
        self.pi.set_servo_pulsewidth(self.servo_pin, self.hit_pw)
        time.sleep(self.hit_duration)

        # Return to rest
        self.pi.set_servo_pulsewidth(self.servo_pin, self.rest_pw)
        time.sleep(0.3)

        # Stop sending pulses to reduce jitter
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)

        response.success = True
        response.message = 'Clock hit successful'
        return response

    def destroy_node(self):
        if self.pi and self.pi.connected:
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)
            self.pi.stop()
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
