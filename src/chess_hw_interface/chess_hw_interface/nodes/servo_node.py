#!/usr/bin/env python3
"""
Servo Node — controls magnet servo via pigpio hardware PWM.

Uses pigpio's set_servo_pulsewidth() for jitter-free hardware-timed
servo control (no software PWM).

Services:
  /servo/engage  (Trigger) — drag position, magnet actuates a piece
  /servo/release (Trigger) — clear position, magnet does not interact with pieces

Positions are calibrated in degrees (see code/test_z_servo.py for the
interactive sweep used to find them) and converted to a pigpio pulse
width with the same 500-2500us / 0-180deg mapping as that script.

Requires:
- pigpio daemon running: sudo pigpiod
"""
import time

import pigpio
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool

# SG90 pulse-width range, matches code/test_z_servo.py's calibration sweep
MIN_PULSE_US = 500
MAX_PULSE_US = 2500


def angle_to_pulsewidth(degrees: float) -> int:
    """Convert a 0-180 degree servo angle to a pigpio pulse width in microseconds."""
    return int(MIN_PULSE_US + (degrees / 180.0) * (MAX_PULSE_US - MIN_PULSE_US))


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Parameters
        self.declare_parameter('servo_pin', 12)
        self.declare_parameter('engage_angle_deg', 145.0)   # Drag position — magnet actuates piece
        self.declare_parameter('release_angle_deg', 170.0)  # Clear position — no piece interaction
        self.declare_parameter('movement_time', 0.5)

        self.servo_pin = self.get_parameter('servo_pin').value
        self.engage_pw = angle_to_pulsewidth(self.get_parameter('engage_angle_deg').value)
        self.release_pw = angle_to_pulsewidth(self.get_parameter('release_angle_deg').value)
        self.move_time = self.get_parameter('movement_time').value

        self.current_state = "unknown"
        self.emergency_stop = False

        # pigpio connection
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod. Start with: sudo pigpiod")
            raise RuntimeError("pigpiod not running")

        # Initialize servo pin
        self.pi.set_mode(self.servo_pin, pigpio.OUTPUT)
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)  # Stop sending pulses

        self.get_logger().info(
            f"Servo initialized on pin {self.servo_pin} "
            f"(engage={self.get_parameter('engage_angle_deg').value}deg/{self.engage_pw}us, "
            f"release={self.get_parameter('release_angle_deg').value}deg/{self.release_pw}us)")

        # Callback groups: /emergency_stop must be able to preempt a
        # blocking engage/release call, so it lives in its own group.
        self._estop_cb_group = ReentrantCallbackGroup()
        self._servo_cb_group = MutuallyExclusiveCallbackGroup()

        # Services
        self.engage_srv = self.create_service(
            Trigger, '/servo/engage', self.engage_callback,
            callback_group=self._servo_cb_group)
        self.release_srv = self.create_service(
            Trigger, '/servo/release', self.release_callback,
            callback_group=self._servo_cb_group)

        # Subscribers
        self.stop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.stop_callback, 10,
            callback_group=self._estop_cb_group)

        # Publishers
        self.state_pub = self.create_publisher(String, '/servo/state', 10)

    def stop_callback(self, msg):
        if msg.data:
            self.emergency_stop = True
            self.get_logger().warn("EMERGENCY STOP: Stopping servo")
            self.pi.set_servo_pulsewidth(self.servo_pin, 0)

    def set_servo(self, pulse_width_us: int) -> bool:
        """Set servo position using hardware-timed PWM."""
        if self.emergency_stop:
            return False

        self.pi.set_servo_pulsewidth(self.servo_pin, pulse_width_us)
        time.sleep(self.move_time)
        # Stop sending pulses to avoid jitter
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        return True

    def engage_callback(self, request, response):
        self.get_logger().info("Engaging Magnet (Down)")
        success = self.set_servo(self.engage_pw)

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
        success = self.set_servo(self.release_pw)

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
        self.pi.set_servo_pulsewidth(self.servo_pin, 0)
        if self.pi.connected:
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
