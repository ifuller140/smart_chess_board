#!/usr/bin/env python3
"""
Limit Switch Node — publishes switch state via pigpio.

Limit switches are wired ACTIVE HIGH:
  - Pressed = 5V flows to GPIO → reads HIGH
  - Released = GPIO pulled LOW via internal pull-down

Publishes:
  /limit_switch/x_min  (Bool) — True when X limit pressed
  /limit_switch/y_min  (Bool) — True when Y limit pressed
  /limit_switch/clock_hit (Bool) — True when clock button pressed

Continuous polling at 50Hz + edge-triggered callbacks for responsiveness.

Requires:
- pigpio daemon running: sudo pigpiod
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

import pigpio


class LimitSwitchNode(Node):
    def __init__(self):
        super().__init__('limit_switch_node')

        # Parameters
        self.declare_parameter('limit_switch_pins.x_min', 10)
        self.declare_parameter('limit_switch_pins.y_min', 9)
        self.declare_parameter('limit_switch_pins.clock_hit', 15)
        self.declare_parameter('poll_rate_hz', 50.0)
        self.declare_parameter('debounce_us', 20000)  # 20ms debounce

        self.pin_x = self.get_parameter('limit_switch_pins.x_min').value
        self.pin_y = self.get_parameter('limit_switch_pins.y_min').value
        self.pin_clock = self.get_parameter('limit_switch_pins.clock_hit').value
        poll_rate = self.get_parameter('poll_rate_hz').value
        self.debounce_us = self.get_parameter('debounce_us').value

        # pigpio connection
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().fatal(
                "Cannot connect to pigpiod. Start with: sudo pigpiod")
            raise RuntimeError("pigpiod not running")

        # GPIO setup — active HIGH with pull-down
        for pin in [self.pin_x, self.pin_y, self.pin_clock]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)

        self.get_logger().info(
            f"Limit switches (active HIGH, PUD_DOWN): "
            f"X={self.pin_x}, Y={self.pin_y}, Clock={self.pin_clock}")

        # Publishers
        self.x_pub = self.create_publisher(Bool, '/limit_switch/x_min', 10)
        self.y_pub = self.create_publisher(Bool, '/limit_switch/y_min', 10)
        self.clock_pub = self.create_publisher(Bool, '/limit_switch/clock_hit', 10)

        # Edge-triggered callbacks (RISING = switch pressed)
        self._cb_x = self.pi.callback(
            self.pin_x, pigpio.RISING_EDGE, self._x_edge)
        self._cb_y = self.pi.callback(
            self.pin_y, pigpio.RISING_EDGE, self._y_edge)
        self._cb_clock = self.pi.callback(
            self.pin_clock, pigpio.RISING_EDGE, self._clock_edge)

        # Continuous polling timer
        self._poll_timer = self.create_timer(1.0 / poll_rate, self._poll)

        # Track last published state for edge-only logging
        self._last_x = False
        self._last_y = False
        self._last_clock = False

    def _x_edge(self, gpio, level, tick):
        self.get_logger().info("X Limit Hit (edge)")
        self.x_pub.publish(Bool(data=True))

    def _y_edge(self, gpio, level, tick):
        self.get_logger().info("Y Limit Hit (edge)")
        self.y_pub.publish(Bool(data=True))

    def _clock_edge(self, gpio, level, tick):
        self.get_logger().info("Clock Button Pressed (edge)")
        self.clock_pub.publish(Bool(data=True))

    def _poll(self):
        """Publish current switch state at poll rate."""
        x_state = self.pi.read(self.pin_x) == 1
        y_state = self.pi.read(self.pin_y) == 1
        clock_state = self.pi.read(self.pin_clock) == 1

        self.x_pub.publish(Bool(data=x_state))
        self.y_pub.publish(Bool(data=y_state))
        self.clock_pub.publish(Bool(data=clock_state))

        # Log transitions
        if x_state != self._last_x:
            self._last_x = x_state
            if x_state:
                self.get_logger().info("X limit: PRESSED")
        if y_state != self._last_y:
            self._last_y = y_state
            if y_state:
                self.get_logger().info("Y limit: PRESSED")
        if clock_state != self._last_clock:
            self._last_clock = clock_state
            if clock_state:
                self.get_logger().info("Clock: PRESSED")

    def destroy_node(self):
        # Cancel pigpio callbacks
        for cb in [self._cb_x, self._cb_y, self._cb_clock]:
            try:
                cb.cancel()
            except Exception:
                pass
        if self.pi.connected:
            self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LimitSwitchNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
