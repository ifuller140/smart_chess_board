#!/usr/bin/env python3
"""
GPIO Watchdog Node — safety monitor for the gantry system.

Monitors limit switch states and stepper status. If a limit switch triggers
during active gantry movement, publishes /emergency_stop to halt all motors.

The emergency stop is latched — only /watchdog/reset can clear it.

Subscribes:
  /limit_switch/x_min  (Bool)  — X-axis limit switch
  /limit_switch/y_min  (Bool)  — Y-axis limit switch
  /stepper/status      (String) — stepper driver state (IDLE / MOVING)

Publishes:
  /emergency_stop      (Bool)  — latched stop signal
  /watchdog/status     (String) — heartbeat + health

Services:
  /watchdog/reset      (Trigger) — clear latched emergency stop
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class GPIOWatchdogNode(Node):
    def __init__(self):
        super().__init__('gpio_watchdog_node')

        # State
        self.estop_latched = False
        self.x_limit_hit = False
        self.y_limit_hit = False
        self.stepper_state = 'IDLE'
        self.topics_alive = {'x_limit': False, 'y_limit': False, 'stepper': False}

        # Publishers
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.status_pub = self.create_publisher(String, '/watchdog/status', 10)

        # Subscribers
        self.create_subscription(
            Bool, '/limit_switch/x_min', self.x_limit_cb, 10)
        self.create_subscription(
            Bool, '/limit_switch/y_min', self.y_limit_cb, 10)
        self.create_subscription(
            String, '/stepper/status', self.stepper_cb, 10)

        # Services
        self.create_service(Trigger, '/watchdog/reset', self.reset_cb)

        # Heartbeat timer (1Hz)
        self.create_timer(1.0, self.heartbeat)

        # E-stop publish timer (10Hz — fast reaction)
        self.create_timer(0.1, self.publish_estop)

        self.get_logger().info('GPIO Watchdog Node started — monitoring limit switches')

    # ── Callbacks ──────────────────────────────────────────────────────

    def x_limit_cb(self, msg):
        self.topics_alive['x_limit'] = True
        was_hit = self.x_limit_hit
        self.x_limit_hit = msg.data

        if msg.data and not was_hit:
            self.get_logger().info('X limit switch triggered')
            self._check_safety('X')

    def y_limit_cb(self, msg):
        self.topics_alive['y_limit'] = True
        was_hit = self.y_limit_hit
        self.y_limit_hit = msg.data

        if msg.data and not was_hit:
            self.get_logger().info('Y limit switch triggered')
            self._check_safety('Y')

    def stepper_cb(self, msg):
        self.topics_alive['stepper'] = True
        self.stepper_state = msg.data

    def _check_safety(self, axis):
        """If a limit switch fires while the gantry is moving, trigger e-stop."""
        if self.stepper_state == 'MOVING' and not self.estop_latched:
            self.get_logger().error(
                f'SAFETY: {axis} limit hit during MOVING — EMERGENCY STOP')
            self.estop_latched = True

    # ── Service ────────────────────────────────────────────────────────

    def reset_cb(self, request, response):
        if self.x_limit_hit or self.y_limit_hit:
            response.success = False
            response.message = (
                f'Cannot reset: limit switch still active '
                f'(X={self.x_limit_hit}, Y={self.y_limit_hit}). '
                f'Move gantry off switch first.')
            self.get_logger().warn(response.message)
        else:
            self.estop_latched = False
            response.success = True
            response.message = 'Emergency stop cleared'
            self.get_logger().info('Emergency stop cleared via /watchdog/reset')
        return response

    # ── Timers ─────────────────────────────────────────────────────────

    def publish_estop(self):
        """Publish current e-stop state at 10Hz."""
        msg = Bool()
        msg.data = self.estop_latched
        self.estop_pub.publish(msg)

    def heartbeat(self):
        """1Hz status heartbeat."""
        dead = [k for k, v in self.topics_alive.items() if not v]
        if self.estop_latched:
            status = 'EMERGENCY_STOP'
        elif dead:
            status = f'WARN:no_data_from:{",".join(dead)}'
        else:
            status = 'OK'

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPIOWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
