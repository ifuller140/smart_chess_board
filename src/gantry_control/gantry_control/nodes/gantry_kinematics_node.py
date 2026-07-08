#!/usr/bin/env python3
"""
Gantry Kinematics Node.

Translates mm-based Cartesian goals into CoreXY motor step commands,
tracks actual position via step counting (subscribed from stepper driver),
and serves the /gantry/move action used by the motion planner.

Position is tracked by the stepper_driver_node (step counting) and
published to /gantry/pose. This node subscribes to that topic to keep
current_x / current_y accurate at all times — even after limit stops,
emergency stops, and partial moves.

Architecture:
  MotionPlanner → [/gantry/move action] → GantryKinematics
  GantryKinematics → [/stepper/velocity Twist] → StepperDriver
  StepperDriver → [/gantry/pose Point] → GantryKinematics (feedback)
"""

import math
import time

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import Bool, String

from chess_interfaces.action import MoveGantry


class GantryKinematicsNode(Node):

    def __init__(self):
        super().__init__('gantry_kinematics_node')

        # Parameters
        self.declare_parameter('steps_per_mm', 5.0)
        self.declare_parameter('max_speed_mm_s', 60.0)
        self.declare_parameter('acceleration_mm_s2', 80.0)
        self.declare_parameter('x_max_mm', 250.0)
        self.declare_parameter('y_max_mm', 250.0)
        self.declare_parameter('position_tolerance_mm', 1.5)

        self.steps_per_mm = float(self.get_parameter('steps_per_mm').value)
        self.max_speed_mm_s = float(self.get_parameter('max_speed_mm_s').value)
        self.accel_mm_s2 = float(self.get_parameter('acceleration_mm_s2').value)
        self.x_max = float(self.get_parameter('x_max_mm').value)
        self.y_max = float(self.get_parameter('y_max_mm').value)
        self.pos_tol = float(self.get_parameter('position_tolerance_mm').value)

        # Current position — updated from /gantry/pose (step-counted)
        self.current_x = 0.0
        self.current_y = 0.0
        self._pose_received = False

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/stepper/velocity', 10)

        # Subscribers
        self.create_subscription(Point, '/gantry/pose', self._pose_cb, 10)
        self.create_subscription(Bool, '/limit_switch/x_min', self._x_limit_cb, 10)
        self.create_subscription(Bool, '/limit_switch/y_min', self._y_limit_cb, 10)

        self._x_limit = False
        self._y_limit = False

        # Action server
        self._action_server = ActionServer(
            self,
            MoveGantry,
            '/gantry/move',
            execute_callback=self.execute_callback,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info(
            f'Gantry Kinematics ready: max={self.max_speed_mm_s}mm/s, '
            f'accel={self.accel_mm_s2}mm/s², tol={self.pos_tol}mm'
        )

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _pose_cb(self, msg: Point):
        """Update current position from step-counted pose published by stepper driver."""
        self.current_x = msg.x
        self.current_y = msg.y
        self._pose_received = True

    def _x_limit_cb(self, msg: Bool):
        self._x_limit = msg.data

    def _y_limit_cb(self, msg: Bool):
        self._y_limit = msg.data

    # ------------------------------------------------------------------
    # Action server
    # ------------------------------------------------------------------

    def _goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info('Gantry move cancel requested')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute a Cartesian move to (x_mm, y_mm).

        Uses a trapezoidal velocity profile:
          - Accelerate to cruise speed over the first ~30% of distance
          - Cruise at max speed
          - Decelerate into the target

        Publishes velocity commands to /stepper/velocity.
        Monitors /gantry/pose for actual position.
        """
        goal = goal_handle.request
        target_x = float(goal.target_x_mm)
        target_y = float(goal.target_y_mm)
        requested_speed = float(goal.speed_mm_s) if goal.speed_mm_s > 0 else self.max_speed_mm_s
        cruise_speed = min(requested_speed, self.max_speed_mm_s)

        # Bounds check
        if not (0.0 <= target_x <= self.x_max) or not (0.0 <= target_y <= self.y_max):
            goal_handle.abort()
            result = MoveGantry.Result()
            result.success = False
            result.message = f'Target ({target_x:.1f}, {target_y:.1f}) out of bounds'
            self.get_logger().error(result.message)
            return result

        self.get_logger().info(
            f'Moving: ({self.current_x:.1f}, {self.current_y:.1f}) → '
            f'({target_x:.1f}, {target_y:.1f}) at {cruise_speed:.0f}mm/s'
        )

        result = MoveGantry.Result()
        try:
            self._execute_trapezoidal_move(goal_handle, target_x, target_y, cruise_speed)
            goal_handle.succeed()
            result.success = True
            result.message = f'Arrived at ({self.current_x:.1f}, {self.current_y:.1f})'
        except Exception as e:
            self._stop()
            goal_handle.abort()
            result.success = False
            result.message = str(e)

        return result

    def _execute_trapezoidal_move(self, goal_handle, target_x, target_y, cruise_speed_mm_s):
        """
        Drive gantry to target using a trapezoidal velocity profile.

        Publishes Twist on /stepper/velocity. The stepper_driver_node's
        velocity tick handles all acceleration ramping internally.
        Here we manage the high-level profile: ramp up while far from target,
        slow down as we approach.
        """
        POLL_RATE_S = 0.05   # 20Hz position check loop

        # Total distance (for feedback percentage)
        dx0 = target_x - self.current_x
        dy0 = target_y - self.current_y
        total_dist = math.sqrt(dx0 * dx0 + dy0 * dy0)
        if total_dist < self.pos_tol:
            return  # Already there

        # Minimum ramp distance to reach full cruise speed from accel:
        # v² = 2 * a * d  →  d = v² / (2a)
        ramp_dist = (cruise_speed_mm_s ** 2) / (2.0 * self.accel_mm_s2)

        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                self._stop()
                goal_handle.canceled()
                return

            dx = target_x - self.current_x
            dy = target_y - self.current_y
            dist = math.sqrt(dx * dx + dy * dy)

            # Check arrival
            if dist <= self.pos_tol:
                break

            # Compute velocity direction (unit vector in Cartesian space)
            ux = dx / dist
            uy = dy / dist

            # Speed profile: slow down when within ramp_dist of target
            if dist < ramp_dist:
                # Deceleration zone: speed proportional to remaining distance
                speed = cruise_speed_mm_s * (dist / ramp_dist)
                speed = max(speed, 5.0)   # floor at 5mm/s (avoid stall at zero)
            else:
                # Acceleration / cruise: let stepper driver's ramp handle it
                speed = cruise_speed_mm_s

            # Convert mm/s Cartesian → steps/sec Cartesian
            # (stepper driver velocity topic units = steps/sec in Cartesian)
            vx_steps = ux * speed * self.steps_per_mm
            vy_steps = uy * speed * self.steps_per_mm

            self._publish_velocity(vx_steps, vy_steps)

            # Publish feedback with correct percentage
            progress_pct = max(0.0, min(100.0, (1.0 - dist / total_dist) * 100.0))
            feedback = MoveGantry.Feedback()
            feedback.percent_complete = progress_pct
            feedback.current_x_mm = self.current_x
            feedback.current_y_mm = self.current_y
            goal_handle.publish_feedback(feedback)

            time.sleep(POLL_RATE_S)

        # Arrived — stop motors
        self._stop()
        self.get_logger().info(
            f'Arrived at ({self.current_x:.1f}, {self.current_y:.1f}), '
            f'target was ({target_x:.1f}, {target_y:.1f})'
        )

    def _publish_velocity(self, vx: float, vy: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        self.vel_pub.publish(msg)

    def _stop(self):
        self._publish_velocity(0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = GantryKinematicsNode()
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
