#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import Point
from rclpy.action import ActionServer
from rclpy.node import Node

from gantry_control.action import MoveGantry


class GantryKinematicsNode(Node):
    def __init__(self):
        super().__init__('gantry_kinematics_node')

        self.declare_parameter('steps_per_mm', 5.0)
        self.declare_parameter('max_speed_mm_s', 50.0)
        self.declare_parameter('x_max', 300.0)
        self.declare_parameter('y_max', 300.0)

        self.steps_per_mm = float(self.get_parameter('steps_per_mm').value)
        self.max_speed = float(self.get_parameter('max_speed_mm_s').value)
        self.x_max = float(self.get_parameter('x_max').value)
        self.y_max = float(self.get_parameter('y_max').value)

        self.current_x = 0.0
        self.current_y = 0.0

        self.stepper_pub = self.create_publisher(Point, '/stepper/command', 10)
        self.pose_pub = self.create_publisher(Point, '/gantry/pose', 10)

        self._action_server = ActionServer(
            self,
            MoveGantry,
            '/gantry/move',
            self.execute_callback,
        )

        self.get_logger().info('Gantry Kinematics Node started')

    def execute_callback(self, goal_handle):
        goal = goal_handle.request

        if not (0.0 <= goal.x_mm <= self.x_max) or not (0.0 <= goal.y_mm <= self.y_max):
            goal_handle.abort()
            result = MoveGantry.Result()
            result.success = False
            result.message = 'Target out of bounds'
            return result

        dx_mm = goal.x_mm - self.current_x
        dy_mm = goal.y_mm - self.current_y

        # CoreXY map for documented physical layout:
        # +X => A+, B- ; +Y => A+, B+
        # A = dx + dy ; B = dy - dx
        steps_a = int((dx_mm + dy_mm) * self.steps_per_mm)
        steps_b = int((dy_mm - dx_mm) * self.steps_per_mm)

        requested_speed = goal.speed_mm_s if goal.speed_mm_s > 0 else self.max_speed
        speed_mm_s = max(0.1, min(self.max_speed, requested_speed))
        speed_percent = max(1.0, min(100.0, (speed_mm_s / self.max_speed) * 100.0))

        cmd = Point()
        cmd.x = float(steps_a)
        cmd.y = float(steps_b)
        cmd.z = float(speed_percent)
        self.stepper_pub.publish(cmd)

        distance = math.sqrt(dx_mm**2 + dy_mm**2)
        duration = max(0.05, distance / speed_mm_s) if distance > 0 else 0.05

        start_time = time.time()
        while (time.time() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = MoveGantry.Result()
                result.success = False
                result.message = 'Canceled'
                return result

            elapsed = time.time() - start_time
            progress = min(1.0, elapsed / duration)

            feedback = MoveGantry.Feedback()
            feedback.percent_complete = progress * 100.0
            feedback.current_x_mm = self.current_x + (dx_mm * progress)
            feedback.current_y_mm = self.current_y + (dy_mm * progress)
            goal_handle.publish_feedback(feedback)

            time.sleep(0.05)

        self.current_x = goal.x_mm
        self.current_y = goal.y_mm

        pose = Point()
        pose.x = self.current_x
        pose.y = self.current_y
        self.pose_pub.publish(pose)

        goal_handle.succeed()
        result = MoveGantry.Result()
        result.success = True
        result.message = 'Arrived'
        return result


def main(args=None):
    rclpy.init(args=args)
    node = GantryKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
