#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from gantry_control.action import MoveGantry
from std_msgs.msg import Bool
import time
import math

class GantryKinematicsNode(Node):
    def __init__(self):
        super().__init__('gantry_kinematics_node')
        
        # Parameters
        self.declare_parameter('steps_per_mm', 10.0) # Calibrate this!
        self.declare_parameter('max_speed_mm_s', 50.0)
        self.declare_parameter('x_max', 300.0)
        self.declare_parameter('y_max', 300.0)
        
        self.steps_per_mm = self.get_parameter('steps_per_mm').value
        self.max_speed = self.get_parameter('max_speed_mm_s').value
        self.x_max = self.get_parameter('x_max').value
        self.y_max = self.get_parameter('y_max').value
        
        # Current State
        self.current_x = 0.0
        self.current_y = 0.0
        
        # Publishers / Subscribers
        self.stepper_pub = self.create_publisher(Point, '/stepper/command', 10)
        self.pose_pub = self.create_publisher(Point, '/gantry/pose', 10)
        
        # Action Server
        self._action_server = ActionServer(
            self,
            MoveGantry,
            '/gantry/move',
            self.execute_callback)
            
        self.get_logger().info("Gantry Kinematics Node Started")

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        goal = goal_handle.request
        
        # 1. Validate Goal
        if not (0 <= goal.x_mm <= self.x_max) or not (0 <= goal.y_mm <= self.y_max):
            goal_handle.abort()
            result = MoveGantry.Result()
            result.success = False
            result.message = "Target out of bounds"
            return result

        # 2. Calculate Delta
        dx_mm = goal.x_mm - self.current_x
        dy_mm = goal.y_mm - self.current_y
        
        # 3. CoreXY Kinematics (mm -> steps)
        # A = X + Y
        # B = X - Y
        # Delta A_steps = (dx + dy) * steps_per_mm
        # Delta B_steps = (dx - dy) * steps_per_mm
        
        steps_a = int((dx_mm + dy_mm) * self.steps_per_mm)
        steps_b = int((dx_mm - dy_mm) * self.steps_per_mm)
        
        # 4. Send Command to Stepper Driver
        # Note: This is a simplified blocking approach. 
        # In a real system, we'd want feedback from the stepper driver 
        # or calculate exact timing here.
        
        cmd = Point()
        cmd.x = float(steps_a)
        cmd.y = float(steps_b)
        self.stepper_pub.publish(cmd)
        
        # 5. Wait for Completion (Open Loop)
        # Calculate expected time based on speed
        distance = math.sqrt(dx_mm**2 + dy_mm**2)
        speed = goal.speed_mm_s if goal.speed_mm_s > 0 else self.max_speed
        duration = distance / speed
        
        # Feedback loop
        start_time = time.time()
        while (time.time() - start_time) < duration:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveGantry.Result(success=False, message="Canceled")
            
            elapsed = time.time() - start_time
            progress = elapsed / duration
            
            feedback = MoveGantry.Feedback()
            feedback.percent_complete = progress * 100.0
            # Estimate current position
            feedback.current_x_mm = self.current_x + (dx_mm * progress)
            feedback.current_y_mm = self.current_y + (dy_mm * progress)
            goal_handle.publish_feedback(feedback)
            
            time.sleep(0.1)
            
        # 6. Update State
        self.current_x = goal.x_mm
        self.current_y = goal.y_mm
        
        # Publish final pose
        pose = Point()
        pose.x = self.current_x
        pose.y = self.current_y
        self.pose_pub.publish(pose)

        goal_handle.succeed()
        
        result = MoveGantry.Result()
        result.success = True
        result.message = "Arrived"
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
