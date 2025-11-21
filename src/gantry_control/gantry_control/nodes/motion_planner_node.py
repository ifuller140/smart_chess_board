#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from gantry_control.action import MoveGantry
from std_srvs.srv import Trigger
from std_msgs.msg import String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')
        
        # Parameters
        self.declare_parameter('square_size_mm', 35.0)
        self.declare_parameter('board_origin_x', 20.0)
        self.declare_parameter('board_origin_y', 20.0)
        self.declare_parameter('travel_height_mm', 0.0) # Not used for 2D gantry but good for API
        
        self.sq_size = self.get_parameter('square_size_mm').value
        self.origin_x = self.get_parameter('board_origin_x').value
        self.origin_y = self.get_parameter('board_origin_y').value
        
        # Clients
        self._action_client = ActionClient(self, MoveGantry, '/gantry/move')
        self.servo_engage = self.create_client(Trigger, '/servo/engage')
        self.servo_release = self.create_client(Trigger, '/servo/release')
        
        # Subscribers (Command Interface)
        # For now, we listen to a simple string topic "e2e4" for testing
        # In real integration, GameManager calls a service or action here.
        self.move_sub = self.create_subscription(String, '/motion/command', self.command_callback, 10)
        
        self.get_logger().info("Motion Planner Started")

    def command_callback(self, msg):
        uci = msg.data
        if len(uci) >= 4:
            source = uci[:2]
            target = uci[2:4]
            self.get_logger().info(f"Executing move: {source} -> {target}")
            self.execute_move_sequence(source, target)

    def get_coords(self, square):
        # square is "e2", "a1" etc.
        col_map = {'a':0, 'b':1, 'c':2, 'd':3, 'e':4, 'f':5, 'g':6, 'h':7}
        col = col_map[square[0]]
        row = int(square[1]) - 1
        
        x = self.origin_x + (col * self.sq_size) + (self.sq_size / 2)
        y = self.origin_y + (row * self.sq_size) + (self.sq_size / 2)
        return x, y

    def execute_move_sequence(self, source, target):
        # 1. Move to Source
        sx, sy = self.get_coords(source)
        self.send_goal(sx, sy)
        
        # 2. Engage Magnet
        self.call_servo(self.servo_engage)
        
        # 3. Move to Target
        tx, ty = self.get_coords(target)
        self.send_goal(tx, ty)
        
        # 4. Release Magnet
        self.call_servo(self.servo_release)
        
        self.get_logger().info("Move Complete")

    def send_goal(self, x, y):
        goal_msg = MoveGantry.Goal()
        goal_msg.x_mm = float(x)
        goal_msg.y_mm = float(y)
        goal_msg.speed_mm_s = 50.0
        
        self._action_client.wait_for_server()
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        return res_future.result()

    def call_servo(self, client):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Servo service not available')
            return
        req = Trigger.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
