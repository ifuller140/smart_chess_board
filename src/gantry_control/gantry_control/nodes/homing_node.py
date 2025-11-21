#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import time

class HomingNode(Node):
    def __init__(self):
        super().__init__('homing_node')
        
        # Subscribers
        self.x_limit_sub = self.create_subscription(Bool, '/limit_switch/x_min', self.x_callback, 10)
        self.y_limit_sub = self.create_subscription(Bool, '/limit_switch/y_min', self.y_callback, 10)
        
        # Publishers
        self.stepper_pub = self.create_publisher(Point, '/stepper/command', 10)
        
        self.homed_x = False
        self.homed_y = False
        
        self.get_logger().info("Homing Node Ready. Run 'ros2 run gantry_control homing_node' to home.")
        
        # Start Homing Sequence
        self.timer = self.create_timer(1.0, self.start_homing)

    def start_homing(self):
        self.timer.cancel()
        self.get_logger().info("Starting Homing Sequence...")
        
        # Move X towards 0
        # This is a naive implementation. 
        # Real homing requires careful step-by-step movement until switch hit.
        # Since our stepper driver is open-loop blocking, we can't easily interrupt it 
        # unless we change the driver to be non-blocking or step-by-step.
        # For this architecture, we assume the driver handles 'move until stop' or we send small increments.
        
        self.get_logger().info("Moving towards X min...")
        # Send negative steps
        msg = Point()
        msg.x = -1000.0 # Large number of steps
        msg.y = -1000.0
        self.stepper_pub.publish(msg)
        
        # In a real system, we'd monitor the limit switch here and stop.
        # Since we are mocking, we assume the limit switch node triggers 
        # and we handle it there or via a specialized homing service in the driver.

    def x_callback(self, msg):
        if msg.data and not self.homed_x:
            self.get_logger().info("X Homing Complete")
            self.homed_x = True
            # Stop X motion

    def y_callback(self, msg):
        if msg.data and not self.homed_y:
            self.get_logger().info("Y Homing Complete")
            self.homed_y = True
            # Stop Y motion

def main(args=None):
    rclpy.init(args=args)
    node = HomingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
