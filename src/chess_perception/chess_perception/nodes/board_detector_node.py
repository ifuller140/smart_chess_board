#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from chess_interfaces.msg import BoardState

class BoardDetectorNode(Node):
    def __init__(self):
        super().__init__('board_detector_node')
        
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        # Publishers
        self.debug_pub = self.create_publisher(Image, '/perception/board_debug', 10)
        self.geometry_pub = self.create_publisher(BoardState, '/perception/board_geometry', 10) # Using BoardState for convenience, mainly for corners
        
        self.get_logger().info("Board Detector Node Started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        # 1. Preprocessing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # 2. Find Contours (looking for the board square)
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        largest_contour = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 5000: # Minimum area filter
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                if len(approx) == 4:
                    if area > max_area:
                        max_area = area
                        largest_contour = approx

        if largest_contour is not None:
            # Draw debug
            cv2.drawContours(cv_image, [largest_contour], -1, (0, 255, 0), 3)
            
            # Sort points: top-left, top-right, bottom-right, bottom-left
            pts = largest_contour.reshape(4, 2)
            rect = np.zeros((4, 2), dtype="float32")
            
            s = pts.sum(axis=1)
            rect[0] = pts[np.argmin(s)] # TL
            rect[2] = pts[np.argmax(s)] # BR
            
            diff = np.diff(pts, axis=1)
            rect[1] = pts[np.argmin(diff)] # TR
            rect[3] = pts[np.argmax(diff)] # BL
            
            # Publish Geometry
            board_msg = BoardState()
            board_msg.header = msg.header
            
            for i in range(4):
                p = Point()
                p.x = float(rect[i][0])
                p.y = float(rect[i][1])
                board_msg.corners[i] = p
                
            self.geometry_pub.publish(board_msg)
            
        # Publish Debug Image
        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoardDetectorNode()
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
