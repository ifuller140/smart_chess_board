#!/usr/bin/env python3
"""
Board Detector Node — detects the chess board in camera frames and
publishes corner geometry for downstream piece detection.

Uses the BoardDetector library class (board_detection.py) which tries
Hough-line detection first, then falls back to contour detection.
Corners are cached so downstream nodes see continuous updates even
when a frame fails detection (ARCH-04).

Published Topics:
  /perception/board_geometry  (chess_interfaces/BoardState) — 4 corners
  /perception/board_debug     (sensor_msgs/Image)           — annotated frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
import cv2
import numpy as np

from chess_interfaces.msg import BoardState
from chess_perception.board_detection import BoardDetector


class BoardDetectorNode(Node):

    def __init__(self):
        super().__init__('board_detector_node')

        self._detector = BoardDetector()
        self._last_corners = None  # Cached corners for temporal smoothing

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.debug_pub = self.create_publisher(
            Image, '/perception/board_debug', 10)
        self.geometry_pub = self.create_publisher(
            BoardState, '/perception/board_geometry', 10)

        self.get_logger().info('Board Detector Node started')

    def image_callback(self, msg):
        try:
            cv_image = np.array(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        geometry = self._detector.detect(cv_image)

        if geometry is not None:
            self._last_corners = geometry.corners  # Update cache on success

        # Publish with current or cached corners so downstream never starves
        corners = geometry.corners if geometry is not None else self._last_corners

        if corners is not None:
            board_msg = BoardState()
            board_msg.header = msg.header
            corners_list = []
            for i in range(4):
                p = Point()
                p.x = float(corners[i][0])
                p.y = float(corners[i][1])
                corners_list.append(p)
            board_msg.corners = corners_list  # Assign as list, not by index
            self.geometry_pub.publish(board_msg)

        # Always publish debug image (shows "No board detected" text when None)
        debug_img = self._detector.draw_debug(cv_image, geometry)
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = debug_img.shape[0]
        debug_msg.width = debug_img.shape[1]
        debug_msg.encoding = 'bgr8'
        debug_msg.is_bigendian = 0
        debug_msg.step = debug_img.shape[1] * 3
        debug_msg.data = debug_img.tobytes()
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
