#!/usr/bin/env python3
"""
Board Detector Node — detects the chess board in camera frames and
publishes corner geometry for downstream piece detection.

Uses the BoardDetector library class (board_detection.py) which tries
Hough-line detection first, then falls back to contour detection.

Performance: board detection is run at reduced resolution (detection_scale,
default 0.5) then corners are scaled back up to full resolution. This
reduces CPU from ~100% to ~25% on Pi 4 at 12fps with 1280x720 input.

Frame skipping: only every Nth frame is processed for board detection
(skip_frames, default 2). Corner cache ensures downstream gets continuous
geometry even on skipped frames.

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

        self.declare_parameter('detection_scale', 0.5)
        self.declare_parameter('skip_frames', 2)

        self._det_scale  = float(self.get_parameter('detection_scale').value)
        self._skip_n     = max(1, int(self.get_parameter('skip_frames').value))
        self._frame_num  = 0

        self._detector    = BoardDetector()
        self._last_corners = None  # Cached corners for temporal smoothing

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)

        self.debug_pub = self.create_publisher(
            Image, '/perception/board_debug', 10)
        self.geometry_pub = self.create_publisher(
            BoardState, '/perception/board_geometry', 10)

        self.get_logger().info(
            f'Board Detector Node started '
            f'(detection_scale={self._det_scale:.1f}, skip_frames={self._skip_n})')

    def image_callback(self, msg):
        self._frame_num += 1

        try:
            cv_image = np.array(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        # Run board detection on every Nth frame at reduced resolution.
        # Half-res detection reduces CPU from ~100% to ~25% on Pi 4.
        geometry = None
        if self._frame_num % self._skip_n == 0:
            if self._det_scale < 1.0:
                small = cv2.resize(cv_image, None,
                                   fx=self._det_scale, fy=self._det_scale)
                geometry = self._detector.detect(small)
                if geometry is not None:
                    geometry.corners = geometry.corners / self._det_scale
            else:
                geometry = self._detector.detect(cv_image)

            if geometry is not None:
                self._last_corners = geometry.corners.copy()

        # Publish geometry using current detection or cached corners
        corners = (geometry.corners if geometry is not None
                   else self._last_corners)

        if corners is not None:
            board_msg = BoardState()
            board_msg.header = msg.header
            corners_list = []
            for i in range(4):
                p = Point()
                p.x = float(corners[i][0])
                p.y = float(corners[i][1])
                corners_list.append(p)
            board_msg.corners = corners_list
            self.geometry_pub.publish(board_msg)

        # Publish debug image on every frame; draw_debug handles None gracefully
        debug_img = self._detector.draw_debug(cv_image, geometry)
        debug_msg = Image()
        debug_msg.header = msg.header
        debug_msg.height = debug_img.shape[0]
        debug_msg.width  = debug_img.shape[1]
        debug_msg.encoding    = 'bgr8'
        debug_msg.is_bigendian = 0
        debug_msg.step        = debug_img.shape[1] * 3
        debug_msg.data        = debug_img.tobytes()
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
