#!/usr/bin/env python3
"""
Board Detector Node — detects the chess board in camera frames and
publishes corner geometry for downstream piece detection.

Architecture: camera subscription stores latest raw message (no decode);
a 5fps timer picks up the latest frame, decodes it, and runs detection.
This avoids decoding 12fps frames when detection only needs 5fps.

Detection runs at reduced resolution (detection_scale=0.5 → 320x240 at
640x480 input) to keep Pi 4 CPU below 30%.

Published Topics:
  /perception/board_geometry  (chess_interfaces/BoardState) — 4 corners
  /perception/board_debug     (sensor_msgs/Image)           — annotated frame
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point

from chess_interfaces.msg import BoardState
from chess_perception.board_detection import BoardDetector


class BoardDetectorNode(Node):

    def __init__(self):
        super().__init__('board_detector_node')

        # At 640x480: scale=0.25 → 160x120. detect() takes ~40ms vs ~322ms at 320x240.
        # 2Hz detection: 2 * 40ms = 80ms/sec = ~8% CPU for detection itself.
        self.declare_parameter('detection_scale', 0.25)
        self.declare_parameter('detection_hz', 2.0)

        self._det_scale = float(self.get_parameter('detection_scale').value)
        det_hz          = float(self.get_parameter('detection_hz').value)

        self._detector     = BoardDetector()
        self._last_corners = None

        # Store latest compressed message — JPEG is ~30x smaller than raw,
        # avoiding 11MB/s DDS deserialization overhead per subscriber.
        self._latest_msg = None

        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_image, 10)

        self.debug_pub = self.create_publisher(
            Image, '/perception/board_debug', 10)
        self.geometry_pub = self.create_publisher(
            BoardState, '/perception/board_geometry', 10)

        # Timer drives detection at a controlled rate
        self.create_timer(1.0 / det_hz, self._detect_tick)

        self.get_logger().info(
            f'Board Detector Node started '
            f'(detection_scale={self._det_scale:.2f}, '
            f'detection_hz={det_hz:.1f})')

    # ─────────────────────────────────────────────────────────────────────
    # Subscription — store raw message, do NOT decode (no numpy)
    # ─────────────────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        self._latest_msg = msg  # Just store reference; timer decodes

    # ─────────────────────────────────────────────────────────────────────
    # Timer callback — decode + detect at controlled rate
    # ─────────────────────────────────────────────────────────────────────

    def _detect_tick(self):
        msg = self._latest_msg
        if msg is None:
            return

        # Decode JPEG — CompressedImage.data is already JPEG bytes
        try:
            buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            cv_image = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # Returns BGR
            if cv_image is None:
                self.get_logger().error('JPEG decode returned None')
                return
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        # Detect at reduced resolution
        if self._det_scale < 1.0:
            small    = cv2.resize(cv_image, None,
                                  fx=self._det_scale, fy=self._det_scale)
            geometry = self._detector.detect(small)
            if geometry is not None:
                geometry.corners = geometry.corners / self._det_scale
        else:
            geometry = self._detector.detect(cv_image)

        if geometry is not None:
            self._last_corners = geometry.corners.copy()

        # Publish geometry (current or cached)
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
            board_msg.corners = corners_list
            self.geometry_pub.publish(board_msg)

        # Publish debug image
        debug_img = self._detector.draw_debug(cv_image, geometry)
        debug_msg = Image()
        debug_msg.header      = msg.header
        debug_msg.height      = debug_img.shape[0]
        debug_msg.width       = debug_img.shape[1]
        debug_msg.encoding    = 'bgr8'
        debug_msg.is_bigendian = 0
        debug_msg.step        = debug_img.shape[1] * 3
        debug_msg.data        = debug_img.tobytes()
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BoardDetectorNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            # timeout_sec=None blocks until the next callback is ready (subscription
            # or timer fires). This eliminates sleep overhead entirely. On CycloneDDS/
            # ARM, this uses epoll internally and gives ~0% CPU when idle.
            executor.spin_once(timeout_sec=None)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
