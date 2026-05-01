#!/usr/bin/env python3
"""
Board Detector Node — detects the chess board in camera frames and
publishes corner geometry for downstream piece detection.

Detection runs in a background thread so the ROS callback never blocks
the executor. The latest frame is always available to the detector;
intermediate frames are dropped when the detector is busy (no queue).

Detection runs at reduced resolution (detection_scale=0.5 → 640x360 at
1280x720 input) to keep CPU below 25% on Pi 4.

Published Topics:
  /perception/board_geometry  (chess_interfaces/BoardState) — 4 corners
  /perception/board_debug     (sensor_msgs/Image)           — annotated frame
"""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from chess_interfaces.msg import BoardState
from chess_perception.board_detection import BoardDetector


class BoardDetectorNode(Node):

    def __init__(self):
        super().__init__('board_detector_node')

        self.declare_parameter('detection_scale', 0.5)
        self._det_scale = float(self.get_parameter('detection_scale').value)

        self._detector      = BoardDetector()
        self._last_corners  = None
        self._last_geometry = None  # Most recent successful BoardGeometry

        # Thread-safe frame handoff: callback writes, detector thread reads
        self._pending_frame  = None
        self._pending_header = None
        self._frame_lock     = threading.Lock()
        self._frame_event    = threading.Event()
        self._shutdown       = threading.Event()

        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)

        self.debug_pub = self.create_publisher(
            Image, '/perception/board_debug', 10)
        self.geometry_pub = self.create_publisher(
            BoardState, '/perception/board_geometry', 10)

        # Start background detection thread
        self._detect_thread = threading.Thread(
            target=self._detect_loop, daemon=True)
        self._detect_thread.start()

        self.get_logger().info(
            f'Board Detector Node started '
            f'(detection_scale={self._det_scale:.1f}, threaded)')

    # ─────────────────────────────────────────────────────────────────────
    # ROS callback — just stores latest frame, never blocks
    # ─────────────────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        try:
            frame = np.array(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        with self._frame_lock:
            self._pending_frame  = frame
            self._pending_header = msg.header
        self._frame_event.set()

    # ─────────────────────────────────────────────────────────────────────
    # Background detection loop
    # ─────────────────────────────────────────────────────────────────────

    def _detect_loop(self):
        while not self._shutdown.is_set():
            # Wait for a new frame (timeout 1s to allow shutdown check)
            if not self._frame_event.wait(timeout=1.0):
                continue
            self._frame_event.clear()

            with self._frame_lock:
                frame  = self._pending_frame
                header = self._pending_header

            if frame is None:
                continue

            # Detect at reduced resolution
            if self._det_scale < 1.0:
                small = cv2.resize(frame, None,
                                   fx=self._det_scale, fy=self._det_scale)
                geometry = self._detector.detect(small)
                if geometry is not None:
                    geometry.corners = geometry.corners / self._det_scale
            else:
                geometry = self._detector.detect(frame)

            if geometry is not None:
                self._last_corners  = geometry.corners.copy()
                self._last_geometry = geometry

            # Publish geometry (current or cached)
            corners = (geometry.corners if geometry is not None
                       else self._last_corners)
            if corners is not None:
                board_msg = BoardState()
                board_msg.header = header
                corners_list = []
                for i in range(4):
                    p = Point()
                    p.x = float(corners[i][0])
                    p.y = float(corners[i][1])
                    corners_list.append(p)
                board_msg.corners = corners_list
                self.geometry_pub.publish(board_msg)

            # Publish debug image
            debug_img = self._detector.draw_debug(frame, geometry)
            debug_msg = Image()
            debug_msg.header      = header
            debug_msg.height      = debug_img.shape[0]
            debug_msg.width       = debug_img.shape[1]
            debug_msg.encoding    = 'bgr8'
            debug_msg.is_bigendian = 0
            debug_msg.step        = debug_img.shape[1] * 3
            debug_msg.data        = debug_img.tobytes()
            self.debug_pub.publish(debug_msg)

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        self._shutdown.set()
        self._frame_event.set()  # Unblock the detection thread
        self._detect_thread.join(timeout=3.0)
        super().destroy_node()


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
