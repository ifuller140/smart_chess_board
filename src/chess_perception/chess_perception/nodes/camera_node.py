#!/usr/bin/env python3
"""
Camera Node — captures images from the Raspberry Pi Camera Module v2.

Uses picamera2 (the libcamera-based Python API for Pi cameras) as the
primary backend. Falls back to OpenCV VideoCapture (for USB cameras or
when picamera2 is unavailable, e.g. during development on a non-Pi host).

Published Topics:
  /camera/image_raw  (sensor_msgs/Image) — live camera feed at ~5fps

Services:
  /camera/capture    (std_srvs/Trigger)  — flush buffer and publish a fresh frame
                                           Returns success=True when a new frame
                                           is published to /camera/image_raw.

Parameters:
  use_picamera2  (bool)   — True = use picamera2 (Pi CSI camera), False = OpenCV
  camera_id      (int)    — OpenCV device index when use_picamera2=False
  width          (int)    — capture width (default 1280)
  height         (int)    — capture height (default 720)
  fps            (float)  — streaming frame rate (default 5.0)
  calibration_file (str)  — path to camera_calibration.yaml; empty = skip
"""

import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

# Try to import picamera2 — only available on Raspberry Pi with libcamera stack
try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # ── Parameters ─────────────────────────────────────────────────────
        self.declare_parameter('use_picamera2', True)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('calibration_file', '')

        self._use_picam   = self.get_parameter('use_picamera2').value
        self._camera_id   = self.get_parameter('camera_id').value
        self._width       = self.get_parameter('width').value
        self._height      = self.get_parameter('height').value
        self._fps         = float(self.get_parameter('fps').value)
        self._cal_file    = self.get_parameter('calibration_file').value

        # ── Calibration matrices ───────────────────────────────────────────
        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs:   Optional[np.ndarray] = None
        self._load_calibration(self._cal_file)

        # ── Bridge ─────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Camera backend ─────────────────────────────────────────────────
        self._picam: Optional['Picamera2'] = None
        self._cap:   Optional[cv2.VideoCapture] = None
        self._init_camera()

        # ── Publisher ──────────────────────────────────────────────────────
        self._image_pub = self.create_publisher(Image, '/camera/image_raw', 10)

        # ── Service ────────────────────────────────────────────────────────
        self.create_service(Trigger, '/camera/capture', self._capture_cb)

        # ── Streaming timer ────────────────────────────────────────────────
        period = 1.0 / max(self._fps, 0.5)
        self._timer = self.create_timer(period, self._stream_tick)

        self.get_logger().info(
            f'Camera node ready '
            f'(backend={"picamera2" if self._picam else "opencv"}, '
            f'{self._width}x{self._height} @ {self._fps}fps, '
            f'calibration={"loaded" if self._camera_matrix is not None else "none"})'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Initialization
    # ─────────────────────────────────────────────────────────────────────

    def _init_camera(self):
        """Initialize the camera backend (picamera2 preferred on Pi)."""
        if self._use_picam and PICAMERA2_AVAILABLE:
            try:
                self._picam = Picamera2()
                config = self._picam.create_still_configuration(
                    main={'size': (self._width, self._height),
                          'format': 'BGR888'},
                    buffer_count=2,
                )
                self._picam.configure(config)
                self._picam.start()
                time.sleep(1.0)  # Allow sensor to warm up / adjust exposure
                self.get_logger().info('picamera2 backend started')
                return
            except Exception as e:
                self.get_logger().warn(
                    f'picamera2 init failed ({e}), falling back to OpenCV')
                self._picam = None

        # GStreamer libcamerasrc — correct backend for Pi CSI camera on Ubuntu 24.04
        # when picamera2 is not installed.  libcamera-ipa must be installed.
        if self._use_picam or self._camera_id < 0:
            gst = (
                f'libcamerasrc ! '
                f'video/x-raw,width={self._width},height={self._height},'
                f'framerate={max(1, int(self._fps))}/1 ! '
                f'videoconvert ! video/x-raw,format=BGR ! appsink drop=true'
            )
            cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
            if cap.isOpened():
                # Test read to ensure pipeline is actually flowing
                ret, _ = cap.read()
                if ret:
                    time.sleep(1.0)  # allow AWB to settle
                    self._cap = cap
                    self.get_logger().info(
                        f'GStreamer libcamerasrc backend started '
                        f'{self._width}x{self._height} @ {self._fps}fps')
                    return
                cap.release()
            self.get_logger().warn('libcamerasrc GStreamer pipeline failed, trying V4L2...')

        # OpenCV fallback — scan for a working video device.
        # On Pi, /dev/video0 is often a metadata device; the real capture
        # device is found by trying each node and checking mean brightness.
        candidate_ids = (
            [self._camera_id]
            if self._camera_id >= 0
            else list(range(32))
        )

        for dev_id in candidate_ids:
            try:
                cap = cv2.VideoCapture(dev_id)
                if not cap.isOpened():
                    cap.release()
                    continue
                cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
                # Flush stale frames and check the device actually gives pixels
                for _ in range(3):
                    cap.read()
                ret, probe = cap.read()
                if ret and probe is not None:
                    self._cap = cap
                    self.get_logger().info(
                        f'OpenCV backend: /dev/video{dev_id} '
                        f'{self._width}x{self._height}')
                    return
                cap.release()
            except Exception:
                pass

        self.get_logger().error(
            'Could not find any working OpenCV camera device. '
            'Check that the camera is connected and not in use.')

    def _load_calibration(self, cal_file: str):
        """
        Load camera_matrix and dist_coeffs from a calibration YAML file.

        The file is expected to be in the format written by
        scripts/calibrate_camera.py (OpenCV FileStorage format):
          camera_matrix: ...
          dist_coeffs: ...
        """
        if not cal_file:
            self.get_logger().info('No calibration file specified — skipping undistortion')
            return

        try:
            fs = cv2.FileStorage(cal_file, cv2.FILE_STORAGE_READ)
            if not fs.isOpened():
                self.get_logger().warn(f'Could not open calibration file: {cal_file}')
                return
            self._camera_matrix = fs.getNode('camera_matrix').mat()
            self._dist_coeffs   = fs.getNode('dist_coeffs').mat()
            fs.release()
            self.get_logger().info(f'Camera calibration loaded from {cal_file}')
        except Exception as e:
            self.get_logger().warn(f'Calibration load error: {e}')

    # ─────────────────────────────────────────────────────────────────────
    # Frame Capture
    # ─────────────────────────────────────────────────────────────────────

    def _read_frame(self) -> Optional[np.ndarray]:
        """Read one frame from the active camera backend. Returns BGR ndarray or None."""
        if self._picam is not None:
            try:
                return self._picam.capture_array('main')   # Already BGR888
            except Exception as e:
                self.get_logger().error(f'picamera2 capture failed: {e}')
                return None

        if self._cap is not None and self._cap.isOpened():
            ret, frame = self._cap.read()
            if ret:
                return frame
            self.get_logger().warn('OpenCV capture read failed')
        return None

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        """Undistort frame using loaded calibration. Returns frame unchanged if no cal."""
        if self._camera_matrix is None:
            return frame
        return cv2.undistort(frame, self._camera_matrix, self._dist_coeffs)

    def _publish_frame(self, frame: np.ndarray):
        """Apply undistortion and publish to /camera/image_raw."""
        corrected = self._undistort(frame)
        msg = self._bridge.cv2_to_imgmsg(corrected, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        self._image_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    # Timer callback (streaming)
    # ─────────────────────────────────────────────────────────────────────

    def _stream_tick(self):
        frame = self._read_frame()
        if frame is not None:
            self._publish_frame(frame)

    # ─────────────────────────────────────────────────────────────────────
    # /camera/capture service
    # ─────────────────────────────────────────────────────────────────────

    def _capture_cb(self, request, response):
        """
        On-demand capture service.

        For picamera2: captures a fresh still (auto-exposure settled).
        For OpenCV: flushes the buffer then reads a fresh frame.
        """
        if self._picam is not None:
            # picamera2 — switch to still config for maximum quality capture
            try:
                frame = self._picam.capture_array('main')
                self._publish_frame(frame)
                response.success = True
                response.message = 'Captured via picamera2'
            except Exception as e:
                response.success = False
                response.message = f'picamera2 capture failed: {e}'
        elif self._cap is not None and self._cap.isOpened():
            # OpenCV — flush stale buffer frames then capture
            for _ in range(5):
                self._cap.read()
            ret, frame = self._cap.read()
            if ret:
                self._publish_frame(frame)
                response.success = True
                response.message = 'Captured via OpenCV'
            else:
                response.success = False
                response.message = 'OpenCV read failed'
        else:
            response.success = False
            response.message = 'No camera backend available'

        return response

    # ─────────────────────────────────────────────────────────────────────
    # Cleanup
    # ─────────────────────────────────────────────────────────────────────

    def destroy_node(self):
        if self._picam is not None:
            try:
                self._picam.stop()
                self._picam.close()
            except Exception:
                pass
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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
