#!/usr/bin/env python3
"""
Camera Node — captures images from the Raspberry Pi Camera Module v2.

Backend priority (tried in order):
  1. picamera2  — Pi CSI camera via libcamera Python API (preferred)
  2. GStreamer   — libcamerasrc pipeline (works on Ubuntu 22.04 if OpenCV
                   was built with GStreamer support, even without python3-libcamera)
  3. OpenCV V4L2 — USB cameras or Pi Camera with bcm2835-v4l2 kernel module

If picamera2 fails with 'No module named libcamera', install it:
  sudo apt install python3-libcamera python3-picamera2
  # Or, if the package is not in the Ubuntu repos, run:
  sudo bash code/install_libcamera_python.sh

Published Topics:
  /camera/image_raw  (sensor_msgs/Image) — live camera feed at ~5fps

Services:
  /camera/capture    (std_srvs/Trigger)  — capture and publish a fresh frame

Parameters:
  use_picamera2    (bool)  — True = try Pi Camera backends, False = V4L2 only
  camera_id        (int)   — V4L2 device index (default 0)
  width            (int)   — capture width (default 1280)
  height           (int)   — capture height (default 720)
  fps              (float) — streaming frame rate (default 5.0)
  calibration_file (str)   — path to calibration.yaml; empty = skip
"""

import os
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

try:
    from picamera2 import Picamera2
    PICAMERA2_AVAILABLE = True
except ImportError:
    PICAMERA2_AVAILABLE = False


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('use_picamera2', True)
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 5.0)
        self.declare_parameter('calibration_file', '')

        self._use_picam = self.get_parameter('use_picamera2').value
        self._camera_id = self.get_parameter('camera_id').value
        self._width     = self.get_parameter('width').value
        self._height    = self.get_parameter('height').value
        self._fps       = float(self.get_parameter('fps').value)
        self._cal_file  = self.get_parameter('calibration_file').value

        self._camera_matrix: Optional[np.ndarray] = None
        self._dist_coeffs:   Optional[np.ndarray] = None
        self._load_calibration(self._cal_file)

        self._picam:   Optional['Picamera2'] = None
        self._cap:     Optional[cv2.VideoCapture] = None
        self._backend: str = 'none'
        self._init_camera()

        self._image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.create_service(Trigger, '/camera/capture', self._capture_cb)

        period = 1.0 / max(self._fps, 0.5)
        self._timer = self.create_timer(period, self._stream_tick)

        self.get_logger().info(
            f'Camera node ready (backend={self._backend}, '
            f'{self._width}x{self._height} @ {self._fps}fps, '
            f'calibration={"loaded" if self._camera_matrix is not None else "none"})'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Initialization
    # ─────────────────────────────────────────────────────────────────────

    def _init_camera(self):
        self._picam = None
        self._cap   = None
        self._backend = 'none'

        if self._use_picam:
            if self._try_picamera2():
                return
            if self._try_gstreamer():
                return

        if self._try_v4l2():
            return

        self.get_logger().error(
            'No working camera backend found.\n'
            '  Pi Camera v2 on Ubuntu 22.04:\n'
            '    sudo apt install python3-libcamera python3-picamera2\n'
            '    Or: sudo bash code/install_libcamera_python.sh\n'
            '  Then restart this node.\n'
            '  Diagnose with: python3 code/probe_camera.py'
        )

    def _try_picamera2(self) -> bool:
        """Attempt to open the Pi Camera via picamera2 (libcamera Python API)."""
        if not PICAMERA2_AVAILABLE:
            self.get_logger().warn(
                'picamera2 not installed — skipping (pip3 install picamera2)')
            return False
        try:
            picam = Picamera2()
            config = picam.create_video_configuration(
                main={'size': (self._width, self._height)})
            picam.configure(config)
            picam.start()
            time.sleep(1.0)
            self._picam   = picam
            self._backend = 'picamera2'
            self.get_logger().info(
                f'picamera2 backend ready: {self._width}x{self._height}')
            return True
        except Exception as e:
            err = str(e)
            if 'libcamera' in err.lower() or 'No module named' in err:
                self.get_logger().error(
                    f'picamera2 requires libcamera Python bindings.\n'
                    f'  Fix: sudo apt install python3-libcamera python3-picamera2\n'
                    f'  Or:  sudo bash code/install_libcamera_python.sh\n'
                    f'  Trying GStreamer fallback...'
                )
            else:
                self.get_logger().error(
                    f'picamera2 init failed: {e}\n  Trying GStreamer fallback...')
            return False

    def _try_gstreamer(self) -> bool:
        """Try GStreamer libcamerasrc pipeline (Ubuntu 22.04 fallback)."""
        pipeline = (
            f'libcamerasrc ! '
            f'video/x-raw,width={self._width},height={self._height},'
            f'framerate={int(self._fps)}/1 ! '
            f'videoconvert ! '
            f'video/x-raw,format=BGR ! '
            f'appsink drop=true max-buffers=1 sync=false'
        )
        try:
            cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not cap.isOpened():
                self.get_logger().debug('GStreamer libcamerasrc pipeline failed to open')
                return False
            time.sleep(1.5)
            ret, frame = cap.read()
            if ret and frame is not None and np.mean(frame) > 1.0:
                self._cap     = cap
                self._backend = 'gstreamer'
                self.get_logger().info(
                    f'GStreamer libcamerasrc backend ready: '
                    f'{frame.shape[1]}x{frame.shape[0]}')
                return True
            cap.release()
            self.get_logger().debug('GStreamer opened but returned empty frame')
            return False
        except Exception as e:
            self.get_logger().debug(f'GStreamer fallback failed: {e}')
            return False

    def _try_v4l2(self) -> bool:
        """Try OpenCV V4L2 backend, probing multiple device indices."""
        # Probe the requested device first, then 0-3
        devices = [self._camera_id] + [i for i in range(4) if i != self._camera_id]
        for dev_id in devices:
            dev_path = f'/dev/video{dev_id}'
            if not os.path.exists(dev_path):
                continue
            cap = cv2.VideoCapture(dev_id, cv2.CAP_V4L2)
            if not cap.isOpened():
                cap = cv2.VideoCapture(dev_id)
            if not cap.isOpened():
                continue
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
            cap.set(cv2.CAP_PROP_FPS, self._fps)
            time.sleep(1.0)
            # Test-read — must return a non-black frame
            for _ in range(5):
                ret, frame = cap.read()
                if ret and frame is not None and np.mean(frame) > 2.0:
                    self._cap     = cap
                    self._backend = f'v4l2:/dev/video{dev_id}'
                    self.get_logger().info(
                        f'OpenCV V4L2 backend ready: /dev/video{dev_id} '
                        f'{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                        f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}')
                    return True
            cap.release()
        return False

    # ─────────────────────────────────────────────────────────────────────
    # Calibration
    # ─────────────────────────────────────────────────────────────────────

    def _load_calibration(self, cal_file: str):
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
        """Read one frame from the active backend. Returns BGR ndarray or None."""
        if self._picam is not None:
            try:
                frame = self._picam.capture_array('main')
                # picamera2 returns RGB (or RGBA) — convert to BGR for OpenCV
                if frame.ndim == 3 and frame.shape[2] == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif frame.ndim == 3 and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                return frame
            except Exception as e:
                self.get_logger().error(f'picamera2 capture error: {e}')
                return None

        if self._cap is not None:
            ret, frame = self._cap.read()
            if ret and frame is not None:
                return frame
            self.get_logger().warn('Camera capture read failed')
            return None

        return None

    def _undistort(self, frame: np.ndarray) -> np.ndarray:
        if self._camera_matrix is None:
            return frame
        return cv2.undistort(frame, self._camera_matrix, self._dist_coeffs)

    def _publish_frame(self, frame: np.ndarray):
        corrected = self._undistort(frame)
        msg = Image()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height          = corrected.shape[0]
        msg.width           = corrected.shape[1]
        msg.encoding        = 'bgr8'
        msg.is_bigendian    = 0
        msg.step            = corrected.shape[1] * 3
        msg.data            = corrected.tobytes()
        self._image_pub.publish(msg)

    # ─────────────────────────────────────────────────────────────────────
    # Timer / Service
    # ─────────────────────────────────────────────────────────────────────

    def _stream_tick(self):
        frame = self._read_frame()
        if frame is not None:
            self._publish_frame(frame)

    def _capture_cb(self, request, response):
        if self._picam is not None:
            try:
                frame = self._picam.capture_array('main')
                if frame.ndim == 3 and frame.shape[2] == 3:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif frame.ndim == 3 and frame.shape[2] == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
                self._publish_frame(frame)
                response.success = True
                response.message = 'Captured via picamera2'
            except Exception as e:
                response.success = False
                response.message = f'picamera2 capture failed: {e}'
        elif self._cap is not None and self._cap.isOpened():
            for _ in range(3):
                self._cap.read()
            ret, frame = self._cap.read()
            if ret:
                self._publish_frame(frame)
                response.success = True
                response.message = f'Captured via {self._backend}'
            else:
                response.success = False
                response.message = 'Camera read failed'
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
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
