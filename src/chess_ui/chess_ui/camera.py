"""MJPEG frame buffer for the raw camera feed, plus the --no-ros dev-mode
OpenCV camera fallback. Board/piece detection lives entirely in
chess_perception — this module only ever holds/streams the current raw
frame, never re-renders or annotates it."""

import threading
import time

import cv2
import numpy as np

from . import state


class CameraManager:
    """Thread-safe MJPEG frame buffer for the raw camera feed.

    The raw frame comes straight from ROS (`ros_client.RosNode._on_img`)
    or, in --no-ros dev mode, from `opencv_camera_loop` — never re-rendered
    or annotated here. Vision/board detection lives in `chess_perception`;
    see `/api/diff_frame` and `/api/square_scores` for its debug output.
    """
    def __init__(self):
        self._lock     = threading.Lock()
        self._raw_jpeg = b''
        self._index    = 0   # increments on every new raw frame

    def update_raw(self, jpeg_bytes: bytes):
        with self._lock:
            self._raw_jpeg = jpeg_bytes
            self._index   += 1

    def get_raw(self) -> bytes:
        with self._lock:
            return self._raw_jpeg

    def mjpeg_stream(self):
        """Generator that pushes MJPEG parts as fast as new frames arrive."""
        last_idx = -1
        while True:
            # Spin-wait for a new frame, up to 500 ms
            for _ in range(100):
                with self._lock:
                    idx  = self._index
                    data = self._raw_jpeg
                if idx != last_idx and data:
                    break
                time.sleep(0.005)
            else:
                data = blank_jpeg(640, 360, "Waiting for camera…")
            last_idx = idx
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')


camera = CameraManager()


def blank_jpeg(w: int, h: int, msg: str) -> bytes:
    img = np.full((h, w, 3), 18, dtype=np.uint8)
    cv2.putText(img, msg, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (80, 80, 80), 1)
    _, buf = cv2.imencode('.jpg', img)
    return bytes(buf)


def to_jpeg(frame, quality: int = 80) -> bytes:
    ok, buf = cv2.imencode('.jpg', frame,
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    return bytes(buf) if ok else b''


def opencv_camera_loop(camera_index: int):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        with state._lock:
            state._state["error"] = f"Cannot open camera {camera_index}"
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            30)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    with state._lock:
        state._state["cam_info"] = f"{w}×{h} opencv"
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        camera.update_raw(to_jpeg(frame, 75))
        with state._lock:
            state._state["raw_frame"] = frame
