#!/usr/bin/env python3
"""
ChessOS — Master Control Interface for Smart Chess Board

Unified dashboard: live camera (MJPEG), showcase display, full debug
controls for gantry, hardware, perception, game logic, and test runner.

Usage:
    python3 code/chess_os.py [--port 5000] [--no-ros] [--camera 0]
    python3 code/chess_os.py --mode showcase
    python3 code/chess_os.py --no-ros --image /path/to/frame.jpg

Access: http://192.168.1.149:5000
"""

from __future__ import annotations

import argparse
import json
import os
import queue
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np

try:
    from flask import Flask, Response, jsonify, render_template_string, request
except ImportError:
    print("ERROR: pip3 install flask"); sys.exit(1)

# ── Optional ROS2 ─────────────────────────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, Float32
    from sensor_msgs.msg import Image
    from geometry_msgs.msg import Point as RosPoint, Twist
    from std_srvs.srv import Trigger
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

# ── Paths ─────────────────────────────────────────────────────────────────────
_ROOT         = Path(__file__).parent.parent
_CORNERS_FILE = _ROOT / "board_corners.json"
_TEST_SCRIPT  = _ROOT / "run_hw_test.sh"

# ── Test catalogue ────────────────────────────────────────────────────────────
TEST_CATALOGUE: Dict[str, List[str]] = {
    "gantry": ["full","limits","pulse","motor_a","motor_b","corexy",
               "speed_sweep","repeatability","enable_hold","homing",
               "manual","lockstep","diagonal_sync","square_return",
               "raw_motor","timing_sweep","square_nav"],
    "servo":  ["full"],
    "camera": ["full"],
    "magnet": ["full"],
    "clock":  ["full", "integration", "display"],
    "vision": ["full", "corners", "board", "pieces", "squares", "fen"],
}

# ── Gantry workspace constants ────────────────────────────────────────────────
# Coordinate system: origin (0,0) at bottom-left (player's perspective)
# +X = rightward toward h-file / X limit switch (at X_MAX on right side)
# +Y = backward toward rank 8 / camera tower
_BOARD_ORIGIN_X = 20.0    # mm: X of a1 center from origin (left margin)
_BOARD_ORIGIN_Y = 20.0    # mm: Y of rank-1 center from origin (front margin)
_SQUARE_MM      = 25.0    # mm per chess square
_STEPS_PER_MM   = 5.0     # full-step mode, 20T pulley
_X_MAX_MM       = 240.0   # mm: X limit switch position (right side, X home)

def square_to_mm(sq: str):
    """Convert a square name (e.g. 'e4') to gantry XY in mm.
    +X = rightward (a=left, h=right), +Y = backward (rank1=front, rank8=back).
    """
    sq = sq.lower().strip()
    if len(sq) != 2:
        return None, None
    fi = ord(sq[0]) - ord('a')
    ri = int(sq[1]) - 1
    if not (0 <= fi < 8 and 0 <= ri < 8):
        return None, None
    return _BOARD_ORIGIN_X + fi * _SQUARE_MM, _BOARD_ORIGIN_Y + ri * _SQUARE_MM

# ── Shared state ──────────────────────────────────────────────────────────────
_lock = threading.Lock()

_state = {
    # Camera / vision
    "raw_frame":      None,
    "frame_count":    0,
    "last_updated":   0.0,
    "cam_info":       "–",
    "error":          "",
    "corners":        [[0.10,0.10],[0.90,0.10],[0.90,0.90],[0.10,0.90]],
    "pieces64":       [""] * 64,
    "detected_fen":   "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "last_move":      "",
    # Game manager
    "game_fen":       "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "game_state":     "OFFLINE",
    "game_turn":      "",
    "fen_source":     "local",
    "move_history":   [],
    # Chess clock
    "white_time":     600.0,
    "black_time":     600.0,
    # Gantry
    "gantry_x":       0.0,
    "gantry_y":       0.0,
    "gantry_homed":   False,
    "gantry_status":  "UNKNOWN",
    "stepper_status": "UNKNOWN",
    # Hardware
    "magnet_engaged": False,
    "limit_x":        False,
    "limit_y":        False,
    "limit_clock":    False,
    "estop_active":   False,
    # System
    "ros_connected":  False,
}

# ── Jog state — background thread publishes velocity to /stepper/velocity ───────
_jog: dict = {"dx": 0.0, "dy": 0.0, "speed": 50.0, "active": False}
_jog_lock = threading.Lock()

def _jog_loop():
    """20 Hz loop: publish velocity to /stepper/velocity while jog is active."""
    while True:
        time.sleep(0.05)
        with _jog_lock:
            if not _jog["active"]:
                continue
            dx, dy, spd = _jog["dx"], _jog["dy"], _jog["speed"]
        node = _ros_node
        if node is None:
            continue
        MAX_VEL = 600.0
        try:
            if HAS_ROS:
                msg = Twist()
                msg.linear.x = float(dx * MAX_VEL * spd / 100.0)
                msg.linear.y = float(dy * MAX_VEL * spd / 100.0)
                node.vel_pub.publish(msg)
        except Exception:
            pass

_params = {
    "white_thresh":       200,
    "black_thresh":        60,
    "min_blob_area":       80,
    "bottom_anchor_bias": 0.85,
    "undistort_enable":    0,
    "undistort_k1":       -25,
    "undistort_k2":         8,
    "undistort_focal":     83,
    "warp_size":          480,
    "show_grid":            1,
    "show_pieces":          1,
    "show_white_mask":      0,
    "show_black_mask":      0,
}

# ── Camera manager — MJPEG frame buffer ───────────────────────────────────────
class _CameraManager:
    def __init__(self):
        self._lock     = threading.Lock()
        self._raw_jpeg = b''
        self._warp_jpeg = b''
        self._index    = 0   # increments on every new raw frame

    def update_raw(self, jpeg_bytes: bytes):
        with self._lock:
            self._raw_jpeg = jpeg_bytes
            self._index   += 1

    def update_warp(self, jpeg_bytes: bytes):
        with self._lock:
            self._warp_jpeg = jpeg_bytes

    def get_raw(self) -> bytes:
        with self._lock:
            return self._raw_jpeg

    def get_warp(self) -> bytes:
        with self._lock:
            return self._warp_jpeg

    def mjpeg_stream(self, source: str = "raw"):
        """Generator that pushes MJPEG parts as fast as new frames arrive."""
        last_idx = -1
        while True:
            # Spin-wait for a new frame, up to 500 ms
            for _ in range(100):
                with self._lock:
                    idx  = self._index
                    data = self._raw_jpeg if source == "raw" else self._warp_jpeg
                if idx != last_idx and data:
                    break
                time.sleep(0.005)
            else:
                data = _blank_jpeg(640, 360, "Waiting for camera…")
            last_idx = idx
            yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n')

_camera = _CameraManager()

# ── Vision helpers (adapted from fen_visualizer.py) ───────────────────────────
def _load_corners():
    try:
        with open(_CORNERS_FILE) as f:
            data = json.load(f)
        if isinstance(data, list) and len(data) == 4:
            with _lock:
                _state["corners"] = data
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"⚠  corners load: {e}")

def _save_corners(corners):
    try:
        with open(_CORNERS_FILE, "w") as f:
            json.dump(corners, f, indent=2)
    except Exception as e:
        print(f"⚠  corners save: {e}")

def _best_fen() -> str:
    src = _state.get("fen_source", "local")
    if src == "game_mgr" and _state.get("game_fen"):
        return _state["game_fen"]
    return _state.get("detected_fen",
                      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")

def _blank_jpeg(w: int, h: int, msg: str) -> bytes:
    img = np.full((h, w, 3), 18, dtype=np.uint8)
    cv2.putText(img, msg, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (80, 80, 80), 1)
    _, buf = cv2.imencode('.jpg', img)
    return bytes(buf)

def _to_jpeg(frame, quality: int = 80) -> bytes:
    ok, buf = cv2.imencode('.jpg', frame,
                           [cv2.IMWRITE_JPEG_QUALITY, quality])
    return bytes(buf) if ok else b''

def _undistort(frame, p):
    if not p.get("undistort_enable", 0):
        return frame
    h, w = frame.shape[:2]
    f    = p["undistort_focal"] / 100.0 * w
    K    = np.array([[f, 0, w/2], [0, f, h/2], [0, 0, 1]], dtype=np.float64)
    D    = np.array([p["undistort_k1"]/100, p["undistort_k2"]/100, 0, 0],
                    dtype=np.float64)
    return cv2.undistort(frame, K, D)

def _warp(frame, corners_norm, sz: int):
    h, w   = frame.shape[:2]
    cpx    = [[c[0]*w, c[1]*h] for c in corners_norm]
    src    = np.float32(cpx)
    dst    = np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]])
    M      = cv2.getPerspectiveTransform(src, dst)
    M_inv  = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(frame, M, (sz, sz))
    return warped, M, M_inv, cpx

def _detect_pieces(warped, p):
    sq   = p["warp_size"] // 8
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    k    = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    wm   = cv2.morphologyEx(
               cv2.threshold(gray, p["white_thresh"], 255,
                             cv2.THRESH_BINARY)[1], cv2.MORPH_OPEN, k)
    bm   = cv2.morphologyEx(
               cv2.threshold(gray, p["black_thresh"], 255,
                             cv2.THRESH_BINARY_INV)[1], cv2.MORPH_OPEN, k)
    best: Dict = {}
    for color, mask in [('W', wm), ('B', bm)]:
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
        for cnt in cnts:
            area = cv2.contourArea(cnt)
            if area < p["min_blob_area"]:
                continue
            Mc = cv2.moments(cnt)
            if Mc["m00"] == 0:
                continue
            _, y0, _, h2 = cv2.boundingRect(cnt)
            cx   = int(Mc["m10"] / Mc["m00"])
            cyc  = int(Mc["m01"] / Mc["m00"])
            bias = p["bottom_anchor_bias"]
            cy   = min(int(cyc*(1-bias) + (y0+h2)*bias), sq*8-1)
            cx   = min(max(cx, 0), sq*8-1)
            key  = (cy // sq, cx // sq)
            if key not in best or area > best[key][0]:
                best[key] = (area, color)
    p64 = [""] * 64
    for (row, col), (_, color) in best.items():
        if 0 <= row < 8 and 0 <= col < 8:
            p64[row*8+col] = color
    return p64, wm, bm

def _fen_from_detection(p64) -> str:
    rows = []
    for ri in range(7, -1, -1):
        wr = 7 - ri
        empty, s = 0, ''
        for fi in range(8):
            pc = p64[wr*8+fi]
            if not pc:
                empty += 1
            else:
                if empty:
                    s += str(empty); empty = 0
                s += 'P' if pc == 'W' else 'p'
        if empty:
            s += str(empty)
        rows.append(s)
    return '/'.join(rows) + ' w KQkq - 0 1'

def _render_warp(warped, p, p64):
    out = warped.copy()
    sz  = p["warp_size"]
    sq  = sz // 8
    for r in range(8):
        for c in range(8):
            if (r + c) % 2 == 0:
                roi = out[r*sq:(r+1)*sq, c*sq:(c+1)*sq].astype(np.float32)
                roi = cv2.addWeighted(roi, 0.82,
                                      np.full_like(roi, 190), 0.18, 0)
                out[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = roi.astype(np.uint8)
    if p["show_grid"]:
        for i in range(9):
            cv2.line(out, (i*sq, 0),  (i*sq, sz),  (0, 200, 200), 1)
            cv2.line(out, (0, i*sq),  (sz, i*sq),  (0, 200, 200), 1)
    if p["show_pieces"]:
        for row in range(8):
            for col in range(8):
                pc = p64[row*8+col]
                if pc:
                    cx, cy = col*sq+sq//2, row*sq+sq//2
                    fill   = (240, 240, 240) if pc == 'W' else (25, 25, 25)
                    cv2.circle(out, (cx, cy), sq//3, fill, -1)
                    cv2.circle(out, (cx, cy), sq//3, (80, 80, 80), 1)
                    tc = (30, 30, 30) if pc == 'W' else (220, 220, 220)
                    cv2.putText(out, pc, (cx-8, cy+6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, tc, 2)
    cv2.rectangle(out, (0, 0), (sz-1, sz-1), (0, 255, 100), 2)
    return out

def _render_raw(frame, cpx, p, p64, M_inv):
    out = frame.copy()
    sz  = p["warp_size"]
    sq  = sz // 8
    if p["show_grid"] and M_inv is not None:
        for i in range(9):
            for segs in [([[i*sq, 0]], [[i*sq, sz]]),
                         ([[0, i*sq]], [[sz, i*sq]])]:
                p0 = cv2.perspectiveTransform(
                    np.float32([segs[0]]), M_inv)[0, 0].astype(int)
                p1 = cv2.perspectiveTransform(
                    np.float32([segs[1]]), M_inv)[0, 0].astype(int)
                cv2.line(out, tuple(p0), tuple(p1), (0, 200, 200), 1)
    labels = ['TL', 'TR', 'BR', 'BL']
    colors = [(255,80,80),(80,255,80),(80,80,255),(255,255,80)]
    for i, (cx, cy) in enumerate(cpx):
        cv2.circle(out, (int(cx), int(cy)), 10, colors[i], -1)
        cv2.putText(out, labels[i], (int(cx)+12, int(cy)+5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, colors[i], 2)
    cv2.polylines(out,
                  [np.array(cpx, dtype=np.int32).reshape(-1, 1, 2)],
                  True, (0, 255, 100), 2)
    return out

def _process_one(frame, corners, p):
    try:
        sz                        = p["warp_size"]
        warped, M, M_inv, cpx     = _warp(frame, corners, sz)
        p64, wm, bm               = _detect_pieces(warped, p)
        fen                       = _fen_from_detection(p64)
        raw_out                   = _render_raw(frame, cpx, p, p64, M_inv)
        warp_out                  = _render_warp(warped, p, p64)
        _camera.update_raw(_to_jpeg(raw_out, 78))
        _camera.update_warp(_to_jpeg(warp_out, 90))
        with _lock:
            _state["pieces64"]     = p64
            _state["detected_fen"] = fen
            _state["frame_count"] += 1
            _state["last_updated"] = time.time()
            _state["error"]        = ""
    except Exception as e:
        with _lock:
            _state["error"] = str(e)

def _vision_loop():
    """Background loop: re-processes the latest frame at ~8 fps."""
    while True:
        time.sleep(0.12)
        with _lock:
            frame   = _state["raw_frame"]
            corners = list(_state["corners"])
            p       = dict(_params)
        if frame is None:
            continue
        _process_one(_undistort(frame, p), corners, p)

def _force_reprocess():
    with _lock:
        frame   = _state["raw_frame"]
        corners = list(_state["corners"])
        p       = dict(_params)
    if frame is not None:
        _process_one(_undistort(frame, p), corners, p)

# ── OpenCV fallback camera ────────────────────────────────────────────────────
def _opencv_camera_loop(camera_index: int):
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        with _lock:
            _state["error"] = f"Cannot open camera {camera_index}"
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,  720)
    cap.set(cv2.CAP_PROP_FPS,            30)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    with _lock:
        _state["cam_info"] = f"{w}×{h} opencv"
    while True:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue
        _camera.update_raw(_to_jpeg(frame, 75))
        with _lock:
            _state["raw_frame"] = frame

# ── ROS2 interface ────────────────────────────────────────────────────────────
_ros_node = None

if HAS_ROS:
    class _RosNode(Node):
        def __init__(self):
            super().__init__("chess_os")

            # ── Subscriptions ──────────────────────────────────────────
            self.create_subscription(Image,   "/camera/image_raw",           self._on_img,            10)
            self.create_subscription(String,  "/game_manager/board_fen",     self._on_fen,            10)
            self.create_subscription(String,  "/game_manager/state",         self._on_state,          10)
            self.create_subscription(String,  "/game_manager/turn",          self._on_turn,           10)
            self.create_subscription(String,  "/perception/changed_squares", self._on_changed_squares,10)
            self.create_subscription(Bool,    "/limit_switch/x_min",         self._on_lim_x,          10)
            self.create_subscription(Bool,    "/limit_switch/y_min",         self._on_lim_y,          10)
            self.create_subscription(Bool,    "/limit_switch/clock_hit",     self._on_lim_clock,      10)
            self.create_subscription(RosPoint,"/gantry/pose",                self._on_gantry_pose,    10)
            self.create_subscription(String,  "/servo/state",                self._on_servo_state,    10)
            self.create_subscription(String,  "/gantry/status",              self._on_gantry_status,  10)
            self.create_subscription(String,  "/stepper/status",             self._on_stepper_status, 10)
            self.create_subscription(Float32, "/clock/white_time",           self._on_white_time,     10)
            self.create_subscription(Float32, "/clock/black_time",           self._on_black_time,     10)

            # ── Publishers ─────────────────────────────────────────────
            self.vel_pub       = self.create_publisher(Twist,    "/stepper/velocity",       10)
            self.cmd_pub       = self.create_publisher(RosPoint, "/stepper/command",        10)
            self.estop_pub     = self.create_publisher(Bool,     "/emergency_stop",         10)
            self.reset_pos_pub = self.create_publisher(Bool,     "/stepper/reset_position", 10)

            # ── Service clients ────────────────────────────────────────
            self._svc_engage       = self.create_client(Trigger, "/servo/engage")
            self._svc_release      = self.create_client(Trigger, "/servo/release")
            self._svc_home         = self.create_client(Trigger, "/gantry/home")
            self._svc_clock        = self.create_client(Trigger, "/clock/hit")
            self._svc_clock_reset  = self.create_client(Trigger, "/clock/reset")
            self._svc_clock_pause  = self.create_client(Trigger, "/clock/pause")
            self._svc_clock_resume = self.create_client(Trigger, "/clock/resume")

            with _lock:
                _state["ros_connected"] = True
            self.get_logger().info("ChessOS ROS2 node ready")

        def _on_img(self, msg):
            try:
                enc  = msg.encoding.lower()
                h, w = msg.height, msg.width
                data = np.frombuffer(msg.data, dtype=np.uint8)
                if enc in ('bgr8', 'bgr888'):
                    frame = data.reshape(h, msg.step)[:, :w*3].reshape(h, w, 3)
                elif enc in ('rgb8', 'rgb888'):
                    frame = data.reshape(h, msg.step)[:, :w*3].reshape(h, w, 3)
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                elif enc in ('yuv422', 'yuyv'):
                    yuyv  = data.reshape(h, msg.step)[:, :w*2].reshape(h, w, 2)
                    frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
                elif enc in ('mono8', '8uc1'):
                    mono  = data.reshape(h, msg.step)[:, :w].reshape(h, w)
                    frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
                else:
                    bpp   = max(3, len(msg.data) // (h * w) if h * w > 0 else 3)
                    frame = data.reshape(h, w, min(bpp, 4)).copy()
                    if bpp == 4:
                        frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
                _camera.update_raw(_to_jpeg(frame, 75))
                with _lock:
                    _state["raw_frame"] = frame
                    _state["cam_info"]  = f"{w}×{h} {enc}"
            except Exception as e:
                with _lock:
                    _state["error"] = f"img: {e}"

        def _on_fen(self, msg):
            fen = msg.data.strip()
            if fen:
                with _lock:
                    _state["game_fen"]     = fen
                    _state["fen_source"]   = "game_mgr"
                    _state["last_updated"] = time.time()
                    _state["frame_count"] += 1

        def _on_state(self, msg):
            with _lock:
                _state["game_state"] = msg.data.strip()

        def _on_turn(self, msg):
            with _lock:
                _state["game_turn"] = msg.data.strip()

        def _on_changed_squares(self, msg):
            raw = msg.data.strip()
            with _lock:
                _state["last_move"]    = raw
                _state["last_updated"] = time.time()
                _state["frame_count"] += 1

        def _on_lim_x(self, msg):
            with _lock: _state["limit_x"] = msg.data

        def _on_lim_y(self, msg):
            with _lock: _state["limit_y"] = msg.data

        def _on_lim_clock(self, msg):
            with _lock: _state["limit_clock"] = msg.data

        def _on_gantry_pose(self, msg):
            with _lock:
                _state["gantry_x"] = msg.x
                _state["gantry_y"] = msg.y

        def _on_servo_state(self, msg):
            with _lock:
                _state["magnet_engaged"] = (msg.data == "engaged")

        def _on_gantry_status(self, msg):
            with _lock:
                _state["gantry_status"] = msg.data
                if msg.data == "HOMED":
                    _state["gantry_homed"] = True

        def _on_stepper_status(self, msg):
            with _lock: _state["stepper_status"] = msg.data

        def _on_white_time(self, msg):
            with _lock: _state["white_time"] = msg.data

        def _on_black_time(self, msg):
            with _lock: _state["black_time"] = msg.data

        def call_svc(self, client, timeout: float = 2.0):
            if not client.wait_for_service(timeout_sec=0.5):
                return False, "Service not available"
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future,
                                             timeout_sec=timeout)
            if future.done():
                r = future.result()
                return r.success, r.message
            return False, "Timeout"

# ── Test runner ───────────────────────────────────────────────────────────────
class _TestRunner:
    def __init__(self):
        self._lock    = threading.Lock()
        self._proc    = None
        self._queue   = queue.Queue()
        self.running  = False
        self.last_run = None

    def start(self, category: str, subtest: str) -> bool:
        with self._lock:
            if self.running:
                return False
            cmd = (['bash', str(_TEST_SCRIPT),
                    '--category', category, '--subtest', subtest]
                   if _TEST_SCRIPT.exists()
                   else ['python3', '-m',
                         'chess_hw_interface.testing.test_runner',
                         '--category', category, '--subtest', subtest])
            try:
                self._proc = subprocess.Popen(
                    cmd, cwd=str(_ROOT),
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1,
                    preexec_fn=os.setsid)
                self.running  = True
                self.last_run = f"{category}/{subtest}"
            except Exception as e:
                self._queue.put({"type": "error", "line": str(e)})
                return False
        threading.Thread(target=self._read, daemon=True).start()
        return True

    def stop(self):
        with self._lock:
            if self._proc:
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGTERM)
                except Exception:
                    pass

    def _read(self):
        for line in self._proc.stdout:
            self._queue.put({"type": "line", "line": line.rstrip()})
        self._proc.wait()
        rc = self._proc.returncode
        self._queue.put({"type": "done", "line": f"Exited: {rc}", "rc": rc})
        with self._lock:
            self.running = False

    def stream_sse(self):
        while True:
            try:
                item = self._queue.get(timeout=30)
            except queue.Empty:
                yield 'data: {"type":"ping"}\n\n'
                continue
            yield f"data: {json.dumps(item)}\n\n"
            if item.get("type") == "done":
                return

_runner = _TestRunner()

# ── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.logger.setLevel("ERROR")

# ── Core routes ───────────────────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template_string(HTML)

@app.route("/api/stream/raw")
def stream_raw():
    return Response(_camera.mjpeg_stream("raw"),
                    mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})

@app.route("/api/stream/warp")
def stream_warp():
    return Response(_camera.mjpeg_stream("warp"),
                    mimetype="multipart/x-mixed-replace; boundary=frame",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})

@app.route("/api/status")
def api_status():
    with _lock:
        return jsonify({
            "frame_count":    _state["frame_count"],
            "last_updated":   _state["last_updated"],
            "cam_info":       _state["cam_info"],
            "error":          _state["error"],
            "fen":            _best_fen(),
            "fen_source":     _state["fen_source"],
            "game_state":     _state["game_state"],
            "game_turn":      _state["game_turn"],
            "last_move":      _state["last_move"],
            "move_history":   _state["move_history"],
            "white_time":     _state["white_time"],
            "black_time":     _state["black_time"],
            "limit_x":        _state["limit_x"],
            "limit_y":        _state["limit_y"],
            "limit_clock":    _state["limit_clock"],
            "gantry_x":       _state["gantry_x"],
            "gantry_y":       _state["gantry_y"],
            "gantry_homed":   _state["gantry_homed"],
            "gantry_status":  _state["gantry_status"],
            "stepper_status": _state["stepper_status"],
            "magnet_engaged": _state["magnet_engaged"],
            "estop_active":   _state["estop_active"],
            "ros_connected":  _state["ros_connected"],
        })

@app.route("/api/snapshot")
def api_snapshot():
    with _lock:
        frame = _state["raw_frame"]
    if frame is None:
        return Response(b'', status=204)
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return Response(bytes(buf), mimetype="image/jpeg",
                    headers={"Content-Disposition":
                             "attachment; filename=snapshot.jpg"})

# ── FEN / game routes ─────────────────────────────────────────────────────────
@app.route("/api/fen")
def api_fen_get():
    with _lock:
        return jsonify({"fen": _best_fen(), "source": _state["fen_source"]})

@app.route("/api/fen", methods=["POST"])
def api_fen_set():
    data = request.get_json(silent=True) or {}
    fen  = data.get("fen", "").strip()
    if not fen:
        return jsonify({"ok": False}), 400
    with _lock:
        _state["game_fen"]     = fen
        _state["fen_source"]   = "local"
        _state["last_updated"] = time.time()
        _state["frame_count"] += 1
    return jsonify({"ok": True})

# ── Vision params / corners ───────────────────────────────────────────────────
@app.route("/api/params", methods=["GET"])
def api_params_get():
    return jsonify(dict(_params))

@app.route("/api/params", methods=["POST"])
def api_params_set():
    data = request.get_json(silent=True) or {}
    with _lock:
        for k, v in data.items():
            if k in _params:
                _params[k] = type(_params[k])(v)
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})

@app.route("/api/corners", methods=["GET"])
def api_corners_get():
    with _lock:
        return jsonify({"corners": _state["corners"]})

@app.route("/api/corners", methods=["POST"])
def api_corners_set():
    data = request.get_json(silent=True) or {}
    c = data.get("corners")
    if not c or len(c) != 4:
        return jsonify({"ok": False}), 400
    try:
        c = [[float(x[0]), float(x[1])] for x in c]
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 400
    with _lock:
        _state["corners"] = c
    _save_corners(c)
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})

# ── Hardware routes ───────────────────────────────────────────────────────────
def _call_svc(attr: str):
    if _ros_node is None:
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503
    ok, msg = _ros_node.call_svc(getattr(_ros_node, attr))
    return jsonify({"ok": ok, "msg": msg})

@app.route("/api/hw/servo/engage", methods=["POST"])
def api_servo_engage():
    resp = _call_svc("_svc_engage")
    if isinstance(resp, tuple):
        return resp
    with _lock:
        _state["magnet_engaged"] = True
    return resp

@app.route("/api/hw/servo/release", methods=["POST"])
def api_servo_release():
    resp = _call_svc("_svc_release")
    if isinstance(resp, tuple):
        return resp
    with _lock:
        _state["magnet_engaged"] = False
    return resp

@app.route("/api/hw/gantry/home", methods=["POST"])
def api_gantry_home():
    resp = _call_svc("_svc_home")
    if isinstance(resp, tuple):
        return resp
    with _lock:
        _state["gantry_x"]     = 0.0
        _state["gantry_y"]     = 0.0
        _state["gantry_homed"] = True
    return resp

@app.route("/api/hw/clock/hit", methods=["POST"])
def api_clock_hit():
    return _call_svc("_svc_clock")

@app.route("/api/clock/reset", methods=["POST"])
def api_clock_reset():
    return _call_svc("_svc_clock_reset")

@app.route("/api/clock/pause", methods=["POST"])
def api_clock_pause():
    return _call_svc("_svc_clock_pause")

@app.route("/api/clock/resume", methods=["POST"])
def api_clock_resume():
    return _call_svc("_svc_clock_resume")

@app.route("/api/hw/estop", methods=["POST"])
def api_estop():
    with _lock:
        _state["estop_active"] = True
    with _jog_lock:
        _jog["active"] = False
    if _ros_node is not None:
        try:
            if HAS_ROS:
                _ros_node.estop_pub.publish(Bool(data=True))
                _ros_node.vel_pub.publish(Twist())
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500
    return jsonify({"ok": True, "msg": "EMERGENCY STOP sent"})

@app.route("/api/hw/estop/clear", methods=["POST"])
def api_estop_clear():
    with _lock:
        _state["estop_active"] = False
    return jsonify({"ok": True})

@app.route("/api/stepper/reset_position", methods=["POST"])
def api_stepper_reset_position():
    """Reset stepper position counter to (0,0). Call after manual homing or calibration."""
    if _ros_node is None:
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503
    try:
        if HAS_ROS:
            _ros_node.reset_pos_pub.publish(Bool(data=True))
        with _lock:
            _state["gantry_x"] = 0.0
            _state["gantry_y"] = 0.0
        return jsonify({"ok": True, "msg": "Position reset to (0, 0)"})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 500

@app.route("/api/capture_premove", methods=["POST"])
def api_capture_premove():
    """Call /perception/capture_premove service to snapshot the current board."""
    node = _ros_node
    if node is None:
        return jsonify({"ok": False, "message": "ROS not connected"}), 503
    try:
        from std_srvs.srv import Trigger
        client = node.create_client(Trigger, "/perception/capture_premove")
        if not client.wait_for_service(timeout_sec=2.0):
            return jsonify({"ok": False, "message": "Service not available"}), 503
        future = client.call_async(Trigger.Request())
        import rclpy as _rclpy
        _rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.done() and future.result():
            r = future.result()
            return jsonify({"ok": r.success, "message": r.message})
        return jsonify({"ok": False, "message": "Timeout"}), 504
    except Exception as e:
        return jsonify({"ok": False, "message": str(e)}), 500

# ── Gantry jog routes ─────────────────────────────────────────────────────────
@app.route("/api/gantry/jog/start", methods=["POST"])
def api_gantry_jog_start():
    """Start continuous jog. Background thread publishes velocity at 20 Hz."""
    data  = request.get_json(silent=True) or {}
    dx    = float(data.get("dx", 0))
    dy    = float(data.get("dy", 0))
    speed = float(data.get("speed", 50))
    with _jog_lock:
        _jog["dx"]     = max(-1.0, min(1.0, dx))
        _jog["dy"]     = max(-1.0, min(1.0, dy))
        _jog["speed"]  = max(5.0, min(100.0, speed))
        _jog["active"] = True
    return jsonify({"ok": True})

@app.route("/api/gantry/jog/stop", methods=["POST"])
def api_gantry_jog_stop():
    """Stop jogging and send a zero-velocity command immediately."""
    with _jog_lock:
        _jog["active"] = False
    if _ros_node is not None and HAS_ROS:
        try:
            _ros_node.vel_pub.publish(Twist())
        except Exception:
            pass
    return jsonify({"ok": True})

@app.route("/api/gantry/step", methods=["POST"])
def api_gantry_step():
    """Send a direct point-to-point step command to the stepper driver."""
    data     = request.get_json(silent=True) or {}
    steps_a  = int(data.get("steps_a", 0))
    steps_b  = int(data.get("steps_b", 0))
    speed    = float(data.get("speed", 50))
    if _ros_node is None:
        return jsonify({"ok": False, "msg": "ROS not connected"}), 503
    try:
        msg   = RosPoint()
        msg.x = float(steps_a)
        msg.y = float(steps_b)
        msg.z = float(speed)
        _ros_node.cmd_pub.publish(msg)
        return jsonify({"ok": True})
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 500

@app.route("/api/gantry/goto", methods=["POST"])
def api_gantry_goto():
    data = request.get_json(silent=True) or {}
    sq   = data.get("square", "")
    if sq:
        x, y = square_to_mm(sq)
        if x is None:
            return jsonify({"ok": False, "msg": "Invalid square"}), 400
    else:
        x = float(data.get("x", 0))
        y = float(data.get("y", 0))
    with _lock:
        _state["gantry_x"] = x
        _state["gantry_y"] = y
    return jsonify({"ok": True, "x": x, "y": y})

@app.route("/api/gantry/position")
def api_gantry_pos():
    with _lock:
        return jsonify({"x": _state["gantry_x"],
                        "y": _state["gantry_y"],
                        "homed": _state["gantry_homed"]})

# ── Test routes ───────────────────────────────────────────────────────────────
@app.route("/api/tests/catalogue")
def api_tests_catalogue():
    return jsonify(TEST_CATALOGUE)

@app.route("/api/tests/run", methods=["POST"])
def api_tests_run():
    data = request.get_json(silent=True) or {}
    cat  = data.get("category", "")
    sub  = data.get("subtest", "")
    if not cat or not sub:
        return jsonify({"ok": False, "msg": "Need category + subtest"}), 400
    ok = _runner.start(cat, sub)
    return jsonify({"ok": ok, "msg": "started" if ok else "already running"})

@app.route("/api/tests/stop", methods=["POST"])
def api_tests_stop():
    _runner.stop()
    return jsonify({"ok": True})

@app.route("/api/tests/stream")
def api_tests_stream():
    return Response(_runner.stream_sse(),
                    mimetype="text/event-stream",
                    headers={"Cache-Control": "no-cache",
                             "X-Accel-Buffering": "no"})

@app.route("/api/tests/status")
def api_tests_status():
    return jsonify({"running": _runner.running,
                    "last_run": _runner.last_run})

# ═════════════════════════════════════════════════════════════════════════════
# HTML TEMPLATE
# ═════════════════════════════════════════════════════════════════════════════
HTML = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ChessOS</title>
<link rel="stylesheet"
  href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
<script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
<script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
<style>
/* ── Reset & base ─────────────────────────────────────────────────── */
*{box-sizing:border-box;margin:0;padding:0}
html,body{height:100%;overflow:hidden}
body{background:#080812;color:#dde0f0;font-family:'Segoe UI',system-ui,sans-serif;
     font-size:13px;display:flex;flex-direction:column}
a{color:inherit;text-decoration:none}

/* ── CSS variables ────────────────────────────────────────────────── */
:root{
  --bg:#080812;--surf:#0e0e1c;--surf2:#151528;--border:#1e1e38;
  --text:#dde0f0;--dim:#5a5a80;--accent:#5ba3ff;--accent2:#00d4aa;
  --ok:#3fca6e;--warn:#f0a030;--err:#ff4060;
  --bar:44px;
}

/* ── Top bar ──────────────────────────────────────────────────────── */
#topbar{
  height:var(--bar);background:#0c0c1c;border-bottom:1px solid var(--border);
  display:flex;align-items:center;padding:0 14px;gap:10px;flex-shrink:0;
  z-index:100;
}
.logo{font-size:1.05em;font-weight:700;color:#fff;
      letter-spacing:.05em;display:flex;align-items:center;gap:7px}
.logo-icon{font-size:1.35em;line-height:1}
.mode-btns{display:flex;gap:2px;margin-left:10px}
.mode-btn{
  background:transparent;color:var(--dim);border:1px solid var(--border);
  padding:4px 12px;border-radius:4px;cursor:pointer;font-size:.7em;
  font-weight:600;letter-spacing:.05em;text-transform:uppercase;
  transition:all .15s;
}
.mode-btn.active{background:var(--accent);color:#fff;border-color:var(--accent)}
.spacer{flex:1}
.pills{display:flex;gap:5px;align-items:center}
.pill{
  display:flex;align-items:center;gap:4px;font-size:.67em;font-weight:600;
  padding:3px 8px;border-radius:10px;border:1px solid var(--border);
  background:var(--surf2);color:var(--dim);letter-spacing:.04em;
  text-transform:uppercase;
}
.pill-dot{width:7px;height:7px;border-radius:50%;background:var(--dim);flex-shrink:0}
.pill.live .pill-dot{background:var(--ok);animation:blink 2.5s infinite}
.pill.dead .pill-dot{background:var(--err)}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.25}}
.hbtn{
  background:var(--surf2);color:var(--dim);border:1px solid var(--border);
  padding:4px 10px;border-radius:4px;cursor:pointer;font-size:.7em;
  transition:color .12s;white-space:nowrap;
}
.hbtn:hover{color:var(--text)}
.estop-btn{
  background:#6b0a0a;color:#ff6060;border:2px solid #ff3030;
  padding:5px 14px;border-radius:5px;cursor:pointer;font-size:.72em;
  font-weight:800;letter-spacing:.08em;white-space:nowrap;
  transition:all .1s;text-transform:uppercase;
}
.estop-btn:hover{background:#900;color:#fff;border-color:#ff0000;box-shadow:0 0 12px #ff000066}
.estop-btn.fired{background:#ff0000;color:#fff;animation:estop-pulse .4s infinite alternate}
@keyframes estop-pulse{0%{box-shadow:0 0 6px #ff0000}100%{box-shadow:0 0 20px #ff0000}}

/* ── Layout: debug vs showcase ────────────────────────────────────── */
#showcase-view{display:none;flex:1;overflow:hidden}
#debug-view{display:flex;flex-direction:column;flex:1;overflow:hidden}
body.showcase #showcase-view{display:flex;flex-direction:column}
body.showcase #debug-view{display:none}

/* ── Showcase ─────────────────────────────────────────────────────── */
#showcase-view{
  align-items:center;justify-content:center;
  background:radial-gradient(ellipse at 40% 50%, #0d0d28 0%, #050510 100%);
}
.sc-wrap{
  display:grid;grid-template-columns:auto minmax(280px,340px);
  gap:40px;align-items:start;padding:20px;max-width:860px;width:100%;
}
.sc-left{display:flex;flex-direction:column;align-items:center;gap:16px}
#sc-board{width:clamp(260px,38vw,400px)}
.sc-cam{border-radius:8px;overflow:hidden;border:1px solid var(--border);
        width:100%;max-height:140px}
.sc-cam img{width:100%;display:block;object-fit:cover;max-height:140px}
.sc-right{display:flex;flex-direction:column;gap:14px;padding-top:12px}
.sc-title{font-size:2em;font-weight:800;color:#fff;letter-spacing:.06em;
          text-shadow:0 0 50px rgba(91,163,255,.35)}
.sc-subtitle{color:var(--dim);font-size:.82em;margin-top:3px}
.sc-card{background:var(--surf2);border:1px solid var(--border);
         border-radius:8px;padding:14px 16px}
.sc-row{display:flex;justify-content:space-between;align-items:center;
        padding:5px 0;border-bottom:1px solid var(--border);font-size:.85em}
.sc-row:last-child{border-bottom:none}
.sc-lbl{color:var(--dim)}
.sc-val{font-weight:600;font-family:monospace}
.sc-fen{font-family:monospace;font-size:.62em;color:var(--dim);
        word-break:break-all;margin-top:4px}

/* ── Debug: cameras ───────────────────────────────────────────────── */
.cam-row{display:grid;grid-template-columns:1fr 1fr;gap:6px;padding:6px;flex-shrink:0}
.cam-card{background:var(--surf);border:1px solid var(--border);border-radius:6px;overflow:hidden}
.cam-hdr{font-size:.6em;text-transform:uppercase;letter-spacing:.09em;color:var(--dim);
         padding:4px 9px;border-bottom:1px solid var(--border);font-weight:700;
         display:flex;justify-content:space-between;align-items:center}
.cam-hdr span{text-transform:none;letter-spacing:0;font-size:1em;color:var(--text)}
.cam-wrap{position:relative}
.cam-wrap img{width:100%;display:block;max-height:220px;object-fit:contain;background:#000}
.corner-handle{
  position:absolute;width:18px;height:18px;border-radius:50%;
  transform:translate(-50%,-50%);cursor:grab;z-index:10;
  border:2px solid #fff;display:flex;align-items:center;justify-content:center;
  font-size:8px;font-weight:700;color:#fff;user-select:none;
  box-shadow:0 0 8px #0008;
}
.corner-handle:active{cursor:grabbing}
.cam-footer{font-size:.6em;color:var(--dim);padding:3px 9px;
             white-space:nowrap;overflow:hidden;text-overflow:ellipsis}

/* ── Tabs ─────────────────────────────────────────────────────────── */
.tab-bar{display:flex;gap:1px;padding:0 6px;border-bottom:1px solid var(--border);
         flex-shrink:0;background:var(--surf)}
.tab-btn{
  background:transparent;color:var(--dim);border:none;
  border-bottom:2px solid transparent;padding:7px 14px;cursor:pointer;
  font-size:.7em;font-weight:600;letter-spacing:.05em;text-transform:uppercase;
  transition:color .12s;
}
.tab-btn.active{color:var(--accent);border-bottom-color:var(--accent)}
.tab-btn:hover:not(.active){color:var(--text)}
.tab-area{flex:1;overflow-y:auto;padding:8px}
.tab-panel{display:none}
.tab-panel.active{display:block}

/* ── Generic card ─────────────────────────────────────────────────── */
.card{background:var(--surf);border:1px solid var(--border);
      border-radius:6px;margin-bottom:8px;overflow:hidden}
.card-hdr{font-size:.6em;text-transform:uppercase;letter-spacing:.09em;
          color:var(--dim);padding:5px 10px;border-bottom:1px solid var(--border);
          font-weight:700;display:flex;justify-content:space-between;align-items:center}
.card-body{padding:10px 12px}
.two-col{display:grid;grid-template-columns:1fr 1fr;gap:8px}
@media(max-width:850px){.two-col{grid-template-columns:1fr}}

/* ── Stat rows ────────────────────────────────────────────────────── */
.stat{display:flex;justify-content:space-between;align-items:center;
      padding:4px 0;border-bottom:1px solid var(--border);font-size:.75em;gap:4px}
.stat:last-child{border-bottom:none}
.stat-lbl{color:var(--dim);white-space:nowrap;flex-shrink:0}
.stat-val{font-family:monospace;text-align:right;word-break:break-all}
.ok{color:var(--ok)}.warn{color:var(--warn)}.err{color:var(--err)}

/* ── Badges ───────────────────────────────────────────────────────── */
.badge{padding:2px 7px;border-radius:3px;font-size:.63em;font-weight:700;
       text-transform:uppercase;letter-spacing:.05em;white-space:nowrap}
.b-offline,.b-startup{background:#18182e;color:var(--dim)}
.b-homing{background:#2e1a00;color:var(--warn)}
.b-idle{background:#00183a;color:#79b8ff}
.b-waiting_player_move{background:#001a5a;color:#b8d4ff}
.b-capturing_board,.b-validating_move{background:#1a0040;color:#c89eff}
.b-calculating_response{background:#2e1a00;color:var(--warn)}
.b-executing_move,.b-hitting_clock{background:#2e0e00;color:#ff9560}
.b-game_over{background:#2e0010;color:#ff6080}
.b-src-game{background:#082a14;color:var(--ok)}
.b-src-perc{background:#00183a;color:#79b8ff}
.b-src-local{background:#18182e;color:var(--dim)}

/* ── Chessboard ───────────────────────────────────────────────────── */
#board-wrap{display:flex;justify-content:center;padding:10px}
#board{width:clamp(200px,28vw,280px)}
#fen-display{
  font-family:monospace;font-size:.6em;color:var(--ok);background:#040408;
  border-top:1px solid var(--border);padding:6px 10px;word-break:break-all;
}
.move-hist{font-family:monospace;font-size:.72em;color:var(--dim);background:#040408;
           border-radius:4px;padding:6px 8px;max-height:100px;overflow-y:auto;line-height:1.8}
.move-hist .mn{color:var(--dim)} .move-hist .mv{color:var(--text)}

/* ── Buttons ──────────────────────────────────────────────────────── */
.btn{
  display:inline-flex;align-items:center;gap:5px;
  background:var(--surf2);color:var(--text);border:1px solid var(--border);
  padding:5px 12px;border-radius:4px;cursor:pointer;font-size:.73em;
  white-space:nowrap;transition:background .1s;
}
.btn:hover{background:var(--border)}
.btn.primary{background:#1a4a9e;color:#fff;border-color:#1a4a9e}
.btn.primary:hover{background:#245ec0}
.btn.success{background:#082a14;color:var(--ok);border-color:#0d4020}
.btn.success:hover{background:#0d4020}
.btn.danger{background:#2a0810;color:#ff4060;border-color:#3a0e18}
.btn.danger:hover{background:#3a0e18}
.btn.warn{background:#2a1400;color:var(--warn);border-color:#3a1e00}
.btn.warn:hover{background:#3a1e00}
.btn-row{display:flex;gap:6px;flex-wrap:wrap;padding:4px 0}

/* ── Chess clock ──────────────────────────────────────────────────── */
.clock-wrap{display:flex;gap:8px;padding:10px 12px 4px;align-items:stretch}
.clock-side{
  flex:1;border-radius:6px;padding:10px 8px;text-align:center;
  border:1px solid var(--border);background:var(--surf2);
}
.clock-side.active-clock{border-color:var(--accent);box-shadow:0 0 10px #5ba3ff33}
.clock-side.low-time{border-color:var(--err);box-shadow:0 0 10px #ff406033;animation:lblink 1s infinite alternate}
@keyframes lblink{0%{border-color:var(--err)}100%{border-color:#ff8090}}
.clock-label{font-size:.6em;text-transform:uppercase;letter-spacing:.1em;
             color:var(--dim);font-weight:700;margin-bottom:6px}
.clock-time{font-family:monospace;font-size:1.7em;font-weight:700;letter-spacing:.06em;color:#eee}
.clock-w .clock-time{color:#f5f0d8}
.clock-b .clock-time{color:#a8a8cc}
.clock-controls{display:flex;flex-direction:column;justify-content:center;gap:6px;flex-shrink:0}

/* ── FEN inject ───────────────────────────────────────────────────── */
.fen-row{display:flex;gap:6px;padding:8px 10px}
.fen-row input{
  flex:1;background:#040408;color:var(--text);border:1px solid var(--border);
  border-radius:4px;padding:5px 8px;font-family:monospace;font-size:.68em;
}

/* ── Sliders ──────────────────────────────────────────────────────── */
.sl-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.sl-row label{flex:0 0 150px;font-size:.7em;color:var(--dim);
               overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.sl-row input[type=range]{flex:1;accent-color:var(--accent);height:4px}
.sl-row .sv{flex:0 0 38px;text-align:right;font-family:monospace;
            font-size:.7em;color:var(--text)}
.tog-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.tog-row label{font-size:.7em;color:var(--dim);cursor:pointer}
.tog-row input[type=checkbox]{accent-color:var(--accent);width:14px;height:14px;cursor:pointer}

/* ── Param groups ─────────────────────────────────────────────────── */
.pg-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
@media(max-width:700px){.pg-grid{grid-template-columns:1fr}}
.pg{background:var(--surf2);border:1px solid var(--border);border-radius:5px;padding:10px 12px}
.pg-title{font-size:.62em;text-transform:uppercase;letter-spacing:.08em;
          color:var(--accent);margin-bottom:8px;font-weight:700;
          border-bottom:1px solid var(--border);padding-bottom:5px}

/* ── Gantry ───────────────────────────────────────────────────────── */
.dpad{display:grid;grid-template-columns:repeat(3,54px);
      grid-template-rows:repeat(3,54px);gap:4px;margin:12px auto;width:fit-content}
.dpad-btn{
  background:var(--surf2);border:1px solid var(--border);border-radius:5px;
  cursor:pointer;font-size:1.5em;display:flex;align-items:center;justify-content:center;
  user-select:none;transition:background .08s;
}
.dpad-btn:hover{background:var(--border)}
.dpad-btn:active,.dpad-btn.pressed{background:var(--accent);color:#fff;border-color:var(--accent)}
.dpad-center{background:var(--surf);cursor:default;font-size:1em;color:var(--dim)}
.dpad-center:hover{background:var(--surf)}
.pos-readout{
  font-family:monospace;font-size:1.1em;text-align:center;
  color:var(--accent2);letter-spacing:.06em;padding:6px 0 10px;
}
.sq-row{display:flex;gap:8px;align-items:center}
.sq-row input{
  width:56px;background:#040408;color:var(--text);border:1px solid var(--border);
  border-radius:4px;padding:5px 8px;font-size:1.1em;font-family:monospace;
  text-align:center;text-transform:lowercase;
}
#gantry-canvas{border:1px solid var(--border);border-radius:4px;background:#030308;display:block;margin:auto;max-width:100%}
.canvas-wrap{display:flex;justify-content:center;padding:8px 0}

/* ── Hardware ─────────────────────────────────────────────────────── */
.hw-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(180px,1fr));gap:8px}
.hw-block{background:var(--surf2);border:1px solid var(--border);
          border-radius:5px;padding:12px;text-align:center}
.hw-title{font-size:.62em;text-transform:uppercase;letter-spacing:.08em;
          color:var(--dim);margin-bottom:10px;font-weight:700}
.indicator{display:inline-block;width:12px;height:12px;border-radius:50%;
           background:var(--border);border:2px solid var(--dim);
           transition:all .2s;vertical-align:middle;margin:0 4px}
.indicator.on{background:var(--ok);border-color:var(--ok);box-shadow:0 0 8px var(--ok)}
.hw-msg{font-size:.68em;color:var(--dim);margin-top:8px;min-height:1.2em}
.lim-row{display:flex;justify-content:space-around;padding:4px 0}
.lim-item{text-align:center}
.lim-item .lbl{font-size:.65em;color:var(--dim);margin-top:6px}

/* ── Tests ────────────────────────────────────────────────────────── */
.terminal{
  background:#020206;border:1px solid var(--border);border-radius:4px;
  font-family:'Consolas','Monaco',monospace;font-size:.7em;padding:8px;
  height:380px;overflow-y:auto;color:#c0ffc0;line-height:1.65;
}
.t-line{white-space:pre-wrap;word-break:break-all}
.t-ok{color:var(--ok)}.t-err{color:var(--err)}.t-warn{color:var(--warn)}
.t-dim{color:var(--dim)}.t-done{color:var(--accent);font-weight:700}
.test-sel{display:flex;gap:8px;align-items:center;flex-wrap:wrap;padding:6px 0}
.test-sel select{
  background:var(--surf2);color:var(--text);border:1px solid var(--border);
  border-radius:4px;padding:5px 8px;font-size:.73em;
}
.cat-list{font-size:.7em;line-height:2;color:var(--dim)}
.cat-list b{color:var(--accent)}
.cat-list a{color:var(--dim);transition:color .12s}
.cat-list a:hover{color:var(--text)}
</style>
</head>
<body class="debug">

<!-- ═══════════════ TOP BAR ════════════════════════════════════════ -->
<div id="topbar">
  <div class="logo">
    <span class="logo-icon">♟</span>ChessOS
  </div>
  <div class="mode-btns">
    <button class="mode-btn" id="btn-sc" onclick="setMode('showcase')">Showcase</button>
    <button class="mode-btn active" id="btn-db" onclick="setMode('debug')">Debug</button>
  </div>
  <div class="spacer"></div>
  <div class="pills">
    <div class="pill" id="pill-cam">
      <div class="pill-dot"></div>CAM
    </div>
    <div class="pill" id="pill-ros">
      <div class="pill-dot"></div>ROS
    </div>
    <div class="pill" id="pill-game">
      <div class="pill-dot"></div>GAME
    </div>
  </div>
  <button class="hbtn" onclick="triggerReprocess()">&#8635; Reprocess</button>
  <button class="hbtn" onclick="saveCorners()">&#128204; Corners</button>
  <a href="/api/snapshot" class="hbtn">&#128247; Snap</a>
  <button class="estop-btn" id="estop-btn" onclick="doEstop()">&#9760; E-STOP</button>
</div>

<!-- ═══════════════ SHOWCASE MODE ═════════════════════════════════ -->
<div id="showcase-view">
  <div class="sc-wrap">
    <div class="sc-left">
      <div id="sc-board"></div>
      <div class="sc-cam">
        <img src="/api/stream/raw" alt="live camera">
      </div>
    </div>
    <div class="sc-right">
      <div>
        <div class="sc-title">&#9822; ChessOS</div>
        <div class="sc-subtitle">Smart Chess Board &mdash; Master Control</div>
      </div>
      <div class="sc-card">
        <div class="sc-row">
          <span class="sc-lbl">Status</span>
          <span class="sc-val" id="sc-state">&#8212;</span>
        </div>
        <div class="sc-row">
          <span class="sc-lbl">Turn</span>
          <span class="sc-val" id="sc-turn">&#8212;</span>
        </div>
        <div class="sc-row">
          <span class="sc-lbl">Move</span>
          <span class="sc-val" id="sc-move">&#8212;</span>
        </div>
        <div class="sc-row">
          <span class="sc-lbl">FEN Source</span>
          <span class="sc-val" id="sc-source">&#8212;</span>
        </div>
        <div class="sc-row" style="border-bottom:none">
          <span class="sc-lbl">Camera</span>
          <span class="sc-val" id="sc-cam">&#8212;</span>
        </div>
      </div>
      <div class="sc-fen" id="sc-fen"></div>
    </div>
  </div>
</div>

<!-- ═══════════════ DEBUG MODE ════════════════════════════════════ -->
<div id="debug-view">

  <!-- Camera strip -->
  <div class="cam-row">
    <div class="cam-card">
      <div class="cam-hdr">
        Live Camera &mdash; drag corners to define board
        <span id="cam-ts">&#8212;</span>
      </div>
      <div class="cam-wrap" id="cam-wrap">
        <img id="cam-img" src="/api/stream/raw" alt="camera">
      </div>
      <div class="cam-footer" id="cam-footer">Waiting&hellip;</div>
    </div>
    <div class="cam-card">
      <div class="cam-hdr">
        Top-Down Board View
        <span style="color:var(--ok)">MJPEG &#9679;</span>
      </div>
      <div class="cam-wrap">
        <img src="/api/stream/warp" alt="warp" style="image-rendering:pixelated">
      </div>
      <div class="cam-footer" id="corner-footer">Corners: &#8212;</div>
    </div>
  </div>

  <!-- Tab bar -->
  <div class="tab-bar">
    <button class="tab-btn active" data-tab="game">Game</button>
    <button class="tab-btn" data-tab="perception">Perception</button>
    <button class="tab-btn" data-tab="gantry">Gantry</button>
    <button class="tab-btn" data-tab="hardware">Hardware</button>
    <button class="tab-btn" data-tab="tests">Tests</button>
  </div>

  <!-- Tab panels -->
  <div class="tab-area">

    <!-- ── GAME ─────────────────────────────────────────────────── -->
    <div class="tab-panel active" id="tab-game">
      <div class="two-col">
        <div>
          <div class="card">
            <div class="card-hdr">Board Position</div>
            <div id="board-wrap"><div id="board"></div></div>
            <div id="fen-display">&#8212;</div>
          </div>
          <div class="card">
            <div class="card-hdr">Inject / Override FEN</div>
            <div class="fen-row">
              <input id="fen-input" type="text"
                placeholder="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
              <button class="btn primary" onclick="injectFen()">Set</button>
            </div>
          </div>
          <div class="card">
            <div class="card-hdr">Move History</div>
            <div class="card-body" style="padding:6px 8px">
              <div class="move-hist" id="move-hist">&#8212;</div>
            </div>
          </div>
        </div>
        <div>
          <div class="card">
            <div class="card-hdr">Game State</div>
            <div class="card-body">
              <div class="stat"><span class="stat-lbl">State</span>
                <span class="stat-val">
                  <span id="gs-state" class="badge b-offline">Offline</span>
                </span>
              </div>
              <div class="stat"><span class="stat-lbl">Turn</span>
                <span class="stat-val" id="gs-turn">&#8212;</span></div>
              <div class="stat"><span class="stat-lbl">Full Move</span>
                <span class="stat-val" id="gs-move">&#8212;</span></div>
              <div class="stat"><span class="stat-lbl">FEN Source</span>
                <span class="stat-val">
                  <span id="gs-source" class="badge b-src-local">Local</span>
                </span>
              </div>
            </div>
          </div>
          <div class="card">
            <div class="card-hdr">Connection</div>
            <div class="card-body">
              <div class="stat"><span class="stat-lbl">Camera</span>
                <span class="stat-val" id="st-cam">&#8212;</span></div>
              <div class="stat"><span class="stat-lbl">ROS</span>
                <span class="stat-val" id="st-ros">&#8212;</span></div>
              <div class="stat"><span class="stat-lbl">Frames</span>
                <span class="stat-val" id="st-frames">0</span></div>
              <div class="stat"><span class="stat-lbl">FEN age</span>
                <span class="stat-val">
                  <span id="st-age">&#8212;</span>s
                </span>
              </div>
              <div class="stat"><span class="stat-lbl">Error</span>
                <span class="stat-val err" id="st-err"
                  style="font-size:.65em">&#8212;</span>
              </div>
            </div>
          </div>

          <div class="card">
            <div class="card-hdr">Chess Clock
              <span style="font-size:.85em;font-weight:400;text-transform:none;letter-spacing:0;color:var(--dim)" id="clock-state-lbl"></span>
            </div>
            <div class="clock-wrap">
              <div class="clock-side clock-w" id="clock-white">
                <div class="clock-label">&#9633; White</div>
                <div class="clock-time" id="white-time-disp">10:00</div>
              </div>
              <div class="clock-controls">
                <button class="btn" style="font-size:.63em;padding:3px 8px" onclick="clockAct('reset')">Reset</button>
                <button class="btn" style="font-size:.63em;padding:3px 8px" onclick="clockAct('pause')">Pause</button>
                <button class="btn" style="font-size:.63em;padding:3px 8px" onclick="clockAct('resume')">Resume</button>
              </div>
              <div class="clock-side clock-b" id="clock-black">
                <div class="clock-label">&#9632; Black</div>
                <div class="clock-time" id="black-time-disp">10:00</div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div><!-- /tab-game -->

    <!-- ── PERCEPTION ───────────────────────────────────────────── -->
    <div class="tab-panel" id="tab-perception">
      <div class="two-col" style="margin-bottom:8px">
        <div class="card">
          <div class="card-hdr">Corner Calibration</div>
          <div class="card-body" id="corner-editor"></div>
          <div class="card-body" style="padding-top:0">
            <div class="btn-row">
              <button class="btn primary" onclick="saveCorners()">&#128190; Save</button>
              <button class="btn" onclick="resetCorners()">&#8635; Reset</button>
              <button class="btn" onclick="triggerReprocess()">&#9881; Reprocess</button>
            </div>
          </div>
        </div>
        <div class="card">
          <div class="card-hdr">Frame-Diff Detection</div>
          <div class="card-body">
            <div class="stat-row">
              <span class="stat-lbl" style="color:var(--dim);font-size:.72em">Method</span>
              <span class="stat-val" style="font-size:.72em;color:#58a6ff">Pre-move frame diff</span>
            </div>
            <div class="stat-row">
              <span class="stat-lbl" style="color:var(--dim);font-size:.72em">Last Changed Squares</span>
              <span class="stat-val" id="perc-lastmove" style="font-family:monospace;color:#3fb950;font-size:.75em">–</span>
            </div>
            <div class="btn-row" style="margin-top:10px">
              <button class="btn primary" onclick="capturePreMove()">&#128247; Capture Pre-Move Ref</button>
            </div>
            <div style="font-size:.63em;color:var(--dim);margin-top:6px;line-height:1.5">
              The pre-move reference is auto-captured by the game manager at the
              start of each player's turn. Use this button to manually recapture
              if the board was disturbed.
            </div>
          </div>
        </div>
      </div>
      <div class="card">
        <div class="card-hdr">Overlay Toggles</div>
        <div class="card-body" id="overlay-togs"></div>
      </div>
      <div class="card">
        <div class="card-hdr">Warp &amp; Lens Parameters</div>
        <div class="card-body">
          <div class="pg-grid" id="pg-grid"></div>
        </div>
      </div>
    </div><!-- /tab-perception -->

    <!-- ── GANTRY ────────────────────────────────────────────────── -->
    <div class="tab-panel" id="tab-gantry">
      <div class="two-col">
        <div>
          <div class="card">
            <div class="card-hdr">
              Joystick Control
              <span style="color:var(--dim);font-size:.9em;text-transform:none;
                font-weight:400;letter-spacing:0">
                Arrow keys active on Gantry tab
              </span>
            </div>
            <div class="card-body">
              <div class="pos-readout" id="gantry-pos">X: 0.0 mm &nbsp;|&nbsp; Y: 0.0 mm</div>
              <div class="dpad">
                <button class="dpad-btn" id="dpad-nw"
                  onmousedown="startJog(-1,1)"  onmouseup="stopJog()"
                  ontouchstart="startJog(-1,1)" ontouchend="stopJog()">&#8598;</button>
                <button class="dpad-btn" id="dpad-n"
                  onmousedown="startJog(0,1)"   onmouseup="stopJog()"
                  ontouchstart="startJog(0,1)"  ontouchend="stopJog()">&#8593;</button>
                <button class="dpad-btn" id="dpad-ne"
                  onmousedown="startJog(1,1)"   onmouseup="stopJog()"
                  ontouchstart="startJog(1,1)"  ontouchend="stopJog()">&#8599;</button>
                <button class="dpad-btn" id="dpad-w"
                  onmousedown="startJog(-1,0)"  onmouseup="stopJog()"
                  ontouchstart="startJog(-1,0)" ontouchend="stopJog()">&#8592;</button>
                <div class="dpad-btn dpad-center" onclick="stopJog()">&#9632;</div>
                <button class="dpad-btn" id="dpad-e"
                  onmousedown="startJog(1,0)"   onmouseup="stopJog()"
                  ontouchstart="startJog(1,0)"  ontouchend="stopJog()">&#8594;</button>
                <button class="dpad-btn" id="dpad-sw"
                  onmousedown="startJog(-1,-1)" onmouseup="stopJog()"
                  ontouchstart="startJog(-1,-1)" ontouchend="stopJog()">&#8601;</button>
                <button class="dpad-btn" id="dpad-s"
                  onmousedown="startJog(0,-1)"  onmouseup="stopJog()"
                  ontouchstart="startJog(0,-1)" ontouchend="stopJog()">&#8595;</button>
                <button class="dpad-btn" id="dpad-se"
                  onmousedown="startJog(1,-1)"  onmouseup="stopJog()"
                  ontouchstart="startJog(1,-1)" ontouchend="stopJog()">&#8600;</button>
              </div>
              <div class="sl-row">
                <label>Speed (%)</label>
                <input type="range" id="jog-speed" min="5" max="100" value="30"
                  oninput="document.getElementById('jog-spv').textContent=this.value">
                <span class="sv" id="jog-spv">30</span>
              </div>
              <div class="btn-row" style="margin-top:8px">
                <button class="btn warn" onclick="doHome()">&#8962; Home Gantry</button>
                <button class="btn" onclick="resetPos()" title="Reset step counter to (0,0)">
                  &#8635; Reset Pos</button>
                <span id="home-badge"
                  style="font-size:.72em;color:var(--dim);align-self:center">Not homed</span>
              </div>
            </div>
          </div>
        </div>

        <div>
          <div class="card">
            <div class="card-hdr">Chess Square Navigation</div>
            <div class="card-body">
              <div class="sq-row">
                <input id="sq-input" type="text" maxlength="2" placeholder="e4">
                <button class="btn primary" onclick="gotoSquare()">Go to Square</button>
              </div>
              <div style="font-size:.68em;color:var(--dim);margin-top:8px;line-height:1.8">
                a1&ndash;h8 &nbsp;&bull;&nbsp; a1&approx;(20, 20)mm &nbsp;&bull;&nbsp;
                +X=right(h-file) &nbsp;&bull;&nbsp; +Y=back(rank8)
              </div>
            </div>
          </div>
          <div class="card">
            <div class="card-hdr">Workspace Visualizer</div>
            <div class="canvas-wrap">
              <canvas id="gantry-canvas" width="320" height="320"></canvas>
            </div>
          </div>
          <div class="card">
            <div class="card-hdr">Limit Switches — Live Status</div>
            <div class="card-body">
              <div class="stat"><span class="stat-lbl">X Home (Right&nbsp;/&nbsp;X&#8209;Max)</span>
                <span class="stat-val">
                  <span class="indicator" id="lim-x"></span>
                  <span id="lim-x-t">&#8212;</span>
                </span>
              </div>
              <div class="stat"><span class="stat-lbl">Y Home (Front&nbsp;/&nbsp;Y&#8209;Min)</span>
                <span class="stat-val">
                  <span class="indicator" id="lim-y"></span>
                  <span id="lim-y-t">&#8212;</span>
                </span>
              </div>
              <div class="stat"><span class="stat-lbl">Clock Hit</span>
                <span class="stat-val">
                  <span class="indicator" id="lim-c"></span>
                  <span id="lim-c-t">&#8212;</span>
                </span>
              </div>
              <div style="font-size:.6em;color:var(--dim);margin-top:8px;line-height:1.6">
                X Home triggers when gantry reaches rightmost position (h-file side).<br>
                Y Home triggers when gantry reaches frontmost position (player side).
              </div>
            </div>
          </div>
        </div>

      </div>
    </div><!-- /tab-gantry -->

    <!-- ── HARDWARE ──────────────────────────────────────────────── -->
    <div class="tab-panel" id="tab-hardware">
      <div class="hw-grid">

        <div class="hw-block">
          <div class="hw-title">Magnet Servo (Z-axis)</div>
          <div class="btn-row" style="justify-content:center">
            <button class="btn success" onclick="hwPost('/api/hw/servo/engage','engage')">
              &#11015; Engage Down</button>
          </div>
          <div class="btn-row" style="justify-content:center;margin-top:4px">
            <button class="btn danger" onclick="hwPost('/api/hw/servo/release','release')">
              &#11014; Release Up</button>
          </div>
          <div class="hw-msg">
            Magnet: <span class="indicator" id="hw-magnet"></span>
            <span id="hw-magnet-t">&#8212;</span>
          </div>
        </div>

        <div class="hw-block">
          <div class="hw-title">Clock Servo</div>
          <div class="btn-row" style="justify-content:center">
            <button class="btn warn" onclick="hwPost('/api/hw/clock/hit','clock')">
              &#9201; Hit Clock</button>
          </div>
          <div class="hw-msg" id="hw-clock-msg">&#8212;</div>
        </div>

        <div class="hw-block">
          <div class="hw-title">Gantry</div>
          <div class="btn-row" style="justify-content:center">
            <button class="btn warn" onclick="doHome()">&#8962; Home</button>
          </div>
          <div class="hw-msg" id="hw-home-msg">&#8212;</div>
        </div>

        <div class="hw-block">
          <div class="hw-title">Stepper Status</div>
          <div style="font-family:monospace;font-size:.9em;margin:8px 0">
            <span id="hw-stepper-status" class="badge b-offline">UNKNOWN</span>
          </div>
          <div style="font-size:.65em;color:var(--dim);margin-top:6px">
            Gantry: <span id="hw-gantry-status" style="color:var(--text)">—</span>
          </div>
          <div style="font-size:.65em;color:var(--dim);margin-top:2px">
            Homed: <span id="hw-homed-status" class="ok">—</span>
          </div>
        </div>

        <div class="hw-block" style="grid-column:1/-1">
          <div class="hw-title">Limit Switches — Live</div>
          <div class="lim-row">
            <div class="lim-item">
              <div class="indicator" id="hw-lim-x"
                style="width:20px;height:20px;margin:0 auto"></div>
              <div class="lbl">X Home</div>
              <div style="font-size:.6em;color:var(--dim)">Right / X-Max</div>
            </div>
            <div class="lim-item">
              <div class="indicator" id="hw-lim-y"
                style="width:20px;height:20px;margin:0 auto"></div>
              <div class="lbl">Y Home</div>
              <div style="font-size:.6em;color:var(--dim)">Front / Y-Min</div>
            </div>
            <div class="lim-item">
              <div class="indicator" id="hw-lim-c"
                style="width:20px;height:20px;margin:0 auto"></div>
              <div class="lbl">Clock Hit</div>
              <div style="font-size:.6em;color:var(--dim)">Button sensor</div>
            </div>
          </div>
        </div>

        <div class="hw-block" style="grid-column:1/-1">
          <div class="hw-title">Direct Stepper Command (Debug)</div>
          <div style="display:flex;gap:8px;align-items:center;flex-wrap:wrap;justify-content:center;margin-top:8px">
            <div style="display:flex;flex-direction:column;align-items:center;gap:3px">
              <span style="font-size:.62em;color:var(--dim)">Steps A</span>
              <input id="step-a" type="number" value="100" step="50"
                style="width:72px;background:#040408;color:var(--text);border:1px solid var(--border);
                       border-radius:4px;padding:4px 6px;font-family:monospace;font-size:.8em;text-align:center">
            </div>
            <div style="display:flex;flex-direction:column;align-items:center;gap:3px">
              <span style="font-size:.62em;color:var(--dim)">Steps B</span>
              <input id="step-b" type="number" value="100" step="50"
                style="width:72px;background:#040408;color:var(--text);border:1px solid var(--border);
                       border-radius:4px;padding:4px 6px;font-family:monospace;font-size:.8em;text-align:center">
            </div>
            <div style="display:flex;flex-direction:column;align-items:center;gap:3px">
              <span style="font-size:.62em;color:var(--dim)">Speed %</span>
              <input id="step-spd" type="number" value="35" min="5" max="100"
                style="width:56px;background:#040408;color:var(--text);border:1px solid var(--border);
                       border-radius:4px;padding:4px 6px;font-family:monospace;font-size:.8em;text-align:center">
            </div>
            <button class="btn primary" onclick="sendStep()" style="margin-top:12px">&#9654; Send</button>
            <button class="btn" onclick="sendStep(0,0)" style="margin-top:12px">&#9632; Stop</button>
          </div>
          <div style="font-size:.62em;color:var(--dim);margin-top:8px;line-height:1.5;text-align:center">
            Raw motor A/B steps. CoreXY: +X = A+,B&minus; &nbsp;|&nbsp; +Y = A+,B+
          </div>
        </div>

      </div>
    </div><!-- /tab-hardware -->

    <!-- ── TESTS ─────────────────────────────────────────────────── -->
    <div class="tab-panel" id="tab-tests">
      <div class="two-col">

        <div>
          <div class="card">
            <div class="card-hdr">Select &amp; Run Test</div>
            <div class="card-body">
              <div class="test-sel">
                <select id="test-cat" onchange="updateSubtests()"></select>
                <select id="test-sub"></select>
              </div>
              <div class="btn-row">
                <button class="btn primary" onclick="runTest()">&#9654; Run</button>
                <button class="btn danger"  onclick="stopTest()">&#9632; Stop</button>
                <button class="btn"         onclick="clearOutput()">&#9003; Clear</button>
                <button class="btn"         onclick="scrollBottom()">&#8659; Bottom</button>
              </div>
              <div style="margin-top:8px;font-size:.72em">
                Status: <span id="test-status" class="badge b-offline">Idle</span>
                &nbsp; Last: <span id="test-last" style="color:var(--dim)">&#8212;</span>
              </div>
            </div>
          </div>
          <div class="card">
            <div class="card-hdr">Quick Launch</div>
            <div class="card-body">
              <div class="cat-list" id="cat-list">Loading&hellip;</div>
            </div>
          </div>
        </div>

        <div>
          <div class="card" style="height:100%">
            <div class="card-hdr">Output Terminal</div>
            <div class="terminal" id="test-output"></div>
          </div>
        </div>

      </div>
    </div><!-- /tab-tests -->

  </div><!-- /tab-area -->
</div><!-- /debug-view -->

<!-- ═══════════════ JAVASCRIPT ════════════════════════════════════ -->
<script>
// ── State ─────────────────────────────────────────────────────────
var uiMode     = 'debug';
var activeTab  = 'game';
var curStatus  = {};
var corners    = [{fx:.10,fy:.10},{fx:.90,fy:.10},{fx:.90,fy:.90},{fx:.10,fy:.90}];
var handles    = [];
var dragging   = null;
var curParams  = {};
var heldKeys   = {};
var jogTimer   = null;  // unused — kept for legacy safety
var jogDx      = 0;
var jogDy      = 0;
var testSSE    = null;
var catalogue  = {};
var board      = null;
var scBoard    = null;

// Workspace constants — origin at bottom-left, +X=rightward, +Y=backward
// a1 = (BO_X, BO_Y), h8 = (BO_X+7*SQ, BO_Y+7*SQ)
var BO_X = 20, BO_Y = 20, SQ = 25, X_MAX = 240;

// ── Mode ──────────────────────────────────────────────────────────
function setMode(m) {
  uiMode = m;
  document.body.className = m;
  document.getElementById('btn-sc').classList.toggle('active', m==='showcase');
  document.getElementById('btn-db').classList.toggle('active', m==='debug');
}

// ── Tabs ──────────────────────────────────────────────────────────
document.querySelectorAll('.tab-btn').forEach(function(btn) {
  btn.addEventListener('click', function() {
    var t = btn.dataset.tab;
    activeTab = t;
    document.querySelectorAll('.tab-btn').forEach(function(b){ b.classList.remove('active'); });
    document.querySelectorAll('.tab-panel').forEach(function(p){ p.classList.remove('active'); });
    btn.classList.add('active');
    document.getElementById('tab-'+t).classList.add('active');
    if (t === 'gantry') drawCanvas();
  });
});

// ── Chess boards (init after jQuery loads) ────────────────────────
$(function() {
  var cfg = {
    showNotation: true,
    pieceTheme: 'https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/img/chesspieces/wikipedia/{piece}.png'
  };
  board   = Chessboard('board',    $.extend({}, cfg, {position:'start'}));
  scBoard = Chessboard('sc-board', $.extend({}, cfg, {position:'start'}));
});

function updateBoards(fen) {
  try {
    var pos = fen.split(' ')[0];
    if (board)   { try { board.position(pos, false); }   catch(e){} }
    if (scBoard) { try { scBoard.position(pos, false); } catch(e){} }
  } catch(e) {}
}

// ── Corner handles ────────────────────────────────────────────────
var CH_COLORS = ['#f05050','#50e050','#5080ff','#f0d050'];
var CH_LABELS = ['TL','TR','BR','BL'];

function buildHandles() {
  var wrap = document.getElementById('cam-wrap');
  handles.forEach(function(h){ if(h.parentNode) h.parentNode.removeChild(h); });
  handles = [];
  corners.forEach(function(c, i) {
    var el = document.createElement('div');
    el.className = 'corner-handle';
    el.style.background = CH_COLORS[i];
    el.textContent = CH_LABELS[i];
    wrap.appendChild(el);
    handles.push(el);
    el.addEventListener('mousedown', function(e) {
      e.preventDefault();
      dragging = {i:i, sx:e.clientX, sy:e.clientY, sfx:c.fx, sfy:c.fy};
    });
    el.addEventListener('touchstart', function(e) {
      e.preventDefault();
      var t = e.touches[0];
      dragging = {i:i, sx:t.clientX, sy:t.clientY, sfx:c.fx, sfy:c.fy};
    }, {passive:false});
  });
  posHandles();
}

function posHandles() {
  var img = document.getElementById('cam-img');
  var W = img.clientWidth, H = img.clientHeight;
  corners.forEach(function(c, i) {
    handles[i].style.left = (c.fx*W)+'px';
    handles[i].style.top  = (c.fy*H)+'px';
  });
}

function applyDrag(cx, cy) {
  if (!dragging) return;
  var img = document.getElementById('cam-img');
  var W = img.clientWidth, H = img.clientHeight;
  corners[dragging.i].fx = Math.min(1,Math.max(0, dragging.sfx+(cx-dragging.sx)/W));
  corners[dragging.i].fy = Math.min(1,Math.max(0, dragging.sfy+(cy-dragging.sy)/H));
  posHandles(); updateCornerInfo();
}

document.addEventListener('mousemove', function(e){ applyDrag(e.clientX, e.clientY); });
document.addEventListener('mouseup',   function(){ if(dragging){ dragging=null; sendCorners(); }});
document.addEventListener('touchmove', function(e){
  if(!dragging) return; e.preventDefault();
  applyDrag(e.touches[0].clientX, e.touches[0].clientY);
},{passive:false});
document.addEventListener('touchend', function(){ if(dragging){ dragging=null; sendCorners(); }});

function updateCornerInfo() {
  var t = corners.map(function(c,i){
    return CH_LABELS[i]+'('+c.fx.toFixed(3)+','+c.fy.toFixed(3)+')';
  }).join('  ');
  document.getElementById('corner-footer').textContent = t;
  corners.forEach(function(c,i) {
    var ex = document.getElementById('ced-x'+i);
    var ey = document.getElementById('ced-y'+i);
    if(ex) ex.value = c.fx.toFixed(3);
    if(ey) ey.value = c.fy.toFixed(3);
  });
}

function sendCorners() {
  fetch('/api/corners', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({corners:corners.map(function(c){return[c.fx,c.fy]})})});
}
function saveCorners()  { sendCorners(); }
function resetCorners() {
  corners = [{fx:.10,fy:.10},{fx:.90,fy:.10},{fx:.90,fy:.90},{fx:.10,fy:.90}];
  posHandles(); updateCornerInfo(); sendCorners();
}
function loadCornersFromServer() {
  fetch('/api/corners').then(function(r){return r.json();}).then(function(d){
    if (d.corners && d.corners.length===4) {
      corners = d.corners.map(function(c){return{fx:c[0],fy:c[1]};});
      posHandles(); updateCornerInfo();
    }
  });
}

document.getElementById('cam-img').addEventListener('load', function(){ buildHandles(); posHandles(); });
window.addEventListener('resize', posHandles);

// ── Corner editor (perception tab) ───────────────────────────────
function buildCornerEditor() {
  var el = document.getElementById('corner-editor');
  var html = '<div style="display:grid;grid-template-columns:auto 1fr 1fr;gap:5px 10px;align-items:center;font-size:.75em">';
  html += '<div style="color:var(--dim)">Corner</div><div style="color:var(--dim)">X</div><div style="color:var(--dim)">Y</div>';
  CH_LABELS.forEach(function(lbl,i) {
    html += '<div style="color:'+CH_COLORS[i]+';font-weight:700">'+lbl+'</div>'+
      '<input id="ced-x'+i+'" type="number" step="0.01" min="0" max="1" value="'+corners[i].fx.toFixed(3)+'"'+
      ' style="background:#040408;color:var(--text);border:1px solid var(--border);border-radius:4px;padding:3px 6px"'+
      ' onchange="setCornerXY('+i+',\'x\',+this.value)">'+
      '<input id="ced-y'+i+'" type="number" step="0.01" min="0" max="1" value="'+corners[i].fy.toFixed(3)+'"'+
      ' style="background:#040408;color:var(--text);border:1px solid var(--border);border-radius:4px;padding:3px 6px"'+
      ' onchange="setCornerXY('+i+',\'y\',+this.value)">';
  });
  el.innerHTML = html + '</div>';
}

function setCornerXY(i, ax, v) {
  if (ax==='x') corners[i].fx = Math.min(1,Math.max(0,v));
  else          corners[i].fy = Math.min(1,Math.max(0,v));
  posHandles(); updateCornerInfo();
}

// ── Overlay toggles ───────────────────────────────────────────────
var OVLAY = [
  {k:'show_grid',        l:'Show grid overlay'},
  {k:'show_pieces',      l:'Show piece circles'},
  {k:'undistort_enable', l:'Enable lens undistortion'},
];
function buildOverlayToggles(p) {
  var el = document.getElementById('overlay-togs');
  var html = '';
  OVLAY.forEach(function(t) {
    html += '<div class="tog-row">'+
      '<input type="checkbox" id="ov_'+t.k+'" '+(p[t.k]?'checked':'')+
      ' onchange="sendParam({'+JSON.stringify(t.k)+':this.checked?1:0})">'+
      '<label for="ov_'+t.k+'">'+t.l+'</label></div>';
  });
  el.innerHTML = html;
}

// ── Param sliders ─────────────────────────────────────────────────
var PARAM_GRP = [
  {t:'Lens Correction', ps:[
    {k:'undistort_k1',    l:'k1 (×100)',      mn:-80,mx:80, st:1},
    {k:'undistort_k2',    l:'k2 (×100)',      mn:-50,mx:50, st:1},
    {k:'undistort_focal', l:'Focal % width',  mn:40, mx:130,st:1},
  ]},
  {t:'Transform', ps:[
    {k:'warp_size',       l:'Warp output size', mn:240,mx:800,st:40},
  ]},
];

function buildParamSliders(p) {
  var grid = document.getElementById('pg-grid');
  var html = '';
  PARAM_GRP.forEach(function(g) {
    html += '<div class="pg"><div class="pg-title">'+g.t+'</div>';
    g.ps.forEach(function(pr) {
      var raw  = p[pr.k] !== undefined ? p[pr.k] : 0;
      var disp = pr.scale ? Math.round(raw*pr.scale) : raw;
      html += '<div class="sl-row">'+
        '<label title="'+pr.k+'">'+pr.l+'</label>'+
        '<input type="range" id="sl_'+pr.k+'" min="'+pr.mn+'" max="'+pr.mx+
        '" step="'+pr.st+'" value="'+disp+'"'+
        ' oninput="document.getElementById(\'sv_'+pr.k+'\').textContent=this.value"'+
        ' onchange="onSlider(\''+pr.k+'\',+this.value,'+(pr.scale||1)+')">'+
        '<span class="sv" id="sv_'+pr.k+'">'+disp+'</span></div>';
    });
    html += '</div>';
  });
  grid.innerHTML = html;
}

function onSlider(k, v, scale) { sendParam({[k]: scale>1 ? v/scale : v}); }
function sendParam(d) {
  fetch('/api/params',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify(d)});
}
function triggerReprocess() { sendParam({}); }
function capturePreMove() {
  fetch('/api/capture_premove', {method:'POST'}).then(function(r){return r.json();})
    .then(function(d){ alert(d.message || (d.ok ? 'Pre-move captured' : 'Failed')); })
    .catch(function(){ alert('Service unavailable'); });
}

// ── Gantry controls ───────────────────────────────────────────────
function getJogSpeed() { return +document.getElementById('jog-speed').value; }

function startJog(dx, dy) {
  jogDx = dx; jogDy = dy;
  fetch('/api/gantry/jog/start', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({dx: dx, dy: dy, speed: getJogSpeed()})
  });
}
function stopJog() {
  jogDx = 0; jogDy = 0;
  fetch('/api/gantry/jog/stop', {method:'POST'});
}

// Arrow key hold-to-jog
document.addEventListener('keydown', function(e) {
  if (activeTab !== 'gantry') return;
  var map = {ArrowUp:[0,1],ArrowDown:[0,-1],ArrowLeft:[-1,0],ArrowRight:[1,0]};
  if (!map[e.key]) return;
  e.preventDefault();
  if (heldKeys[e.key]) return;
  heldKeys[e.key] = true;
  var dx = (heldKeys.ArrowRight ? 1 : 0) - (heldKeys.ArrowLeft ? 1 : 0);
  var dy = (heldKeys.ArrowUp    ? 1 : 0) - (heldKeys.ArrowDown ? 1 : 0);
  startJog(dx, dy);
});
document.addEventListener('keyup', function(e) {
  if (!heldKeys[e.key]) return;
  delete heldKeys[e.key];
  if (Object.keys(heldKeys).length === 0) stopJog();
});
document.addEventListener('blur', function() { heldKeys = {}; stopJog(); });

function gotoSquare() {
  var sq = document.getElementById('sq-input').value.trim().toLowerCase();
  if (!sq) return;
  fetch('/api/gantry/goto',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({square:sq})
  }).then(function(r){return r.json();}).then(function(d){
    if (d.ok) setGantryPos(d.x, d.y);
    else alert('Invalid square: '+sq);
  });
}

function doHome() {
  var badge = document.getElementById('home-badge');
  badge.textContent = '⟳ Homing…';
  badge.style.color = 'var(--warn)';
  fetch('/api/hw/gantry/home',{method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      var msg = (d.ok ? '✓ ' : '✗ ') + (d.msg||'');
      document.getElementById('hw-home-msg').textContent = msg;
      if (d.ok) {
        setGantryPos(0, 0);
        badge.textContent = '✓ Homed';
        badge.style.color = 'var(--ok)';
      } else {
        badge.textContent = '✗ Failed';
        badge.style.color = 'var(--err)';
      }
    }).catch(function(){
      badge.textContent = '✗ Error';
      badge.style.color = 'var(--err)';
    });
}

function resetPos() {
  fetch('/api/stepper/reset_position', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if (d.ok) { setGantryPos(0, 0); }
      else { alert('Reset failed: '+(d.msg||'unknown')); }
    }).catch(function(){});
}

function setGantryPos(x, y) {
  document.getElementById('gantry-pos').textContent =
    'X: '+x.toFixed(1)+' mm  |  Y: '+y.toFixed(1)+' mm';
  drawCanvas(x, y);
}

// ── Gantry workspace canvas ───────────────────────────────────────
// Coordinate system: origin bottom-left, +X right, +Y up
// Canvas maps mm→px: px = pad + mm*sc, py = H-pad - mm*sc
function drawCanvas(gx, gy) {
  var canvas = document.getElementById('gantry-canvas');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var W = 320, H = 320, pad = 30;
  // Scale: workspace is X_MAX mm wide (full travel)
  var sc = (W - 2*pad) / X_MAX;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#020206';
  ctx.fillRect(0, 0, W, H);

  // Workspace border (full travel area)
  ctx.strokeStyle = '#1a1a2e';
  ctx.lineWidth = 1;
  ctx.strokeRect(pad, pad, W-2*pad, H-2*pad);

  // X limit indicator (right side = X home)
  ctx.fillStyle = '#1e0a0a';
  ctx.fillRect(W-pad-2, pad, 2, H-2*pad);
  ctx.fillStyle = '#ff4060';
  ctx.font = '7px monospace';
  ctx.fillText('X HOME', W-pad+3, pad+12);

  // Y limit indicator (bottom = Y home)
  ctx.fillStyle = '#0a1e0a';
  ctx.fillRect(pad, H-pad, W-2*pad, 2);
  ctx.fillStyle = '#3fca6e';
  ctx.font = '7px monospace';
  ctx.fillText('Y HOME', pad+2, H-pad+10);

  // Chess square grid
  var sqpx = SQ * sc;
  for (var r = 0; r < 8; r++) {
    for (var c = 0; c < 8; c++) {
      var px = pad + (BO_X + c*SQ)*sc;
      var py = H - pad - (BO_Y + (r+1)*SQ)*sc;
      ctx.fillStyle = (r+c)%2===0 ? '#1a1a3e' : '#0d0d1e';
      ctx.fillRect(px, py, sqpx, sqpx);
    }
  }

  // Board border
  ctx.strokeStyle = '#3a3a7a';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(
    pad + BO_X*sc,
    H - pad - (BO_Y + 8*SQ)*sc,
    8*sqpx, 8*sqpx
  );

  // Corner square labels
  ctx.fillStyle = '#40407a';
  ctx.font = '7px monospace';
  ctx.fillText('a1', pad+BO_X*sc+2, H-pad-BO_Y*sc-2);
  ctx.fillText('h8', pad+(BO_X+7*SQ)*sc+2, H-pad-(BO_Y+7*SQ)*sc-12);

  // Axis tick labels (0, X_MAX)
  ctx.fillStyle = '#30305a';
  ctx.font = '7px monospace';
  ctx.fillText('0', pad-6, H-pad+10);
  ctx.fillText(X_MAX+'mm', W-pad-16, H-pad+10);

  // Origin marker
  ctx.fillStyle = '#3a3a6a';
  ctx.fillRect(pad-1, H-pad-1, 3, 3);

  // Current position dot + crosshair
  if (gx === undefined) {
    gx = curStatus.gantry_x || 0;
    gy = curStatus.gantry_y || 0;
  }
  var cx = pad + gx*sc;
  var cy = H - pad - gy*sc;

  // Clamp to canvas
  cx = Math.max(pad, Math.min(W-pad, cx));
  cy = Math.max(pad, Math.min(H-pad, cy));

  ctx.strokeStyle = '#00d4aa';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx-10,cy); ctx.lineTo(cx+10,cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx,cy-10); ctx.lineTo(cx,cy+10); ctx.stroke();
  ctx.fillStyle = '#00d4aa';
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2*Math.PI); ctx.fill();

  // Position readout on canvas
  ctx.fillStyle = '#00d4aa';
  ctx.font = '9px monospace';
  ctx.fillText('('+gx.toFixed(0)+', '+gy.toFixed(0)+')', pad+2, pad-6);
}

// ── Emergency stop ────────────────────────────────────────────────
function doEstop() {
  fetch('/api/hw/estop', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      var btn = document.getElementById('estop-btn');
      if (d.ok) {
        btn.classList.add('fired');
        btn.textContent = '⚠ STOPPED';
        setTimeout(function(){
          btn.classList.remove('fired');
          btn.textContent = '☠ E-STOP';
          fetch('/api/hw/estop/clear', {method:'POST'});
        }, 3000);
      }
    }).catch(function(){});
}

// ── Clock controls ────────────────────────────────────────────────
function clockAct(action) {
  fetch('/api/clock/'+action, {method:'POST'}).catch(function(){});
}

// ── Direct step command ───────────────────────────────────────────
function sendStep(a, b) {
  var sa = a !== undefined ? a : +document.getElementById('step-a').value;
  var sb = b !== undefined ? b : +document.getElementById('step-b').value;
  var sp = +document.getElementById('step-spd').value;
  fetch('/api/gantry/step', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({steps_a: sa, steps_b: sb, speed: sp})
  }).catch(function(){});
}

// ── Hardware buttons ──────────────────────────────────────────────
function hwPost(url, label) {
  fetch(url, {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      var msg = (d.ok ? '✓ ' : '✗ ') + (d.msg||'');
      if (label==='engage' || label==='release') {
        document.getElementById('hw-magnet-t').textContent = d.ok ? label : 'failed';
      }
      if (label==='clock')  document.getElementById('hw-clock-msg').textContent = msg;
      if (label==='home')   document.getElementById('hw-home-msg').textContent = msg;
    }).catch(function(){});
}

// ── FEN inject ────────────────────────────────────────────────────
function injectFen() {
  var fen = document.getElementById('fen-input').value.trim();
  if (!fen) return;
  fetch('/api/fen',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({fen:fen})});
}

// ── Test runner ───────────────────────────────────────────────────
function buildCatalogue(cat) {
  catalogue = cat;
  var sel = document.getElementById('test-cat');
  sel.innerHTML = '';
  Object.keys(cat).forEach(function(k) {
    var o = document.createElement('option');
    o.value = k; o.textContent = k;
    sel.appendChild(o);
  });
  updateSubtests();

  var html = '';
  Object.keys(cat).forEach(function(k) {
    html += '<b>'+k+'</b>: ';
    html += cat[k].map(function(s){
      return '<a href="#" onclick="quickRun(\''+k+'\',\''+s+'\');return false">'+s+'</a>';
    }).join(' · ');
    html += '<br>';
  });
  document.getElementById('cat-list').innerHTML = html;
}

function updateSubtests() {
  var cat = document.getElementById('test-cat').value;
  var sel = document.getElementById('test-sub');
  sel.innerHTML = '';
  (catalogue[cat]||[]).forEach(function(s) {
    var o = document.createElement('option');
    o.value = s; o.textContent = s;
    sel.appendChild(o);
  });
}

function quickRun(cat, sub) {
  // Switch to tests tab
  document.querySelectorAll('.tab-btn').forEach(function(b){
    if (b.dataset.tab==='tests') b.click();
  });
  document.getElementById('test-cat').value = cat;
  updateSubtests();
  document.getElementById('test-sub').value = sub;
  runTest();
}

function runTest() {
  var cat = document.getElementById('test-cat').value;
  var sub = document.getElementById('test-sub').value;
  fetch('/api/tests/run',{method:'POST',
    headers:{'Content-Type':'application/json'},
    body:JSON.stringify({category:cat, subtest:sub})
  }).then(function(r){return r.json();}).then(function(d){
    if (d.ok) {
      document.getElementById('test-status').textContent = 'Running…';
      document.getElementById('test-status').className = 'badge b-executing_move';
      document.getElementById('test-last').textContent = cat+'/'+sub;
      connectTestSSE();
    } else {
      alert(d.msg);
    }
  });
}

function stopTest() {
  fetch('/api/tests/stop',{method:'POST'});
  if (testSSE) { testSSE.close(); testSSE = null; }
}

function connectTestSSE() {
  if (testSSE) testSSE.close();
  testSSE = new EventSource('/api/tests/stream');
  testSSE.onmessage = function(e) {
    var data; try { data = JSON.parse(e.data); } catch(ex){ return; }
    if (data.type === 'ping') return;
    var line = document.createElement('div');
    line.className = 't-line';
    var txt = data.line || '';
    if      (txt.match(/PASS|PASSED|✓/i)) line.classList.add('t-ok');
    else if (txt.match(/FAIL|FAILED|ERROR|✗/i)) line.classList.add('t-err');
    else if (txt.match(/WARN|⚠/i))              line.classList.add('t-warn');
    else if (txt.match(/^(Sourcing|Running|WARNING)/)) line.classList.add('t-dim');
    line.textContent = txt;
    var out = document.getElementById('test-output');
    out.appendChild(line);
    out.scrollTop = out.scrollHeight;
    if (data.type === 'done') {
      testSSE.close(); testSSE = null;
      var ok = data.rc === 0;
      document.getElementById('test-status').textContent = ok ? 'Passed' : 'Failed';
      document.getElementById('test-status').className   = 'badge '+(ok?'b-idle':'b-game_over');
      var finish = document.createElement('div');
      finish.className = 't-line t-done';
      finish.textContent = ok ? '✓ Test passed' : '✗ Test failed (rc='+data.rc+')';
      out.appendChild(finish);
      out.scrollTop = out.scrollHeight;
    }
  };
  testSSE.onerror = function() { testSSE = null; };
}

function clearOutput()  { document.getElementById('test-output').innerHTML = ''; }
function scrollBottom() { var el = document.getElementById('test-output'); el.scrollTop = el.scrollHeight; }

// ── Status poll — master heartbeat at 500 ms ──────────────────────
var STATE_LBL = {
  'OFFLINE':'Offline','STARTUP':'Startup','HOMING':'Homing',
  'IDLE':'Idle','WAITING_PLAYER_MOVE':'Waiting for Player',
  'CAPTURING_BOARD':'Capturing','VALIDATING_MOVE':'Validating',
  'CALCULATING_RESPONSE':'Engine Thinking…','EXECUTING_MOVE':'Executing Move',
  'HITTING_CLOCK':'Hitting Clock','PROMOTION_WAIT':'Promotion Wait',
  'GAME_OVER':'Game Over'
};

function pollStatus() {
  fetch('/api/status').then(function(r){return r.json();}).then(function(d){
    curStatus = d;

    // Boards
    if (d.fen) {
      updateBoards(d.fen);
      document.getElementById('fen-display').textContent = d.fen;
      document.getElementById('sc-fen').textContent = d.fen;
      var fi = document.getElementById('fen-input');
      if (!fi.matches(':focus')) fi.value = d.fen;
      var parts = d.fen.split(' ');
      if (parts.length >= 6) {
        document.getElementById('gs-move').textContent = 'Move '+parts[5];
        document.getElementById('sc-move').textContent = 'Move '+parts[5];
      }
    }

    // Game state
    var gs  = (d.game_state||'OFFLINE').toUpperCase();
    var gEl = document.getElementById('gs-state');
    gEl.textContent = STATE_LBL[gs] || gs.replace(/_/g,' ');
    gEl.className   = 'badge b-'+(gs.toLowerCase());
    document.getElementById('sc-state').textContent = STATE_LBL[gs]||gs;

    // Turn
    var t = (d.game_turn||'').toUpperCase();
    var tEl = document.getElementById('gs-turn');
    var scT = document.getElementById('sc-turn');
    if (t==='WHITE') {
      tEl.innerHTML='<span style="color:#f0ede0">⬜ White</span>'; scT.textContent='⬜ White';
    } else if (t==='BLACK') {
      tEl.innerHTML='<span style="color:#a0a0c0">⬛ Black</span>'; scT.textContent='⬛ Black';
    } else {
      tEl.textContent='—'; scT.textContent='—';
    }

    // FEN source
    var src = d.fen_source||'local';
    var sEl = document.getElementById('gs-source');
    var scS = document.getElementById('sc-source');
    var sm  = {game_mgr:['Game Mgr ✓','b-src-game'],
               local:['Local','b-src-local']}[src] || ['Local','b-src-local'];
    sEl.textContent = sm[0]; sEl.className = 'badge '+sm[1];
    scS.textContent = sm[0];

    // Last detected changed squares (from perception tab)
    var lmEl = document.getElementById('perc-lastmove');
    if (lmEl) {
      lmEl.textContent = d.last_move ? d.last_move.split(',').join(' → ') : '–';
    }

    // Move history
    if (d.move_history && d.move_history.length) {
      var mh = '';
      d.move_history.forEach(function(m,i) {
        if (i%2===0) mh += '<span class="mn">'+(Math.floor(i/2)+1)+'. </span>';
        mh += '<span class="mv">'+m+' </span>';
      });
      document.getElementById('move-hist').innerHTML = mh;
    }

    // Connection stats
    var age = d.last_updated > 0 ? (Date.now()/1000 - d.last_updated) : 9999;
    var ae  = document.getElementById('st-age');
    ae.textContent = age < 9999 ? age.toFixed(1) : 'never';
    ae.className   = age < 4 ? 'ok' : age < 15 ? 'warn' : 'err';
    document.getElementById('st-frames').textContent = d.frame_count;
    document.getElementById('st-cam').textContent    = d.cam_info || '—';
    document.getElementById('st-ros').textContent    = d.ros_connected ? 'Connected ✓' : 'Not connected';
    document.getElementById('st-ros').className      = 'stat-val '+(d.ros_connected?'ok':'warn');
    document.getElementById('st-err').textContent    = d.error || '—';
    document.getElementById('sc-cam').textContent    = d.cam_info || '—';

    // Status pills
    setPill('pill-cam',  age < 5);
    setPill('pill-ros',  d.ros_connected);
    setPill('pill-game', gs !== 'OFFLINE' && gs !== 'IDLE');
    document.getElementById('cam-ts').textContent =
      age < 9999 ? age.toFixed(1)+'s ago' : '—';
    document.getElementById('cam-footer').textContent =
      (d.cam_info||'—') + (d.error ? ' ⚠ '+d.error : '');

    // Gantry
    if (d.gantry_x !== undefined) {
      setGantryPos(d.gantry_x, d.gantry_y);
      if (d.gantry_homed) {
        document.getElementById('home-badge').textContent = '✓ Homed';
        document.getElementById('home-badge').style.color = 'var(--ok)';
      }
    }

    // Limits
    setLim('lim-x','lim-x-t','hw-lim-x', d.limit_x);
    setLim('lim-y','lim-y-t','hw-lim-y', d.limit_y);
    setLim('lim-c','lim-c-t','hw-lim-c', d.limit_clock);

    // Magnet
    var mg = document.getElementById('hw-magnet');
    if (mg) mg.className = 'indicator'+(d.magnet_engaged?' on':'');
    var mt = document.getElementById('hw-magnet-t');
    if (mt) mt.textContent = d.magnet_engaged ? 'Engaged' : 'Released';

    // Stepper / gantry status
    var ssEl = document.getElementById('hw-stepper-status');
    if (ssEl) {
      ssEl.textContent = d.stepper_status || 'UNKNOWN';
      ssEl.className = 'badge ' + (d.stepper_status==='MOVING' ? 'b-executing_move' :
                                   d.stepper_status==='IDLE'   ? 'b-idle' : 'b-offline');
    }
    var gsEl = document.getElementById('hw-gantry-status');
    if (gsEl) gsEl.textContent = d.gantry_status || '—';
    var hoEl = document.getElementById('hw-homed-status');
    if (hoEl) {
      hoEl.textContent = d.gantry_homed ? '✓ Yes' : '✗ No';
      hoEl.className   = d.gantry_homed ? 'ok' : 'warn';
    }

    // Chess clock
    updateClock(d.white_time, d.black_time, d.game_turn, d.game_state);

    // E-stop visual state
    if (d.estop_active) {
      document.getElementById('estop-btn').classList.add('fired');
    }

  }).catch(function(){
    setPill('pill-cam', false);
    setPill('pill-ros', false);
  });
}

function fmtTime(s) {
  if (s === undefined || s === null) return '--:--';
  var t = Math.max(0, Math.round(s));
  var m = Math.floor(t / 60);
  var sec = t % 60;
  return (m < 10 ? '0' : '') + m + ':' + (sec < 10 ? '0' : '') + sec;
}

function updateClock(wt, bt, turn, state) {
  var wEl  = document.getElementById('white-time-disp');
  var bEl  = document.getElementById('black-time-disp');
  var wBox = document.getElementById('clock-white');
  var bBox = document.getElementById('clock-black');
  var lbl  = document.getElementById('clock-state-lbl');
  if (!wEl) return;

  wEl.textContent = fmtTime(wt);
  bEl.textContent = fmtTime(bt);

  var wActive = (turn||'').toUpperCase() === 'WHITE' && state !== 'STOPPED' && state !== 'IDLE' && state !== 'OFFLINE';
  var bActive = (turn||'').toUpperCase() === 'BLACK' && state !== 'STOPPED' && state !== 'IDLE' && state !== 'OFFLINE';

  wBox.classList.toggle('active-clock', wActive);
  bBox.classList.toggle('active-clock', bActive);
  wBox.classList.toggle('low-time', wt !== undefined && wt < 30 && wActive);
  bBox.classList.toggle('low-time', bt !== undefined && bt < 30 && bActive);

  if (lbl) lbl.textContent = (state && state !== 'OFFLINE') ? '— ' + state : '';
}

function setPill(id, live) {
  var el = document.getElementById(id);
  el.className = 'pill' + (live ? ' live' : '');
}
function setLim(dotId, txtId, hwId, val) {
  var d = document.getElementById(dotId);
  if (d) d.className = 'indicator'+(val?' on':'');
  if (txtId) {
    var t = document.getElementById(txtId);
    if (t) { t.textContent=val?'TRIGGERED':'—'; t.className='stat-val '+(val?'warn':''); }
  }
  var h = document.getElementById(hwId);
  if (h) h.className = 'indicator'+(val?' on':'');
}

// ── Boot ──────────────────────────────────────────────────────────
fetch('/api/params').then(function(r){return r.json();}).then(function(p){
  curParams = p;
  buildParamSliders(p);
  buildOverlayToggles(p);
});
fetch('/api/tests/catalogue').then(function(r){return r.json();}).then(buildCatalogue);
buildHandles();
buildCornerEditor();
loadCornersFromServer();
updateCornerInfo();
drawCanvas(0, 0);
setInterval(pollStatus, 500);
pollStatus();
</script>
</body>
</html>"""

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ChessOS — Smart Chess Board UI")
    parser.add_argument("--port",   type=int, default=5000)
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true",
                        help="Disable ROS2 integration, use OpenCV camera")
    parser.add_argument("--camera", type=int, default=0,
                        help="OpenCV camera device index (no-ROS mode)")
    parser.add_argument("--image",  default=None,
                        help="Static image for offline testing")
    parser.add_argument("--mode",   choices=["showcase", "debug"], default="debug",
                        help="Initial display mode")
    args = parser.parse_args()

    _load_corners()

    # Static image
    if args.image:
        frame = cv2.imread(args.image)
        if frame is not None:
            with _lock:
                _state["raw_frame"] = frame
                _state["cam_info"]  = f"static {frame.shape[1]}×{frame.shape[0]}"
            _camera.update_raw(_to_jpeg(frame, 85))
            print(f"  ✓ Static image: {args.image}")
        else:
            print(f"  ⚠  Cannot load: {args.image}")

    # Background threads
    threading.Thread(target=_vision_loop, daemon=True).start()
    threading.Thread(target=_jog_loop,    daemon=True).start()

    # ROS2 or OpenCV camera
    if not args.no_ros:
        if not HAS_ROS:
            print("  ⚠  rclpy not available — falling back to OpenCV camera")
            args.no_ros = True
        else:
            global _ros_node
            try:
                rclpy.init()
                _ros_node = _RosNode()
                threading.Thread(
                    target=lambda: rclpy.spin(_ros_node),
                    daemon=True).start()
                print("  ✓ ROS2 node started")
                print("    Subscribing: /camera/image_raw, /game_manager/{board_fen,state,turn}")
            except Exception as e:
                print(f"  ⚠  ROS2 init failed: {e}")
                args.no_ros = True

    if args.no_ros and args.image is None:
        print(f"  OpenCV camera index {args.camera}")
        threading.Thread(
            target=_opencv_camera_loop, args=(args.camera,),
            daemon=True).start()

    print(f"\n  ♞  ChessOS")
    print(f"     http://localhost:{args.port}")
    print(f"     http://192.168.1.149:{args.port}")
    print(f"     Mode:   {args.mode}")
    print(f"     ROS2:   {'enabled' if not args.no_ros else 'disabled'}")
    print(f"     Tests:  {_TEST_SCRIPT}\n")

    app.run(host=args.host, port=args.port,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
