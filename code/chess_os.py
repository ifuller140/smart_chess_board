#!/usr/bin/env python3
"""
ChessOS — Master Control Interface for Smart Chess Board

Unified dashboard: live camera (MJPEG), showcase display, full debug
controls for gantry, hardware, perception, game logic, and test runner.

Usage:
    python3 code/chess_os.py [--port 5000] [--no-ros] [--camera 0]
    python3 code/chess_os.py --mode showcase
    python3 code/chess_os.py --no-ros --image /path/to/frame.jpg

Access: http://<pi-ip>:5000  (IP printed at startup)
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

try:
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    HAS_RCL_PARAMS = True
except ImportError:
    HAS_RCL_PARAMS = False

# ── Paths ─────────────────────────────────────────────────────────────────────
_ROOT         = Path(__file__).parent.parent
_CORNERS_FILE = _ROOT / "board_corners.json"
_CALIB_FILE   = _ROOT / "board_calibration.json"
_TEST_SCRIPT  = _ROOT / "run_hw_test.sh"

# ── Test catalogue ────────────────────────────────────────────────────────────
TEST_CATALOGUE: Dict[str, List[str]] = {
    "gantry": ["full","limits","pulse","motor_a","motor_b","corexy",
               "speed_sweep","repeatability","enable_hold","homing",
               "manual","lockstep","diagonal_sync","square_return",
               "raw_motor","timing_sweep","square_nav","calibrate"],
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
    Uses board_calibration.json when applied; falls back to defaults.
    +X = rightward (a=left, h=right), +Y = backward (rank1=front, rank8=back).
    """
    sq = sq.lower().strip()
    if len(sq) != 2:
        return None, None
    fi = ord(sq[0]) - ord('a')
    try:
        ri = int(sq[1]) - 1
    except ValueError:
        return None, None
    if not (0 <= fi < 8 and 0 <= ri < 8):
        return None, None
    with _lock:
        a1x = _state.get("calib_a1_x")
        a1y = _state.get("calib_a1_y")
        sqx = _state.get("calib_sq_x")
        sqy = _state.get("calib_sq_y")
    if a1x is not None and sqx is not None and sqy is not None:
        return round(a1x + fi * sqx, 2), round(a1y + ri * sqy, 2)
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
    # Board calibration
    "calib_a1_x":    None,
    "calib_a1_y":    None,
    "calib_h8_x":    None,
    "calib_h8_y":    None,
    "calib_sq_x":    None,
    "calib_sq_y":    None,
    "calib_applied": False,
    # Game control
    "move_history_uci":           [],
    "game_result":                "",
    "session_reference_captured": False,
    "think_time_s":               2.0,
    # Perception diff data
    "square_scores":              {},
    "diff_frame":                 None,
    "ref_status":                 "No reference captured. Click 'Capture Reference' first.",
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

def _load_calibration():
    """Load board_calibration.json into _state at startup."""
    if not _CALIB_FILE.exists():
        return
    try:
        c = json.loads(_CALIB_FILE.read_text())
        sq_x = float(c["sq_x"])
        sq_y = float(c["sq_y"])
        with _lock:
            _state["calib_a1_x"]    = float(c["a1_x"])
            _state["calib_a1_y"]    = float(c["a1_y"])
            _state["calib_h8_x"]    = float(c["h8_x"])
            _state["calib_h8_y"]    = float(c["h8_y"])
            _state["calib_sq_x"]    = sq_x
            _state["calib_sq_y"]    = sq_y
            _state["calib_applied"] = True
        print(f"  ✓ Board calibration: a1=({c['a1_x']:.1f},{c['a1_y']:.1f})"
              f" sq=({sq_x:.1f}×{sq_y:.1f}mm)")
    except Exception as e:
        print(f"  ⚠  Calibration load failed: {e}")


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
    # Diff detection (pushed to piece_detector_node via SetParameters)
    "display_threshold":   18.0,
    "shift_compensation":   1.0,
    "clump_enable":          0,
    "clump_keep_per_group":  1,
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
            # Game control / perception data from ROS
            self.create_subscription(String,  "/game_manager/move_history",  self._on_move_history,   10)
            self.create_subscription(String,  "/game_manager/result",        self._on_result,         10)
            self.create_subscription(String,  "/perception/square_scores",   self._on_square_scores,  10)
            self.create_subscription(Image,   "/perception/piece_debug",     self._on_diff_img,       10)
            self.create_subscription(String,  "/perception/reference_status",self._on_ref_status,     10)

            # ── Publishers ─────────────────────────────────────────────
            self.vel_pub       = self.create_publisher(Twist,    "/stepper/velocity",       10)
            self.cmd_pub       = self.create_publisher(RosPoint, "/stepper/command",        10)
            self.estop_pub     = self.create_publisher(Bool,     "/emergency_stop",         10)
            self.reset_pos_pub = self.create_publisher(Bool,     "/stepper/reset_position", 10)
            self.set_time_pub  = self.create_publisher(Float32,  "/clock/set_time",         10)

            # ── Service clients ────────────────────────────────────────
            self._svc_engage       = self.create_client(Trigger, "/servo/engage")
            self._svc_release      = self.create_client(Trigger, "/servo/release")
            self._svc_home         = self.create_client(Trigger, "/gantry/home")
            self._svc_clock        = self.create_client(Trigger, "/clock/hit")
            self._svc_clock_reset  = self.create_client(Trigger, "/clock/reset")
            self._svc_clock_pause  = self.create_client(Trigger, "/clock/pause")
            self._svc_clock_resume = self.create_client(Trigger, "/clock/resume")
            # Game control services
            self._svc_game_start    = self.create_client(Trigger, "/game/start")
            self._svc_game_new      = self.create_client(Trigger, "/game/new_game")
            self._svc_game_resign   = self.create_client(Trigger, "/game/resign")
            self._svc_cap_reference = self.create_client(Trigger, "/perception/capture_premove")
            # Detector parameter client (optional — requires rcl_interfaces)
            self._svc_detector_params = None
            if HAS_RCL_PARAMS:
                self._svc_detector_params = self.create_client(
                    SetParameters, "/piece_detector_node/set_parameters")

            # Pending detector param update (set by Flask thread, applied by ROS timer)
            self._pending_detector_params = None
            self.create_timer(0.2, self._check_pending)

            # ── Action client for point-to-point gantry moves ─────────
            try:
                from chess_interfaces.action import MoveGantry as _MGA
                from rclpy.action import ActionClient as _AClient
                self._move_ac = _AClient(self, _MGA, '/gantry/move')
                self._has_move_action = True
            except ImportError:
                self._move_ac = None
                self._has_move_action = False

            with _lock:
                _state["ros_connected"] = True
            self.get_logger().info("ChessOS ROS2 node ready")

        def send_goto(self, x_mm: float, y_mm: float, speed: float = 40.0) -> dict:
            """Fire-and-forget MoveGantry action. Position updates via /gantry/pose."""
            if not self._has_move_action or self._move_ac is None:
                return {"ok": False, "msg": "chess_interfaces not installed"}
            if not self._move_ac.wait_for_server(timeout_sec=1.0):
                return {"ok": False, "msg": "/gantry/move action server not running"}
            try:
                from chess_interfaces.action import MoveGantry as _MGA
                goal = _MGA.Goal()
                goal.target_x_mm   = float(x_mm)
                goal.target_y_mm   = float(y_mm)
                goal.speed_mm_s    = float(speed)
                goal.engage_magnet = False
                self._move_ac.send_goal_async(goal)
                return {"ok": True, "msg": f"Moving to ({x_mm:.1f}, {y_mm:.1f}) mm"}
            except Exception as e:
                return {"ok": False, "msg": str(e)}

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

        def _on_move_history(self, msg):
            uci_str = msg.data.strip()
            uci_list = uci_str.split() if uci_str else []
            with _lock:
                _state["move_history_uci"] = uci_list
                _state["move_history"]     = uci_list

        def _on_result(self, msg):
            with _lock:
                _state["game_result"] = msg.data.strip()

        def _on_square_scores(self, msg):
            try:
                with _lock:
                    _state["square_scores"] = json.loads(msg.data)
            except Exception:
                pass

        def _on_diff_img(self, msg):
            try:
                h, w = msg.height, msg.width
                data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
                img  = data.reshape((h, w, 3))
                ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ok:
                    with _lock:
                        _state["diff_frame"] = bytes(buf)
            except Exception:
                pass

        def _on_ref_status(self, msg):
            with _lock:
                _state["ref_status"] = msg.data

        def request_detector_params(self, threshold, compensation,
                                    clump_enable=False, clump_keep=1):
            if HAS_RCL_PARAMS:
                self._pending_detector_params = {
                    "diff_threshold":            threshold,
                    "global_shift_compensation": compensation,
                    "clump_enable":              clump_enable,
                    "clump_keep_per_group":      clump_keep,
                }

        def _check_pending(self):
            params = self._pending_detector_params
            if params is None or not HAS_RCL_PARAMS:
                return
            if self._svc_detector_params is None:
                return
            self._pending_detector_params = None
            if not self._svc_detector_params.service_is_ready():
                return
            type_map = {
                "diff_threshold":            ParameterType.PARAMETER_DOUBLE,
                "global_shift_compensation": ParameterType.PARAMETER_DOUBLE,
                "clump_enable":              ParameterType.PARAMETER_BOOL,
                "clump_keep_per_group":      ParameterType.PARAMETER_INTEGER,
            }
            req = SetParameters.Request()
            for name, value in params.items():
                pv = ParameterValue()
                ptype = type_map.get(name, ParameterType.PARAMETER_DOUBLE)
                pv.type = ptype
                if ptype == ParameterType.PARAMETER_DOUBLE:
                    pv.double_value = float(value)
                elif ptype == ParameterType.PARAMETER_BOOL:
                    pv.bool_value = bool(int(value))
                elif ptype == ParameterType.PARAMETER_INTEGER:
                    pv.integer_value = int(value)
                p = Parameter()
                p.name = name
                p.value = pv
                req.parameters.append(p)
            self._svc_detector_params.call_async(req)

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
            "calib_applied":  _state["calib_applied"],
            "calib_a1_x":     _state["calib_a1_x"],
            "calib_a1_y":     _state["calib_a1_y"],
            "calib_h8_x":     _state["calib_h8_x"],
            "calib_h8_y":     _state["calib_h8_y"],
            "calib_sq_x":     _state["calib_sq_x"],
            "calib_sq_y":     _state["calib_sq_y"],
            # Game control
            "move_history_uci":           _state["move_history_uci"],
            "game_result":                _state["game_result"],
            "think_time_s":               _state.get("think_time_s", 2.0),
            "session_reference_captured": _state["session_reference_captured"],
            "ref_status":                 _state["ref_status"],
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
    """Backward-compatible alias for /api/perception/capture_reference."""
    return api_capture_reference()


@app.route("/api/perception/capture_reference", methods=["POST"])
def api_capture_reference():
    """Call /perception/capture_premove and mark session_reference_captured."""
    resp = _call_svc("_svc_cap_reference")
    if not isinstance(resp, tuple):
        with _lock:
            _state["session_reference_captured"] = True
            _state["ref_status"] = "Reference captured ✓"
    return resp


@app.route("/api/diff_frame")
def api_diff_frame():
    """Piece detector LAB diff heatmap (from /perception/piece_debug)."""
    with _lock:
        data = _state.get("diff_frame")
    if not data:
        data = _blank_jpeg(400, 400, "Waiting for piece_detector...")
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/square_scores")
def api_square_scores():
    with _lock:
        return jsonify({
            "scores":               dict(_state.get("square_scores", {})),
            "display_threshold":    _params.get("display_threshold", 18.0),
            "shift_compensation":   _params.get("shift_compensation", 1.0),
            "clump_enable":         int(_params.get("clump_enable", 0)),
            "clump_keep_per_group": int(_params.get("clump_keep_per_group", 1)),
        })


@app.route("/api/detector_params", methods=["POST"])
def api_detector_params():
    """Push diff threshold + compensation + clump settings to piece_detector_node."""
    data = request.get_json(silent=True) or {}
    thresh       = float(data.get("diff_threshold", 18.0))
    comp         = float(data.get("shift_compensation", 1.0))
    clump_enable = bool(data.get("clump_enable", False))
    clump_keep   = int(data.get("clump_keep_per_group", 1))
    with _lock:
        _params["display_threshold"]    = thresh
        _params["shift_compensation"]   = comp
        _params["clump_enable"]         = int(clump_enable)
        _params["clump_keep_per_group"] = clump_keep
    if _ros_node is not None and HAS_RCL_PARAMS:
        _ros_node.request_detector_params(thresh, comp, clump_enable, clump_keep)
        return jsonify({"ok": True,
                        "msg": f"Applied threshold={thresh}, comp={comp}, "
                               f"clump={clump_enable}(keep={clump_keep})"})
    elif _ros_node is not None:
        return jsonify({"ok": False,
                        "msg": "rcl_interfaces not available — params stored locally only"}), 503
    return jsonify({"ok": False, "msg": "ROS not connected"}), 503


# ── Game control routes ───────────────────────────────────────────────────────

@app.route("/api/game/start", methods=["POST"])
def api_game_start():
    """Trigger game start (same as pressing the physical clock from IDLE)."""
    return _call_svc("_svc_game_start")


@app.route("/api/game/new", methods=["POST"])
def api_game_new():
    """Reset to a fresh game."""
    with _lock:
        _state["game_result"]       = ""
        _state["move_history_uci"]  = []
        _state["move_history"]      = []
    return _call_svc("_svc_game_new")


@app.route("/api/game/resign", methods=["POST"])
def api_game_resign():
    """Resign the current game."""
    return _call_svc("_svc_game_resign")


@app.route("/api/game/settings", methods=["POST"])
def api_game_settings():
    """Update think time and/or clock time per player."""
    data = request.get_json(silent=True) or {}
    msgs = []
    if "think_time_s" in data:
        with _lock:
            _state["think_time_s"] = float(data["think_time_s"])
        msgs.append(f"think_time={data['think_time_s']}s (stored; restart game_manager to apply)")
    if "time_per_player_min" in data and _ros_node is not None and HAS_ROS:
        try:
            m = Float32()
            m.data = float(data["time_per_player_min"])
            _ros_node.set_time_pub.publish(m)
            msgs.append(f"clock={data['time_per_player_min']}min")
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500
    return jsonify({"ok": True, "applied": msgs})


# ── Node health ───────────────────────────────────────────────────────────────

EXPECTED_NODES = [
    "stepper_driver_node", "servo_node", "limit_switch_node",
    "clock_display_node",  "camera_node", "board_detector_node",
    "piece_detector_node", "gantry_kinematics_node", "motion_planner_node",
    "homing_node",         "game_manager_node",       "chess_engine_node",
]


@app.route("/api/nodes/health")
def api_nodes_health():
    if _ros_node is None or not HAS_ROS:
        return jsonify({n: False for n in EXPECTED_NODES})
    try:
        alive = set(_ros_node.get_node_names())
        return jsonify({n: (n in alive) for n in EXPECTED_NODES})
    except Exception as e:
        return jsonify({"error": str(e)}), 500


# ─────────────────── (old api_capture_premove body kept below for ref)


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
    speed = float(data.get("speed", 40.0))
    # Attempt physical move via action server
    moved = False
    move_msg = "ROS not connected"
    if _ros_node is not None:
        r = _ros_node.send_goto(x, y, speed)
        moved    = r["ok"]
        move_msg = r.get("msg", "")
    # Always update local display state
    with _lock:
        _state["gantry_x"] = x
        _state["gantry_y"] = y
    return jsonify({"ok": True, "x": x, "y": y, "moved": moved, "msg": move_msg})


# ── Board calibration routes ───────────────────────────────────────────────────

@app.route("/api/gantry/calibration", methods=["GET"])
def api_calib_get():
    with _lock:
        return jsonify({
            "applied": _state["calib_applied"],
            "a1_x":    _state["calib_a1_x"],
            "a1_y":    _state["calib_a1_y"],
            "h8_x":    _state["calib_h8_x"],
            "h8_y":    _state["calib_h8_y"],
            "sq_x":    _state["calib_sq_x"],
            "sq_y":    _state["calib_sq_y"],
        })

@app.route("/api/gantry/calibration/save_a1", methods=["POST"])
def api_calib_save_a1():
    with _lock:
        x = _state["gantry_x"]
        y = _state["gantry_y"]
        _state["calib_a1_x"] = x
        _state["calib_a1_y"] = y
        _state["calib_applied"] = False  # require re-apply
    return jsonify({"ok": True, "x": x, "y": y})

@app.route("/api/gantry/calibration/save_h8", methods=["POST"])
def api_calib_save_h8():
    with _lock:
        x = _state["gantry_x"]
        y = _state["gantry_y"]
        _state["calib_h8_x"] = x
        _state["calib_h8_y"] = y
        _state["calib_applied"] = False
    return jsonify({"ok": True, "x": x, "y": y})

@app.route("/api/gantry/calibration/apply", methods=["POST"])
def api_calib_apply():
    with _lock:
        a1x = _state["calib_a1_x"]
        a1y = _state["calib_a1_y"]
        h8x = _state["calib_h8_x"]
        h8y = _state["calib_h8_y"]
    if None in (a1x, a1y, h8x, h8y):
        return jsonify({"ok": False, "msg": "Save both a1 and h8 first"}), 400
    sq_x = (h8x - a1x) / 7.0
    sq_y = (h8y - a1y) / 7.0
    if sq_x < 5.0 or sq_y < 5.0:
        return jsonify({"ok": False,
                        "msg": f"Square size too small ({sq_x:.1f}×{sq_y:.1f}mm). "
                               "Check that a1 and h8 are the correct corners."}), 400
    with _lock:
        _state["calib_sq_x"]    = sq_x
        _state["calib_sq_y"]    = sq_y
        _state["calib_applied"] = True
    try:
        _CALIB_FILE.write_text(json.dumps({
            "a1_x": a1x, "a1_y": a1y,
            "h8_x": h8x, "h8_y": h8y,
            "sq_x": sq_x, "sq_y": sq_y,
            "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }, indent=2))
    except Exception as e:
        return jsonify({"ok": True, "sq_x": sq_x, "sq_y": sq_y,
                        "a1_x": a1x, "a1_y": a1y, "h8_x": h8x, "h8_y": h8y,
                        "warn": f"Saved in memory; file write failed: {e}"})
    return jsonify({"ok": True, "sq_x": sq_x, "sq_y": sq_y,
                    "a1_x": a1x, "a1_y": a1y, "h8_x": h8x, "h8_y": h8y})

@app.route("/api/gantry/calibration/clear", methods=["POST"])
def api_calib_clear():
    with _lock:
        for k in ("calib_a1_x","calib_a1_y","calib_h8_x","calib_h8_y",
                  "calib_sq_x","calib_sq_y"):
            _state[k] = None
        _state["calib_applied"] = False
    try:
        _CALIB_FILE.unlink(missing_ok=True)
    except Exception:
        pass
    return jsonify({"ok": True})

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

/* ── Diff analysis (Perception tab) ──────────────────────────────── */
.diff-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:start}
@media(max-width:700px){.diff-grid{grid-template-columns:1fr}}
#score-canvas{display:block;border:1px solid var(--border);border-radius:4px;
              image-rendering:pixelated;width:100%;max-width:360px;aspect-ratio:1}
#diff-heatmap{width:100%;display:block;image-rendering:pixelated;border-radius:4px;
              border:1px solid var(--border)}
.ref-banner{
  padding:10px 14px;border-radius:6px;margin-bottom:10px;
  display:flex;align-items:center;gap:10px;flex-wrap:wrap;
}
.ref-banner.needs{background:#2a1400;border:2px solid var(--warn);color:var(--warn)}
.ref-banner.done{background:#082a14;border:2px solid var(--ok);color:var(--ok)}
.flagged-row{padding:5px 0;font-size:.78em}
.flagged-row .lbl{color:var(--dim)}
#flagged-squares{color:var(--ok);font-family:monospace;font-weight:700;letter-spacing:.05em}
.median-row{padding:2px 0;font-size:.68em;color:var(--dim)}
#median-floor{color:var(--accent);font-family:monospace}
.slider-row{display:flex;align-items:center;gap:6px;margin:5px 0}
.slider-row label{flex:0 0 150px;font-size:.7em;color:var(--dim);
                  overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.slider-row input[type=range]{flex:1;accent-color:var(--accent);height:4px;min-width:80px}
.slider-row .vb{flex:0 0 42px;text-align:right;font-family:monospace;font-size:.72em;color:var(--text)}
.tog-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.tog-row label{font-size:.7em;color:var(--dim);cursor:pointer}
.tog-row input[type=checkbox]{accent-color:var(--accent);width:13px;height:13px;cursor:pointer}

/* ── Node health (Hardware tab) ───────────────────────────────────── */
.node-grid{display:grid;grid-template-columns:repeat(2,1fr);gap:4px 14px;padding:4px 0}
.node-row{display:flex;align-items:center;gap:6px;font-size:.72em;font-family:monospace}
.node-dot{width:8px;height:8px;border-radius:50%;flex-shrink:0}
.node-dot.ok{background:var(--ok)}
.node-dot.err{background:var(--err)}
.node-dot.unk{background:var(--dim)}

/* ── Game controls (Game tab) ─────────────────────────────────────── */
.checklist-card{padding:10px 12px;border-radius:5px;
  background:var(--surf2);border:1px solid var(--border);margin-bottom:8px}
.checklist-title{font-size:.6em;text-transform:uppercase;letter-spacing:.09em;
  color:var(--dim);font-weight:700;margin-bottom:8px;padding-bottom:5px;
  border-bottom:1px solid var(--border)}
.check-row{display:flex;align-items:center;gap:8px;padding:3px 0;font-size:.75em}
.check-icon{width:14px;height:14px;border-radius:3px;flex-shrink:0;
  display:flex;align-items:center;justify-content:center;font-size:.9em}
.check-icon.ok{background:#082a14;color:var(--ok)}
.check-icon.no{background:#18182e;color:var(--dim)}
.promo-banner{
  background:#2a1e50;border:2px solid #8957e5;border-radius:6px;
  padding:12px 16px;margin-bottom:10px;display:none;
}
.promo-banner.visible{display:block}
.promo-banner .promo-title{color:#c084fc;font-weight:700;font-size:.85em;margin-bottom:6px}
.gameover-banner{
  background:#2a0810;border:2px solid var(--err);border-radius:6px;
  padding:12px 16px;margin-bottom:10px;display:none;
}
.gameover-banner.visible{display:block}
.gameover-banner .go-result{color:#ff8080;font-size:1em;font-weight:700;margin-bottom:8px}
.settings-row{display:flex;align-items:center;gap:8px;padding:4px 0;font-size:.75em}
.settings-row label{flex:0 0 140px;color:var(--dim)}
.settings-row input[type=range]{flex:1;accent-color:var(--accent)}
.settings-row input[type=number]{width:60px;background:#040408;color:var(--text);
  border:1px solid var(--border);border-radius:4px;padding:3px 6px;font-size:.9em;text-align:center}
.settings-row .vb{flex:0 0 42px;font-family:monospace;font-size:.72em;color:var(--text);text-align:right}
.game-ctrl-row{display:flex;gap:8px;flex-wrap:wrap;padding:6px 0}
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

      <!-- Promotion banner -->
      <div class="promo-banner" id="promo-banner">
        <div class="promo-title">&#9837; Pawn Promotion</div>
        <div style="font-size:.8em;color:#c9d1d9;margin-bottom:10px">
          The computer promoted a pawn! Place a <b>queen</b> on the indicated square,
          then click <b>OK</b> or press the physical clock button.
        </div>
        <button class="btn primary" onclick="gameAct('start')">&#10003; OK — Piece placed</button>
      </div>

      <!-- Game-over banner -->
      <div class="gameover-banner" id="gameover-banner">
        <div class="go-result" id="gameover-result">Game Over</div>
        <button class="btn" onclick="gameAct('new')" style="margin-top:4px">&#8635; New Game</button>
      </div>

      <div class="two-col">
        <div>
          <!-- Pre-game checklist -->
          <div class="checklist-card">
            <div class="checklist-title">Pre-Game Checklist</div>
            <div class="check-row">
              <div class="check-icon" id="chk-ros">?</div>
              <span>ROS connected</span>
            </div>
            <div class="check-row">
              <div class="check-icon" id="chk-homed">?</div>
              <span>Gantry homed</span>
            </div>
            <div class="check-row">
              <div class="check-icon" id="chk-calib">?</div>
              <span>Board calibrated</span>
            </div>
            <div class="check-row">
              <div class="check-icon" id="chk-ref">?</div>
              <span>Reference captured</span>
            </div>
            <div style="margin-top:10px;display:flex;gap:8px;flex-wrap:wrap">
              <button class="btn primary" id="start-game-btn"
                      onclick="gameAct('start')" disabled>&#9654; Start Game</button>
              <span id="start-game-hint" style="font-size:.65em;color:var(--warn);align-self:center"></span>
            </div>
          </div>

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

          <!-- Game controls -->
          <div class="card">
            <div class="card-hdr">Game Controls</div>
            <div class="card-body">
              <div class="game-ctrl-row">
                <button class="btn warn" onclick="confirmGameAct('new','Start a new game? Current game will be abandoned.')">&#8635; New Game</button>
                <button class="btn danger" onclick="confirmGameAct('resign','Resign this game?')">&#9873; Resign</button>
              </div>
            </div>
          </div>

          <!-- Game settings -->
          <div class="card">
            <div class="card-hdr">Game Settings</div>
            <div class="card-body">
              <div class="settings-row">
                <label title="Stockfish think time in seconds">Stockfish (s)</label>
                <input type="range" id="think-slider" min="0.5" max="10" step="0.5" value="2"
                  oninput="document.getElementById('think-vb').textContent=(+this.value).toFixed(1);sendGameSettings()">
                <span class="vb" id="think-vb">2.0</span>
              </div>
              <div class="settings-row">
                <label title="Time per player in minutes">Time / player (min)</label>
                <input type="number" id="clock-time-input" min="1" max="60" value="10"
                  style="width:60px;background:#040408;color:var(--text);border:1px solid var(--border);border-radius:4px;padding:3px 6px;font-size:.9em"
                  onchange="sendGameSettings()">
              </div>
              <div style="font-size:.62em;color:var(--dim);margin-top:6px">
                Clock time applies immediately. Think time takes effect next game (restart node to apply mid-game).
              </div>
            </div>
          </div>
        </div>
      </div>
    </div><!-- /tab-game -->

    <!-- ── PERCEPTION ───────────────────────────────────────────── -->
    <div class="tab-panel" id="tab-perception">

      <!-- Capture Reference banner (prominent — required before game start) -->
      <div class="ref-banner needs" id="ref-banner">
        <div style="flex:1">
          <div style="font-weight:700;font-size:.82em;margin-bottom:3px">
            &#9888; Capture Pre-Move Reference
          </div>
          <div style="font-size:.7em;line-height:1.5" id="ref-status-text">
            No reference captured. Required before starting a game — snapshots the current board as the move-detection baseline.
          </div>
        </div>
        <button class="btn warn" id="capture-ref-btn" onclick="doCaptureReference()">
          &#128247; Capture Reference
        </button>
      </div>

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
              <span class="stat-val" style="font-size:.72em;color:#58a6ff">LAB per-square diff</span>
            </div>
            <div class="stat-row">
              <span class="stat-lbl" style="color:var(--dim);font-size:.72em">Last Changed Squares</span>
              <span class="stat-val" id="perc-lastmove" style="font-family:monospace;color:#3fb950;font-size:.75em">–</span>
            </div>
            <div class="stat-row">
              <span class="stat-lbl" style="color:var(--dim);font-size:.72em">Reference Status</span>
              <span class="stat-val" id="ref-status-inline" style="font-size:.7em;color:var(--warn)">–</span>
            </div>
            <div style="font-size:.63em;color:var(--dim);margin-top:8px;line-height:1.6;background:#040408;padding:8px;border-radius:4px">
              <b style="color:var(--text)">How it works:</b>
              game_manager enters <b>WAITING_PLAYER_MOVE</b> → calls /perception/capture_premove.
              Human moves, presses clock → piece_detector compares every frame to the reference.
              Per-square LAB diff scores are published to /perception/square_scores.
              After global shift compensation, squares above diff_threshold are flagged as changed.
              game_manager matches changed squares to a legal move.
            </div>
          </div>
        </div>
      </div>

      <!-- Diff analysis section (fen_visualizer pipeline) -->
      <div class="card">
        <div class="card-hdr">Diff Analysis — Threshold Tuning</div>
        <div class="card-body">
          <div class="diff-grid">
            <!-- Left: heatmap from ROS -->
            <div>
              <div style="font-size:.63em;color:var(--dim);margin-bottom:6px;text-transform:uppercase;letter-spacing:.08em">
                Piece Detector Heatmap (live from /perception/piece_debug)
              </div>
              <img id="diff-heatmap" src="/api/diff_frame" alt="diff heatmap">
              <div style="margin-top:8px;font-size:.63em;color:var(--dim)">
                Green outlines = squares flagged by the actual detector using its own threshold.
                Use <b>Apply to Detector</b> to push your tuned values to the node.
              </div>
            </div>
            <!-- Right: interactive score canvas + controls -->
            <div>
              <div style="font-size:.63em;color:var(--dim);margin-bottom:6px;text-transform:uppercase;letter-spacing:.08em">
                Interactive Score Grid (preview using sliders below)
              </div>
              <canvas id="score-canvas" width="360" height="360"></canvas>
              <div class="flagged-row">
                <span class="lbl">Would flag: </span>
                <span id="flagged-squares">–</span>
              </div>
              <div class="median-row">
                Compensation floor: <span id="median-floor">–</span>
              </div>

              <div style="margin-top:10px;background:var(--surf2);border:1px solid var(--border);border-radius:5px;padding:10px 12px">
                <div style="font-size:.63em;text-transform:uppercase;letter-spacing:.08em;color:var(--accent);margin-bottom:8px;font-weight:700;border-bottom:1px solid var(--border);padding-bottom:5px">
                  Detection Thresholds
                  <small style="font-weight:400;text-transform:none;letter-spacing:0;color:var(--dim)">— preview only</small>
                </div>
                <div class="slider-row">
                  <label title="Adjusted score above this = square flagged as changed. Default: 18">Diff threshold</label>
                  <input type="range" id="s_display_threshold" min="5" max="80" step="0.5" value="18"
                    oninput="onDiffSlider('display_threshold',+this.value);document.getElementById('vb_display_threshold').textContent=(+this.value).toFixed(1)">
                  <span class="vb" id="vb_display_threshold">18.0</span>
                </div>
                <div class="slider-row">
                  <label title="0=none, 1=full median subtraction, >1=aggressive. Default: 1.0">Shift compensation</label>
                  <input type="range" id="s_shift_compensation" min="0" max="2.5" step="0.05" value="1.0"
                    oninput="onDiffSlider('shift_compensation',+this.value);document.getElementById('vb_shift_compensation').textContent=(+this.value).toFixed(2)">
                  <span class="vb" id="vb_shift_compensation">1.00</span>
                </div>
                <div class="tog-row">
                  <input type="checkbox" id="t_clump_enable" onchange="onDiffTog('clump_enable',this.checked)">
                  <label for="t_clump_enable" title="Group adjacent flagged squares, keep only hottest per cluster. NOTE: set Keep ≥ 4 for castling.">Clump mode (reduce perspective bleed)</label>
                </div>
                <div class="slider-row">
                  <label title="Squares to keep per clump. 1=tight, 2-4 for castling.">Keep per clump</label>
                  <input type="range" id="s_clump_keep" min="1" max="4" step="1" value="1"
                    oninput="onDiffSlider('clump_keep_per_group',+this.value);document.getElementById('vb_clump_keep').textContent=this.value">
                  <span class="vb" id="vb_clump_keep">1</span>
                </div>
                <div class="btn-row" style="margin-top:8px">
                  <button class="btn primary" onclick="applyToDetector()">&#9881; Apply to Actual Detector</button>
                </div>
                <div style="font-size:.62em;color:var(--dim);margin-top:6px;line-height:1.5">
                  Pushes <code>diff_threshold</code>, <code>global_shift_compensation</code>,
                  and <code>clump_enable</code> to <code>piece_detector_node</code> via ROS set_parameters.
                </div>
              </div>
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

          <div class="card">
            <div class="card-hdr">Magnet (Z-axis Servo)</div>
            <div class="card-body">
              <div class="btn-row" style="justify-content:center">
                <button class="btn success"
                  onclick="hwPost('/api/hw/servo/engage','magnet-gantry-msg')">
                  &#11015; Engage</button>
                <button class="btn warn"
                  onclick="hwPost('/api/hw/servo/release','magnet-gantry-msg')">
                  &#11014; Release</button>
              </div>
              <div id="magnet-gantry-msg"
                style="text-align:center;font-size:.75em;color:var(--dim);margin-top:6px">&#8212;</div>
            </div>
          </div>
        </div>

        <div>
          <div class="card">
            <div class="card-hdr">Chess Square Navigation</div>
            <div class="card-body">
              <div class="sq-row">
                <input id="sq-input" type="text" maxlength="2" placeholder="e4">
                <button class="btn primary" onclick="gotoSquare()">&#8594; Go</button>
              </div>
              <div id="sq-nav-msg"
                style="font-size:.7em;color:var(--dim);margin-top:6px">
                Uses calibrated coordinates when applied, otherwise defaults.
              </div>
            </div>
          </div>

          <!-- ── Board Calibration ─────────────────────────────────────── -->
          <div class="card">
            <div class="card-hdr">Board Calibration</div>
            <div class="card-body">
              <div style="font-size:.68em;color:var(--dim);margin-bottom:8px;line-height:1.8">
                <b>Workflow:</b>
                Engage magnet &rarr; place piece &rarr;
                jog to <b>a1</b> center &rarr; <kbd>Save a1</kbd> &rarr;
                jog to <b>h8</b> center &rarr; <kbd>Save h8</kbd> &rarr; <kbd>Apply</kbd>
              </div>
              <div class="btn-row">
                <button class="btn primary" onclick="calibSaveA1()"
                  title="Save current gantry position as a1 (bottom-left corner)">
                  &#128204; Save a1</button>
                <button class="btn primary" onclick="calibSaveH8()"
                  title="Save current gantry position as h8 (top-right corner)">
                  &#128204; Save h8</button>
                <button class="btn success" onclick="calibApply()"
                  title="Compute square size and save board_calibration.json">
                  &#10003; Apply</button>
                <button class="btn danger" onclick="calibClear()"
                  title="Clear calibration and delete file">&#10005;</button>
              </div>
              <div id="calib-status"
                style="font-size:.72em;margin-top:8px;color:var(--dim);min-height:2.2em;
                       line-height:1.5;padding:4px 6px;background:#0a0a18;border-radius:4px">
                No calibration loaded
              </div>
              <div style="margin-top:8px;font-size:.7em;color:var(--dim);margin-bottom:4px">
                Verify — move to square using calibrated coords:
              </div>
              <div class="sq-row" style="gap:4px">
                <input id="calib-verify-sq" type="text" maxlength="2" placeholder="e4"
                  style="width:52px">
                <button class="btn" onclick="calibGoto()"
                  title="Go to typed square using calibrated coordinates">&#8594; Goto</button>
                <select id="calib-sq-sel" onchange="calibGotoSel(this)"
                  style="font-size:.8em;flex:1;min-width:0">
                  <option value="">Quick&hellip;</option>
                </select>
              </div>
              <div style="margin-top:6px">
                <button class="btn" style="width:100%;font-size:.72em"
                  onclick="quickRun('gantry','calibrate')"
                  title="Run CLI calibration verification test">
                  &#9654; Run Calibration Verify Test</button>
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

      <!-- Node Health panel -->
      <div class="card" style="margin-bottom:8px">
        <div class="card-hdr">
          ROS Node Health
          <span style="font-size:.85em;font-weight:400;text-transform:none;letter-spacing:0;color:var(--dim);margin-left:8px" id="node-health-ts">polling…</span>
        </div>
        <div class="card-body">
          <div class="node-grid" id="node-health-grid">
            <!-- populated by JS -->
            <div style="color:var(--dim);font-size:.72em">Loading…</div>
          </div>
        </div>
      </div>

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

// Board calibration state (loaded from /api/gantry/calibration on init)
var calibState = {applied:false, a1_x:null, a1_y:null,
                  h8_x:null, h8_y:null, sq_x:null, sq_y:null};

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
function capturePreMove() { doCaptureReference(); }

// ── Capture Reference ─────────────────────────────────────────────
function doCaptureReference() {
  var btn = document.getElementById('capture-ref-btn');
  if (btn) btn.disabled = true;
  fetch('/api/perception/capture_reference', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      var banner = document.getElementById('ref-banner');
      var txt    = document.getElementById('ref-status-text');
      var inl    = document.getElementById('ref-status-inline');
      if (d.ok) {
        if (banner) { banner.className='ref-banner done'; }
        if (txt)    { txt.textContent='Reference captured ✓ — board is ready for move detection.'; }
        if (inl)    { inl.textContent='Captured ✓'; inl.style.color='var(--ok)'; }
      } else {
        if (txt) { txt.textContent='Failed: '+(d.msg||d.message||'unknown error'); }
        if (inl) { inl.textContent='Failed'; inl.style.color='var(--err)'; }
      }
      if (btn) btn.disabled = false;
    }).catch(function(e) {
      var txt = document.getElementById('ref-status-text');
      if (txt) txt.textContent = 'Service unavailable';
      if (btn) btn.disabled = false;
    });
}

// ── Game control ──────────────────────────────────────────────────
function gameAct(action) {
  var url = {start:'/api/game/start', new:'/api/game/new', resign:'/api/game/resign'}[action];
  if (!url) return;
  fetch(url, {method:'POST'}).then(function(r){return r.json();})
    .then(function(d){
      if (!d.ok) { alert('Game action failed: '+(d.msg||d.message||'unknown')); }
    }).catch(function(){ alert('ROS not connected'); });
}

function confirmGameAct(action, msg) {
  if (window.confirm(msg)) gameAct(action);
}

var _settingsTimer = null;
function sendGameSettings() {
  clearTimeout(_settingsTimer);
  _settingsTimer = setTimeout(function() {
    var think = parseFloat(document.getElementById('think-slider').value);
    var clock = parseFloat(document.getElementById('clock-time-input').value);
    fetch('/api/game/settings', {method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({think_time_s: think, time_per_player_min: clock})
    });
  }, 500);
}

// ── Diff score canvas ─────────────────────────────────────────────
var _diffParams = {display_threshold:18.0, shift_compensation:1.0, clump_enable:0, clump_keep_per_group:1};
var _squareScores = {};

function onDiffSlider(key, val) {
  _diffParams[key] = val;
  renderScoreCanvas();
}
function onDiffTog(key, val) {
  _diffParams[key] = val ? 1 : 0;
  renderScoreCanvas();
}

function renderScoreCanvas() {
  var canvas = document.getElementById('score-canvas');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var w = canvas.width, h = canvas.height;
  var sq = w / 8;
  ctx.clearRect(0, 0, w, h);
  var scores = _squareScores;
  var vals = Object.values(scores);
  var median = 0;
  if (vals.length) {
    var sorted = vals.slice().sort(function(a,b){return a-b;});
    median = sorted[Math.floor(sorted.length/2)];
  }
  var comp   = _diffParams.shift_compensation;
  var thresh = _diffParams.display_threshold;
  var floor  = median * comp;
  var flagged = [];
  var files = 'abcdefgh';
  for (var r = 0; r < 8; r++) {
    for (var f = 0; f < 8; f++) {
      var sq_name = files[f] + (8 - r);
      var raw = scores[sq_name] || 0;
      var adj = Math.max(0, raw - floor);
      var isLight = (r + f) % 2 === 0;
      // base tile
      ctx.fillStyle = isLight ? '#2a2a3a' : '#1a1a28';
      ctx.fillRect(f * sq, r * sq, sq, sq);
      // heat overlay
      if (raw > 0) {
        var intensity = Math.min(1.0, adj / (thresh * 1.5));
        ctx.fillStyle = 'rgba(255,' + Math.round(60 + 180 * (1-intensity)) + ',40,' + (0.2 + 0.75 * intensity) + ')';
        ctx.fillRect(f * sq, r * sq, sq, sq);
      }
      // flag outline
      if (adj >= thresh) {
        ctx.strokeStyle = 'rgba(60,255,120,0.9)';
        ctx.lineWidth = 2;
        ctx.strokeRect(f * sq + 1, r * sq + 1, sq - 2, sq - 2);
        flagged.push(sq_name);
      }
      // score text
      ctx.fillStyle = adj >= thresh ? '#ffffff' : '#888';
      ctx.font = Math.round(sq * 0.28) + 'px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(adj.toFixed(0), f * sq + sq / 2, r * sq + sq * 0.65);
      // square name
      ctx.fillStyle = '#445';
      ctx.font = Math.round(sq * 0.22) + 'px monospace';
      ctx.fillText(sq_name, f * sq + sq / 2, r * sq + sq - 3);
    }
  }
  var fEl = document.getElementById('flagged-squares');
  if (fEl) fEl.textContent = flagged.length ? flagged.join(' ') : '—';
  var mEl = document.getElementById('median-floor');
  if (mEl) mEl.textContent = floor.toFixed(1);
}

function applyToDetector() {
  fetch('/api/detector_params', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({
      diff_threshold:     _diffParams.display_threshold,
      shift_compensation: _diffParams.shift_compensation,
      clump_enable:       _diffParams.clump_enable,
      clump_keep_per_group: _diffParams.clump_keep_per_group
    })
  }).then(function(r){return r.json();})
    .then(function(d){ if (!d.ok) alert('Apply failed: '+(d.msg||'')); });
}

// Refresh diff images and score data when on perception tab
setInterval(function() {
  if (activeTab !== 'perception') return;
  fetch('/api/square_scores').then(function(r){return r.json();}).then(function(d){
    _squareScores = d.scores || {};
    renderScoreCanvas();
    // Sync sliders from server if not being dragged
    var elems = {s_display_threshold:d.display_threshold, s_shift_compensation:d.shift_compensation,
                 s_clump_keep:d.clump_keep_per_group};
    // (don't override while user is interacting — just update internal state)
    if (d.display_threshold !== undefined) _diffParams.display_threshold = d.display_threshold;
    if (d.shift_compensation !== undefined) _diffParams.shift_compensation = d.shift_compensation;
    if (d.clump_enable !== undefined) _diffParams.clump_enable = d.clump_enable;
    if (d.clump_keep_per_group !== undefined) _diffParams.clump_keep_per_group = d.clump_keep_per_group;
  });
  // Refresh diff heatmap image
  var img = document.getElementById('diff-heatmap');
  if (img) img.src = '/api/diff_frame?t=' + Date.now();
}, 1500);

// ── Node health polling ───────────────────────────────────────────
var _nodeHealthInterval = null;
function startNodeHealthPoll() {
  if (_nodeHealthInterval) return;
  _nodeHealthInterval = setInterval(function() {
    if (activeTab !== 'hardware') return;
    fetch('/api/nodes/health').then(function(r){return r.json();}).then(function(d){
      var grid = document.getElementById('node-health-grid');
      var ts   = document.getElementById('node-health-ts');
      if (!grid) return;
      if (d.error) { grid.innerHTML='<span style="color:var(--err)">'+d.error+'</span>'; return; }
      var nodes = Object.keys(d);
      var html  = '';
      nodes.forEach(function(n) {
        var ok  = d[n];
        var cls = ok ? 'ok' : 'err';
        html += '<div class="node-row"><div class="node-dot '+cls+'"></div>'+n+'</div>';
      });
      grid.innerHTML = html;
      if (ts) ts.textContent = 'updated '+new Date().toLocaleTimeString();
    }).catch(function(){
      var grid = document.getElementById('node-health-grid');
      if (grid) grid.innerHTML='<span style="color:var(--dim)">ROS offline</span>';
    });
  }, 3000);
}
startNodeHealthPoll();

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
  if (sq) _gotoSquareMM(sq);
}

// Shared: send goto to server (updates canvas + triggers physical move if action up)
function _gotoSquareMM(sq) {
  fetch('/api/gantry/goto', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({square: sq})
  }).then(function(r){return r.json();}).then(function(d){
    if (d.ok) {
      setGantryPos(d.x, d.y);
      var msg = document.getElementById('sq-nav-msg');
      if (msg) msg.textContent = sq.toUpperCase()+' → ('+d.x.toFixed(1)+', '+d.y.toFixed(1)+') mm'
        + (d.moved ? ' ✓ moving' : ' (display only — start action server to move)');
    } else {
      alert('Invalid square: '+sq);
    }
  });
}

// ── Board calibration ──────────────────────────────────────────────────────
function loadCalibState() {
  fetch('/api/gantry/calibration').then(function(r){return r.json();}).then(function(d){
    calibState = d;
    updateCalibUI();
    drawCanvas();
  });
}

function updateCalibUI() {
  var el = document.getElementById('calib-status');
  if (!el) return;
  if (calibState.applied && calibState.sq_x !== null) {
    el.style.color = 'var(--ok)';
    el.textContent = '✓ Calibrated'
      + ' | sq_x=' + calibState.sq_x.toFixed(1) + 'mm'
      + ' sq_y=' + calibState.sq_y.toFixed(1) + 'mm'
      + ' | a1=(' + calibState.a1_x.toFixed(0) + ',' + calibState.a1_y.toFixed(0) + ')'
      + ' h8=(' + calibState.h8_x.toFixed(0) + ',' + calibState.h8_y.toFixed(0) + ')';
  } else if (calibState.a1_x !== null || calibState.h8_x !== null) {
    el.style.color = 'var(--warn)';
    var parts = [];
    parts.push(calibState.a1_x !== null ? 'a1 ✓ ('+calibState.a1_x.toFixed(0)+','+calibState.a1_y.toFixed(0)+')' : 'a1 missing');
    parts.push(calibState.h8_x !== null ? 'h8 ✓ ('+calibState.h8_x.toFixed(0)+','+calibState.h8_y.toFixed(0)+')' : 'h8 missing');
    el.textContent = parts.join('  ·  ') + '  →  click Apply when both saved';
  } else {
    el.style.color = 'var(--dim)';
    el.textContent = 'No calibration. Jog to a1 center then click Save a1.';
  }
}

function calibSaveA1() {
  fetch('/api/gantry/calibration/save_a1', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if (d.ok) {
        calibState.a1_x = d.x; calibState.a1_y = d.y;
        calibState.applied = false;
        updateCalibUI();
        drawCanvas();
      } else { alert('Save a1 failed: '+(d.msg||'')); }
    });
}

function calibSaveH8() {
  fetch('/api/gantry/calibration/save_h8', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if (d.ok) {
        calibState.h8_x = d.x; calibState.h8_y = d.y;
        calibState.applied = false;
        updateCalibUI();
        drawCanvas();
      } else { alert('Save h8 failed: '+(d.msg||'')); }
    });
}

function calibApply() {
  fetch('/api/gantry/calibration/apply', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(d){
      if (d.ok) {
        calibState = {applied:true, a1_x:d.a1_x, a1_y:d.a1_y,
                      h8_x:d.h8_x, h8_y:d.h8_y, sq_x:d.sq_x, sq_y:d.sq_y};
        updateCalibUI();
        drawCanvas();
        if (d.warn) console.warn(d.warn);
      } else { alert('Calibration failed: '+(d.msg||'')); }
    });
}

function calibClear() {
  if (!confirm('Clear board calibration and delete board_calibration.json?')) return;
  fetch('/api/gantry/calibration/clear', {method:'POST'})
    .then(function(r){return r.json();})
    .then(function(){
      calibState = {applied:false,a1_x:null,a1_y:null,h8_x:null,h8_y:null,sq_x:null,sq_y:null};
      updateCalibUI();
      drawCanvas();
    });
}

function calibGoto() {
  var sq = document.getElementById('calib-verify-sq').value.trim().toLowerCase();
  if (sq) _gotoSquareMM(sq);
}

function calibGotoSel(sel) {
  var sq = sel.value;
  if (!sq) return;
  document.getElementById('calib-verify-sq').value = sq;
  sel.value = '';
  _gotoSquareMM(sq);
}

function buildCalibVerifySelect() {
  var sel = document.getElementById('calib-sq-sel');
  if (!sel) return;
  var squares = ['a1','h1','a8','h8','d4','e4','d5','e5','a4','h4','e1','e8'];
  squares.forEach(function(sq) {
    var o = document.createElement('option');
    o.value = sq; o.textContent = sq.toUpperCase();
    sel.appendChild(o);
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
  var sc = (W - 2*pad) / X_MAX;

  // Use calibrated board geometry when available
  var bx  = (calibState.applied && calibState.sq_x) ? calibState.a1_x : BO_X;
  var by  = (calibState.applied && calibState.sq_y) ? calibState.a1_y : BO_Y;
  var sqx = (calibState.applied && calibState.sq_x) ? calibState.sq_x : SQ;
  var sqy = (calibState.applied && calibState.sq_y) ? calibState.sq_y : SQ;

  ctx.clearRect(0, 0, W, H);
  ctx.fillStyle = '#020206';
  ctx.fillRect(0, 0, W, H);

  // Workspace border
  ctx.strokeStyle = '#1a1a2e';
  ctx.lineWidth = 1;
  ctx.strokeRect(pad, pad, W-2*pad, H-2*pad);

  // X limit (right side = X home)
  ctx.fillStyle = '#1e0a0a';
  ctx.fillRect(W-pad-2, pad, 2, H-2*pad);
  ctx.fillStyle = '#ff4060';
  ctx.font = '7px monospace';
  ctx.fillText('X HOME', W-pad+3, pad+12);

  // Y limit (bottom = Y home)
  ctx.fillStyle = '#0a1e0a';
  ctx.fillRect(pad, H-pad, W-2*pad, 2);
  ctx.fillStyle = '#3fca6e';
  ctx.font = '7px monospace';
  ctx.fillText('Y HOME', pad+2, H-pad+10);

  // Chess square grid
  for (var r = 0; r < 8; r++) {
    for (var c = 0; c < 8; c++) {
      var px = pad + (bx + c*sqx)*sc;
      var py = H - pad - (by + (r+1)*sqy)*sc;
      ctx.fillStyle = (r+c)%2===0 ? '#1a1a3e' : '#0d0d1e';
      ctx.fillRect(px, py, sqx*sc, sqy*sc);
    }
  }

  // Board border
  ctx.strokeStyle = calibState.applied ? '#5a5aaa' : '#3a3a7a';
  ctx.lineWidth = 1.5;
  ctx.strokeRect(pad + bx*sc, H - pad - (by + 8*sqy)*sc, 8*sqx*sc, 8*sqy*sc);

  // Corner labels
  ctx.fillStyle = '#40407a';
  ctx.font = '7px monospace';
  ctx.fillText('a1', pad+bx*sc+2, H-pad-by*sc-2);
  ctx.fillText('h8', pad+(bx+7*sqx)*sc+2, H-pad-(by+7*sqy)*sc-12);

  // Axis labels
  ctx.fillStyle = '#30305a';
  ctx.font = '7px monospace';
  ctx.fillText('0', pad-6, H-pad+10);
  ctx.fillText(X_MAX+'mm', W-pad-16, H-pad+10);

  // Origin marker
  ctx.fillStyle = '#3a3a6a';
  ctx.fillRect(pad-1, H-pad-1, 3, 3);

  // Calibration save points (orange=a1, blue=h8, shown even before Apply)
  if (calibState.a1_x !== null) {
    var ax = pad + calibState.a1_x * sc;
    var ay = H - pad - calibState.a1_y * sc;
    ctx.fillStyle = '#ff8030';
    ctx.beginPath(); ctx.arc(ax, ay, 5, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = '#ff8030'; ctx.font = '8px monospace';
    ctx.fillText('a1', ax+6, ay+3);
  }
  if (calibState.h8_x !== null) {
    var hx = pad + calibState.h8_x * sc;
    var hy = H - pad - calibState.h8_y * sc;
    ctx.fillStyle = '#4090ff';
    ctx.beginPath(); ctx.arc(hx, hy, 5, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = '#4090ff'; ctx.font = '8px monospace';
    ctx.fillText('h8', hx+6, hy+3);
  }

  // Calibration badge
  if (calibState.applied) {
    ctx.fillStyle = '#2a4a2a';
    ctx.fillRect(pad, pad, 54, 12);
    ctx.fillStyle = '#5fca7f';
    ctx.font = '7px monospace';
    ctx.fillText('CALIBRATED', pad+2, pad+9);
  }

  // Current position dot + crosshair
  if (gx === undefined) {
    gx = curStatus.gantry_x || 0;
    gy = curStatus.gantry_y || 0;
  }
  var cx = Math.max(pad, Math.min(W-pad, pad + gx*sc));
  var cy = Math.max(pad, Math.min(H-pad, H - pad - gy*sc));

  ctx.strokeStyle = '#00d4aa';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx-10,cy); ctx.lineTo(cx+10,cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx,cy-10); ctx.lineTo(cx,cy+10); ctx.stroke();
  ctx.fillStyle = '#00d4aa';
  ctx.beginPath(); ctx.arc(cx, cy, 4, 0, 2*Math.PI); ctx.fill();

  // Position readout
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
      if (label==='clock') document.getElementById('hw-clock-msg').textContent = msg;
      if (label==='home')  document.getElementById('hw-home-msg').textContent = msg;
      // Generic: if label is an element ID, update its text
      var el = document.getElementById(label);
      if (el) el.textContent = msg;
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

    // Calibration state sync from status
    if (d.calib_applied !== undefined && d.calib_applied !== calibState.applied) {
      calibState = {applied: d.calib_applied,
                    a1_x: d.calib_a1_x, a1_y: d.calib_a1_y,
                    h8_x: d.calib_h8_x, h8_y: d.calib_h8_y,
                    sq_x: d.calib_sq_x, sq_y: d.calib_sq_y};
      updateCalibUI();
    }

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

    // ── Pre-game checklist ────────────────────────────────────────
    var chkFn = function(id, ok) {
      var el = document.getElementById(id);
      if (!el) return;
      el.textContent = ok ? '✓' : '✗';
      el.className = 'check-icon ' + (ok ? 'ok' : 'no');
    };
    var rosOk   = !!d.ros_connected;
    var homedOk = !!d.gantry_homed;
    var calibOk = !!d.calib_applied;
    var refOk   = !!d.session_reference_captured;
    chkFn('chk-ros',   rosOk);
    chkFn('chk-homed', homedOk);
    chkFn('chk-calib', calibOk);
    chkFn('chk-ref',   refOk);
    var startBtn  = document.getElementById('start-game-btn');
    var startHint = document.getElementById('start-game-hint');
    var canStart  = rosOk && homedOk && refOk;
    if (startBtn) startBtn.disabled = !canStart;
    if (startHint) {
      if (!rosOk)   startHint.textContent = 'ROS not connected';
      else if (!homedOk) startHint.textContent = 'Home the gantry first';
      else if (!refOk)   startHint.textContent = 'Capture reference first (Perception tab)';
      else               startHint.textContent = '';
    }

    // ── Move history (UCI from game_manager) ──────────────────────
    var hist = d.move_history_uci || d.move_history || [];
    if (hist.length) {
      var mh = '';
      hist.forEach(function(m,i) {
        if (i%2===0) mh += '<span class="mn">'+(Math.floor(i/2)+1)+'. </span>';
        mh += '<span class="mv">'+m+' </span>';
      });
      var mhEl = document.getElementById('move-hist');
      if (mhEl) { mhEl.innerHTML = mh; mhEl.scrollTop = mhEl.scrollHeight; }
    }

    // ── Promotion banner ──────────────────────────────────────────
    var promoBanner  = document.getElementById('promo-banner');
    if (promoBanner) {
      promoBanner.className = 'promo-banner' + (gs === 'PROMOTION_WAIT' ? ' visible' : '');
    }

    // ── Game-over banner ──────────────────────────────────────────
    var goBanner = document.getElementById('gameover-banner');
    var goResult = document.getElementById('gameover-result');
    if (goBanner) {
      var showGo = (gs === 'GAME_OVER' && d.game_result);
      goBanner.className = 'gameover-banner' + (showGo ? ' visible' : '');
      if (goResult && d.game_result) goResult.textContent = d.game_result;
    }

    // ── Reference status (Perception tab) ────────────────────────
    var refBanner = document.getElementById('ref-banner');
    var refTxt    = document.getElementById('ref-status-text');
    var refInl    = document.getElementById('ref-status-inline');
    if (refBanner) {
      refBanner.className = 'ref-banner ' + (refOk ? 'done' : 'needs');
    }
    if (refTxt && d.ref_status) refTxt.textContent = d.ref_status;
    if (refInl) {
      refInl.textContent = d.ref_status || '–';
      refInl.style.color = refOk ? 'var(--ok)' : 'var(--warn)';
    }

    // ── Settings: populate on first load ─────────────────────────
    var ts = document.getElementById('think-slider');
    var tv = document.getElementById('think-vb');
    if (ts && !ts._synced && d.think_time_s) {
      ts.value = d.think_time_s;
      if (tv) tv.textContent = parseFloat(d.think_time_s).toFixed(1);
      ts._synced = true;
    }
    var ct = document.getElementById('clock-time-input');
    if (ct && !ct._synced && d.white_time) {
      ct.value = Math.round(d.white_time / 60);
      ct._synced = true;
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
loadCalibState();
buildCalibVerifySelect();
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
    _load_calibration()

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

    import socket as _socket
    def _get_local_ip():
        try:
            s = _socket.socket(_socket.AF_INET, _socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return "localhost"
    pi_ip = _get_local_ip()
    print(f"\n  ♞  ChessOS ready")
    print(f"     http://{pi_ip}:{args.port}")
    print(f"     http://localhost:{args.port}")
    print(f"     Mode:   {args.mode}")
    print(f"     ROS2:   {'enabled' if not args.no_ros else 'disabled'}")
    print(f"     Tests:  {_TEST_SCRIPT}\n")

    app.run(host=args.host, port=args.port,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
