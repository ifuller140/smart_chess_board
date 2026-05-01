#!/usr/bin/env python3
"""
FEN Live Board Visualizer — Enhanced Debug Dashboard
=====================================================
Serves at http://<pi-ip>:5000

Two camera views:
  /api/frame        raw camera + piece/grid overlay
  /api/debug_frame  edge/line/corner/color-mask debug view

All detection parameters are tunable live via the browser.

Usage:
  # With ROS running:
  python3 src/chess_perception/scripts/fen_visualizer.py

  # Offline test (no ROS):
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros
"""

import argparse, threading, time, sys, io, base64
import numpy as np
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import Image
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from flask import Flask, jsonify, request as freq, Response, render_template_string
except ImportError:
    print("ERROR: pip3 install flask"); sys.exit(1)

try:
    import chess
    CHESS_OK = True
except ImportError:
    CHESS_OK = False

# ─── Shared state ─────────────────────────────────────────────────────────────
_lock = threading.Lock()
_state = {
    "fen":           "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "frame_count":   0,
    "last_updated":  0.0,
    "error":         "",
    "raw_frame":     None,   # latest BGR ndarray from camera
    "corners":       None,   # 4x2 float32 or None
}

# All tunable detection parameters with defaults
_params = {
    # Preprocessing
    "blur_ksize":        5,     # Gaussian blur kernel (odd number)
    # Canny edge detection
    "canny_low":        50,
    "canny_high":      150,
    # Hough line detection
    "hough_thresh":     70,    # Accumulator threshold
    "hough_min_len":    40,    # Min line length (probabilistic)
    "hough_max_gap":    10,    # Max gap between segments
    # Contour fallback
    "contour_min_area": 50000,
    # Color mask for board squares (HSV)
    "sq_light_h_lo":    15,  "sq_light_h_hi": 35,
    "sq_light_s_lo":    20,  "sq_light_s_hi": 120,
    "sq_light_v_lo":   100,  "sq_light_v_hi": 230,
    "sq_dark_h_lo":     15,  "sq_dark_h_hi":  35,
    "sq_dark_s_lo":     20,  "sq_dark_s_hi":  110,
    "sq_dark_v_lo":     40,  "sq_dark_v_hi":  130,
    # Piece detection
    "piece_std_thresh":     30,   # Std-dev threshold to call square occupied
    "piece_white_bright":  150,   # Min mean brightness → white piece
    "piece_black_bright":   80,   # Max mean brightness → black piece
    # Display toggles (1=on, 0=off)
    "show_edges":    1,
    "show_lines":    1,
    "show_corners":  1,
    "show_grid":     1,
    "show_pieces":   1,
    "show_color_mask": 0,
    # Warp output size
    "warp_size":     480,
}


# ─── Detection pipeline ────────────────────────────────────────────────────────
def _detect_board(gray, blur, edges, p):
    """
    Returns corners (4x2 float32, ordered TL/TR/BR/BL) or None.
    Tries Hough lines first, falls back to largest quadrilateral contour.
    """
    corners = _detect_hough(gray, blur, edges, p)
    if corners is None:
        corners = _detect_contour(edges, p)
    return corners


def _detect_hough(gray, blur, edges, p):
    lines = cv2.HoughLines(edges, 1, np.pi / 180, p["hough_thresh"])
    if lines is None or len(lines) < 4:
        return None
    horiz, vert = [], []
    for l in lines:
        rho, theta = l[0]
        if theta < np.pi / 6 or theta > 5 * np.pi / 6:
            vert.append((rho, theta))
        elif np.pi / 3 < theta < 2 * np.pi / 3:
            horiz.append((rho, theta))
    if len(horiz) < 2 or len(vert) < 2:
        return None
    horiz.sort(key=lambda x: x[0]); vert.sort(key=lambda x: x[0])
    h_pair = [horiz[0], horiz[-1]]
    v_pair = [vert[0],  vert[-1]]
    pts = []
    for h in h_pair:
        for v in v_pair:
            pt = _intersect(h, v)
            if pt is not None:
                pts.append(pt)
    if len(pts) != 4:
        return None
    return _order_corners(np.array(pts, dtype=np.float32))


def _detect_contour(edges, p):
    kernel = np.ones((3, 3), np.uint8)
    dilated = cv2.dilate(edges, kernel, iterations=2)
    cnts, _ = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < p["contour_min_area"]:
            continue
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        if len(approx) == 4 and area > best_area:
            best_area = area; best = approx
    if best is None:
        return None
    return _order_corners(best.reshape(4, 2).astype(np.float32))


def _intersect(l1, l2):
    r1, t1 = l1; r2, t2 = l2
    A = np.array([[np.cos(t1), np.sin(t1)], [np.cos(t2), np.sin(t2)]])
    b = np.array([r1, r2])
    try:
        x = np.linalg.solve(A, b)
        return tuple(x)
    except np.linalg.LinAlgError:
        return None


def _order_corners(pts):
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # TL
    rect[2] = pts[np.argmax(s)]   # BR
    d = np.diff(pts, axis=1).ravel()
    rect[1] = pts[np.argmin(d)]   # TR
    rect[3] = pts[np.argmax(d)]   # BL
    return rect


def _analyze_squares(warped, p):
    """Returns list of 64 ('W'|'B'|'') strings."""
    size = warped.shape[0]
    sq = size // 8
    margin = sq // 6
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    board_mean = float(np.mean(gray))
    results = []
    for row in range(8):
        for col in range(8):
            y1, y2 = row * sq + margin, (row + 1) * sq - margin
            x1, x2 = col * sq + margin, (col + 1) * sq - margin
            roi = gray[y1:y2, x1:x2]
            std = float(np.std(roi))
            mean = float(np.mean(roi))
            if std > p["piece_std_thresh"] or abs(mean - board_mean) > 40:
                results.append('W' if mean > p["piece_white_bright"] else 'B')
            else:
                results.append('')
    return results


def _build_fen(pieces64):
    """Build piece-placement string from 64-element color list."""
    rows = []
    for rank_i in range(7, -1, -1):
        empty, row = 0, ''
        for file_i in range(8):
            idx = (7 - rank_i) * 8 + file_i
            p = pieces64[idx]
            if not p:
                empty += 1
            else:
                if empty: row += str(empty); empty = 0
                row += 'P' if p == 'W' else 'p'
        if empty: row += str(empty)
        rows.append(row)
    return '/'.join(rows) + ' w KQkq - 0 1'


def _make_color_mask(bgr, p):
    """Return binary mask combining light-square and dark-square HSV ranges."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    light = cv2.inRange(hsv,
        np.array([p["sq_light_h_lo"], p["sq_light_s_lo"], p["sq_light_v_lo"]]),
        np.array([p["sq_light_h_hi"], p["sq_light_s_hi"], p["sq_light_v_hi"]]))
    dark = cv2.inRange(hsv,
        np.array([p["sq_dark_h_lo"],  p["sq_dark_s_lo"],  p["sq_dark_v_lo"]]),
        np.array([p["sq_dark_h_hi"],  p["sq_dark_s_hi"],  p["sq_dark_v_hi"]]))
    return cv2.bitwise_or(light, dark)


# ─── Frame rendering ───────────────────────────────────────────────────────────
def _render_main(frame, p):
    """Raw frame + detected grid overlay + piece labels."""
    out = frame.copy()
    with _lock:
        corners = _state["corners"]

    if corners is not None and p["show_corners"]:
        for i, pt in enumerate(corners):
            cv2.circle(out, tuple(pt.astype(int)), 10,
                       [(255,0,0),(0,255,0),(0,0,255),(255,255,0)][i], -1)
        cv2.polylines(out, [corners.astype(int).reshape(-1,1,2)], True, (0,255,0), 3)

    if corners is not None and p["show_grid"]:
        sz = p["warp_size"]
        dst = np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]])
        M = cv2.getPerspectiveTransform(corners, dst)
        M_inv = cv2.getPerspectiveTransform(dst, corners)
        sq = sz // 8
        for i in range(1, 8):
            # vertical
            p0 = np.float32([[[i*sq, 0]]])
            p1 = np.float32([[[i*sq, sz]]])
            pt0 = cv2.perspectiveTransform(p0, M_inv)[0,0].astype(int)
            pt1 = cv2.perspectiveTransform(p1, M_inv)[0,0].astype(int)
            cv2.line(out, tuple(pt0), tuple(pt1), (0,200,200), 1)
            # horizontal
            q0 = np.float32([[[0, i*sq]]])
            q1 = np.float32([[[sz, i*sq]]])
            qt0 = cv2.perspectiveTransform(q0, M_inv)[0,0].astype(int)
            qt1 = cv2.perspectiveTransform(q1, M_inv)[0,0].astype(int)
            cv2.line(out, tuple(qt0), tuple(qt1), (0,200,200), 1)

    # Piece labels in warped space mapped back
    if corners is not None and p["show_pieces"]:
        sz = p["warp_size"]
        dst = np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]])
        M_inv = cv2.getPerspectiveTransform(
            np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]]), corners)
        warped = cv2.warpPerspective(frame, cv2.getPerspectiveTransform(corners, dst), (sz, sz))
        pieces = _analyze_squares(warped, p)
        sq = sz // 8
        for row in range(8):
            for col in range(8):
                idx = row * 8 + col
                c = pieces[idx]
                if c:
                    cx = col * sq + sq // 2
                    cy = row * sq + sq // 2
                    pt = cv2.perspectiveTransform(np.float32([[[cx, cy]]]), M_inv)[0,0].astype(int)
                    color = (255,255,255) if c == 'W' else (0,0,0)
                    cv2.putText(out, c, tuple(pt), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    _add_status(out, corners is not None)
    return out


def _render_debug(frame, p):
    """Debug view: edges, Hough lines, corners, color mask."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    k = p["blur_ksize"] | 1  # ensure odd
    blur = cv2.GaussianBlur(gray, (k, k), 0)
    edges = cv2.Canny(blur, p["canny_low"], p["canny_high"])

    if p["show_color_mask"]:
        mask = _make_color_mask(frame, p)
        out = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    elif p["show_edges"]:
        out = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
    else:
        out = frame.copy()

    # Hough lines
    if p["show_lines"]:
        lines = cv2.HoughLines(edges, 1, np.pi / 180, p["hough_thresh"])
        if lines is not None:
            for l in lines[:20]:
                rho, theta = l[0]
                a, b = np.cos(theta), np.sin(theta)
                x0, y0 = a * rho, b * rho
                pt1 = (int(x0 + 2000 * (-b)), int(y0 + 2000 * a))
                pt2 = (int(x0 - 2000 * (-b)), int(y0 - 2000 * a))
                cv2.line(out, pt1, pt2, (0, 100, 255), 1)

    # Detected corners
    corners = _detect_board(gray, blur, edges, p)
    with _lock:
        _state["corners"] = corners

    if corners is not None and p["show_corners"]:
        labels = ['TL','TR','BR','BL']
        colors = [(255,0,0),(0,255,0),(0,0,255),(255,255,0)]
        for i, (pt, col, lbl) in enumerate(zip(corners, colors, labels)):
            cv2.circle(out, tuple(pt.astype(int)), 12, col, -1)
            cv2.putText(out, lbl, (int(pt[0])+14, int(pt[1])+6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, col, 2)
        cv2.polylines(out, [corners.astype(int).reshape(-1,1,2)], True, (0,255,0), 3)

    # Stats overlay
    n_lines = 0
    raw_lines = cv2.HoughLines(edges, 1, np.pi / 180, p["hough_thresh"])
    if raw_lines is not None: n_lines = len(raw_lines)
    cv2.putText(out, f"Lines: {n_lines}  Board: {'YES' if corners is not None else 'NO'}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0,255,0) if corners is not None else (0,0,255), 2)
    return out


def _add_status(img, detected):
    cv2.putText(img, "Board: " + ("DETECTED" if detected else "NOT FOUND"),
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0,255,0) if detected else (0,0,255), 2)


def _frame_to_jpeg(frame, scale=0.6):
    """Resize and encode frame to JPEG bytes."""
    h, w = frame.shape[:2]
    small = cv2.resize(frame, (int(w * scale), int(h * scale)))
    ok, buf = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 75])
    return bytes(buf) if ok else b''


# ─── Background processing thread ─────────────────────────────────────────────
_processed = {"main": None, "debug": None}
_proc_lock = threading.Lock()

def _processing_loop():
    """Runs every 2s: processes latest frame with current params."""
    while True:
        time.sleep(2.0)
        with _lock:
            frame = _state["raw_frame"]
        if frame is None:
            continue
        with _lock:
            p = dict(_params)
        try:
            main_img  = _render_main(frame, p)
            debug_img = _render_debug(frame, p)
            # After render_debug, update FEN
            with _lock:
                corners = _state["corners"]
            if corners is not None:
                sz = p["warp_size"]
                dst = np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]])
                M = cv2.getPerspectiveTransform(corners, dst)
                warped = cv2.warpPerspective(frame, M, (sz, sz))
                pieces64 = _analyze_squares(warped, p)
                fen = _build_fen(pieces64)
                with _lock:
                    _state["fen"] = fen
                    _state["frame_count"] += 1
                    _state["last_updated"] = time.time()
            with _proc_lock:
                _processed["main"]  = _frame_to_jpeg(main_img)
                _processed["debug"] = _frame_to_jpeg(debug_img)
        except Exception as e:
            with _lock:
                _state["error"] = str(e)


# ─── ROS subscriber ────────────────────────────────────────────────────────────
class FenNode(Node):
    def __init__(self):
        super().__init__("fen_visualizer")
        # Try BoardState, fallback to String
        try:
            try:
                from chess_perception.msg import BoardState
            except ImportError:
                from chess_interfaces.msg import BoardState
            self.create_subscription(BoardState, "/perception/board_state",
                                     self._on_bs, 10)
            self.get_logger().info("Subscribed to /perception/board_state (BoardState)")
        except ImportError:
            self.create_subscription(String, "/perception/board_state_fen",
                                     self._on_str, 10)
            self.get_logger().warn("Fallback: subscribed to /perception/board_state_fen (String)")

        self.create_subscription(Image, "/camera/image_raw", self._on_img, 10)
        self.get_logger().info("Subscribed to /camera/image_raw")

    def _on_bs(self, msg):
        fen = getattr(msg, "fen", "")
        if fen:
            with _lock:
                _state["fen"] = fen
                _state["last_updated"] = time.time()
                _state["frame_count"] += 1

    def _on_str(self, msg):
        with _lock:
            _state["fen"] = msg.data.strip()
            _state["last_updated"] = time.time()
            _state["frame_count"] += 1

    def _on_img(self, msg):
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            with _lock:
                _state["raw_frame"] = frame.copy()
        except Exception as e:
            pass


def _ros_thread_fn(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ─── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.logger.setLevel("ERROR")

HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Chess Perception Debug</title>
<link rel="stylesheet" href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
<script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
<script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px}
h1{color:#58a6ff;font-size:1.1em;padding:12px 16px;border-bottom:1px solid #21262d;display:flex;align-items:center;gap:10px}
#status-dot{width:10px;height:10px;border-radius:50%;background:#d29922;flex-shrink:0}
#status-dot.live{background:#3fb950;animation:pulse 2s infinite}
#status-dot.dead{background:#f85149}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.layout{display:grid;grid-template-columns:1fr 1fr 370px;grid-template-rows:auto auto;gap:10px;padding:10px}
.cam-panel{background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.cam-panel h2{font-size:.72em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;padding:6px 10px;border-bottom:1px solid #21262d;display:flex;justify-content:space-between}
.cam-panel img{width:100%;display:block}
.right-col{grid-row:1/3;display:flex;flex-direction:column;gap:10px}
.card{background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.card h2{font-size:.72em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;padding:6px 10px;border-bottom:1px solid #21262d}
.card-body{padding:10px}
#board-wrap{padding:10px;display:flex;justify-content:center}
#board{width:300px}
#fen-text{font-family:monospace;font-size:.72em;color:#3fb950;word-break:break-all;background:#0d1117;border:1px solid #21262d;border-radius:4px;padding:6px;margin-top:6px}
.stat-row{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #21262d;font-size:.78em}
.stat-row:last-child{border-bottom:none}
.lbl{color:#8b949e}.val{font-family:monospace}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}
.ctrl-panel{grid-column:1/3;background:#161b22;border:1px solid #21262d;border-radius:6px;padding:12px}
.ctrl-panel h2{font-size:.72em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;margin-bottom:10px}
.ctrl-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(195px,1fr));gap:8px 16px}
.ctrl-group{background:#0d1117;border:1px solid #21262d;border-radius:5px;padding:8px}
.ctrl-group-title{font-size:.68em;text-transform:uppercase;color:#58a6ff;margin-bottom:6px;font-weight:700}
.slider-row{display:flex;align-items:center;gap:5px;margin:3px 0}
.slider-row label{flex:0 0 110px;font-size:.72em;color:#8b949e;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.slider-row input[type=range]{flex:1;accent-color:#58a6ff}
.slider-row .vb{flex:0 0 36px;text-align:right;font-family:monospace;font-size:.72em;color:#c9d1d9}
.tog-row{display:flex;align-items:center;gap:7px;margin:3px 0}
.tog-row label{font-size:.72em;color:#8b949e}
.tog-row input[type=checkbox]{accent-color:#58a6ff;width:13px;height:13px}
.hsv-preview{display:flex;gap:4px;margin-top:6px}
.hsv-swatch{flex:1;height:18px;border-radius:3px;border:1px solid #30363d}
.btn{background:#21262d;color:#c9d1d9;border:1px solid #30363d;padding:4px 10px;border-radius:4px;cursor:pointer;font-size:.75em;text-decoration:none;display:inline-block}
.btn:hover{background:#30363d}
.btn.blue{background:#1f6feb;color:#fff;border-color:#1f6feb}
.btn.blue:hover{background:#388bfd}
.diff-item{font-family:monospace;font-size:.72em;padding:2px 5px;margin:1px 0;background:#2a1515;color:#f85149;border-radius:3px}
.inj-row{display:flex;gap:6px;margin-top:6px}
.inj-row input{flex:1;background:#0d1117;color:#c9d1d9;border:1px solid #21262d;border-radius:4px;padding:4px 7px;font-family:monospace;font-size:.72em}
</style>
</head>
<body>
<h1>
  <span id="status-dot"></span>
  ♟ Chess Perception — Live Debug Dashboard
  <span style="margin-left:auto;display:flex;gap:6px">
    <button class="btn" onclick="forceReprocess()">⟳ Reprocess Now</button>
    <a href="/api/snapshot" class="btn">📷 Download Snapshot</a>
  </span>
</h1>

<div class="layout">

  <div class="cam-panel">
    <h2>Live Camera + Grid &amp; Piece Overlay <span id="cam-ts" class="warn"></span></h2>
    <img id="cam-img" src="/api/frame" alt="camera">
  </div>

  <div class="cam-panel">
    <h2>Debug: Edges / Hough Lines / Corners <span id="dbg-ts" class="ok"></span></h2>
    <img id="dbg-img" src="/api/debug_frame" alt="debug">
  </div>

  <div class="right-col">

    <div class="card">
      <h2>Status</h2>
      <div class="card-body">
        <div class="stat-row"><span class="lbl">Feed</span><span class="val" id="conn-stat">–</span></div>
        <div class="stat-row"><span class="lbl">Frames processed</span><span class="val" id="fc">0</span></div>
        <div class="stat-row"><span class="lbl">FEN age</span><span class="val"><span id="fen-age">–</span>s</span></div>
        <div class="stat-row"><span class="lbl">Error</span><span class="val err" id="err-msg" style="font-size:.68em;word-break:break-all">–</span></div>
      </div>
    </div>

    <div class="card">
      <h2>Board State</h2>
      <div id="board-wrap"><div id="board"></div></div>
      <div style="padding:0 10px 10px"><div id="fen-text">–</div></div>
    </div>

    <div class="card">
      <h2>Diff vs Reference</h2>
      <div class="card-body">
        <div style="display:flex;gap:6px;margin-bottom:7px">
          <button class="btn blue" onclick="setRef()">📌 Set Reference</button>
          <button class="btn" onclick="resetRef()">Reset</button>
        </div>
        <div id="ref-lbl" style="font-size:.7em;color:#8b949e;margin-bottom:5px">Reference: starting position</div>
        <div id="diff-list" style="max-height:110px;overflow-y:auto"><span style="color:#8b949e;font-size:.78em">Set a reference first</span></div>
      </div>
    </div>

    <div class="card">
      <h2>Inject FEN (offline test)</h2>
      <div class="card-body">
        <div class="inj-row">
          <input id="inj-in" type="text" placeholder="paste FEN…"
            value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
          <button class="btn blue" onclick="injectFen()">Send</button>
        </div>
      </div>
    </div>

  </div><!-- /right-col -->

  <div class="ctrl-panel">
    <h2>Detection Parameters — sliders apply immediately on release</h2>
    <div class="ctrl-grid" id="ctrl-grid"></div>
  </div>

</div><!-- /layout -->

<script>
// Board
var board = Chessboard('board',{position:'start',showNotation:true,
  pieceTheme:'https://lichess1.org/assets/piece/cburnett/{piece}.svg'});
var refFen='rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1';
var curParams={};

// Parameter definitions
var GROUPS=[
  {t:'Preprocessing',p:[{k:'blur_ksize',l:'Blur kernel',mn:1,mx:21,st:2}]},
  {t:'Canny Edges',p:[
    {k:'canny_low',l:'Low threshold',mn:10,mx:255,st:5},
    {k:'canny_high',l:'High threshold',mn:20,mx:400,st:5}]},
  {t:'Hough Lines',p:[
    {k:'hough_thresh',l:'Accumulator',mn:20,mx:250,st:5},
    {k:'hough_min_len',l:'Min line len',mn:10,mx:300,st:5},
    {k:'hough_max_gap',l:'Max gap',mn:1,mx:100,st:1}]},
  {t:'Contour Fallback',p:[{k:'contour_min_area',l:'Min area px²',mn:1000,mx:250000,st:1000}]},
  {t:'Light Square (HSV)',p:[
    {k:'sq_light_h_lo',l:'H lo',mn:0,mx:179,st:1},
    {k:'sq_light_h_hi',l:'H hi',mn:0,mx:179,st:1},
    {k:'sq_light_s_lo',l:'S lo',mn:0,mx:255,st:1},
    {k:'sq_light_s_hi',l:'S hi',mn:0,mx:255,st:1},
    {k:'sq_light_v_lo',l:'V lo',mn:0,mx:255,st:1},
    {k:'sq_light_v_hi',l:'V hi',mn:0,mx:255,st:1}],
   hsv:true,sw:'sl'},
  {t:'Dark Square (HSV)',p:[
    {k:'sq_dark_h_lo',l:'H lo',mn:0,mx:179,st:1},
    {k:'sq_dark_h_hi',l:'H hi',mn:0,mx:179,st:1},
    {k:'sq_dark_s_lo',l:'S lo',mn:0,mx:255,st:1},
    {k:'sq_dark_s_hi',l:'S hi',mn:0,mx:255,st:1},
    {k:'sq_dark_v_lo',l:'V lo',mn:0,mx:255,st:1},
    {k:'sq_dark_v_hi',l:'V hi',mn:0,mx:255,st:1}],
   hsv:true,sw:'sd'},
  {t:'Piece Detection',p:[
    {k:'piece_std_thresh',l:'Std-dev thresh',mn:5,mx:100,st:1},
    {k:'piece_white_bright',l:'White min bright',mn:80,mx:255,st:1},
    {k:'piece_black_bright',l:'Black max bright',mn:10,mx:200,st:1}]},
  {t:'Overlays',togs:[
    {k:'show_edges',l:'Canny edges'},
    {k:'show_lines',l:'Hough lines'},
    {k:'show_corners',l:'Board corners'},
    {k:'show_grid',l:'Grid overlay'},
    {k:'show_pieces',l:'Piece labels'},
    {k:'show_color_mask',l:'HSV color mask'}]}
];

function hsv2css(h,s,v){
  return 'hsl('+(h*2)+','+Math.round(s/255*100)+'%,'+Math.round(v/255*50)+'%)';
}

function buildControls(p){
  var grid=document.getElementById('ctrl-grid');
  grid.innerHTML='';
  GROUPS.forEach(function(g){
    var box=document.createElement('div');
    box.className='ctrl-group';
    var html='<div class="ctrl-group-title">'+g.t+'</div>';
    if(g.togs){
      g.togs.forEach(function(t){
        html+='<div class="tog-row"><input type="checkbox" id="t_'+t.k+'" '+(p[t.k]?'checked':'')+
              ' onchange="onTog(\''+t.k+'\',this.checked)"><label for="t_'+t.k+'">'+t.l+'</label></div>';
      });
    } else {
      g.p.forEach(function(pr){
        var v=p[pr.k]!==undefined?p[pr.k]:0;
        html+='<div class="slider-row"><label title="'+pr.k+'">'+pr.l+'</label>'+
              '<input type="range" id="s_'+pr.k+'" min="'+pr.mn+'" max="'+pr.mx+'" step="'+pr.st+'" value="'+v+'"'+
              ' oninput="document.getElementById(\'vb_'+pr.k+'\').textContent=this.value"'+
              ' onchange="onSlider(\''+pr.k+'\',+this.value)">'+
              '<span class="vb" id="vb_'+pr.k+'">'+v+'</span></div>';
      });
      if(g.hsv&&g.sw){
        html+='<div class="hsv-preview">'+
              '<div class="hsv-swatch" id="'+g.sw+'lo" title="lower bound"></div>'+
              '<div class="hsv-swatch" id="'+g.sw+'hi" title="upper bound"></div></div>';
      }
    }
    box.innerHTML=html;
    grid.appendChild(box);
  });
  refreshSwatches(p);
}

function refreshSwatches(p){
  function s(id,h,sat,v){var e=document.getElementById(id);if(e)e.style.background=hsv2css(h,sat,v);}
  s('sllo',p.sq_light_h_lo,p.sq_light_s_lo,p.sq_light_v_lo);
  s('slhi',p.sq_light_h_hi,p.sq_light_s_hi,p.sq_light_v_hi);
  s('sdlo',p.sq_dark_h_lo, p.sq_dark_s_lo, p.sq_dark_v_lo);
  s('sdhi',p.sq_dark_h_hi, p.sq_dark_s_hi, p.sq_dark_v_hi);
}

function onSlider(k,v){
  curParams[k]=v;
  refreshSwatches(curParams);
  sendParams({[k]:v});
}
function onTog(k,c){curParams[k]=c?1:0;sendParams({[k]:c?1:0});}
function sendParams(d){
  fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
}
function forceReprocess(){sendParams({});}

// FEN diff
function fenSq(fen){
  var r=fen.split(' ')[0].split('/'),sq={},f='abcdefgh';
  r.forEach(function(row,ri){var fi=0;for(var i=0;i<row.length;i++){var c=row[i];if(isNaN(c)){sq[f[fi]+(8-ri)]=c;fi++;}else fi+=+c;}});
  return sq;
}
function plabel(p){var m={K:'♔K',Q:'♕Q',R:'♖R',B:'♗B',N:'♘N',P:'♙P',k:'♚k',q:'♛q',r:'♜r',b:'♝b',n:'♞n',p:'♟p','(empty)':'□'};return m[p]||p;}
function showDiff(a,b){
  var A=fenSq(a),B=fenSq(b),all=new Set(Object.keys(A).concat(Object.keys(B))),d=[];
  all.forEach(function(sq){var av=A[sq]||'(empty)',bv=B[sq]||'(empty)';if(av!==bv)d.push(sq.toUpperCase()+': '+plabel(av)+' → '+plabel(bv));});
  var el=document.getElementById('diff-list');
  el.innerHTML=d.length?d.map(function(x){return'<div class="diff-item">'+x+'</div>';}).join(''):'<span class="ok" style="font-size:.78em">✓ Matches reference</span>';
}
function setRef(){fetch('/api/fen').then(r=>r.json()).then(d=>{refFen=d.fen;document.getElementById('ref-lbl').textContent='Ref: '+d.fen.split(' ')[0];});}
function resetRef(){refFen='rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1';document.getElementById('ref-lbl').textContent='Reference: starting position';}
function injectFen(){var f=document.getElementById('inj-in').value.trim();fetch('/api/fen',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({fen:f})});}

// Refresh image feeds every 2s
function refreshImages(){
  var t=Date.now();
  document.getElementById('cam-img').src='/api/frame?t='+t;
  document.getElementById('dbg-img').src='/api/debug_frame?t='+t;
  document.getElementById('cam-ts').textContent=new Date().toLocaleTimeString();
  document.getElementById('dbg-ts').textContent=new Date().toLocaleTimeString();
}

// Poll FEN every 1s
function pollFen(){
  fetch('/api/fen').then(r=>r.json()).then(d=>{
    try{board.position(d.fen.split(' ')[0],false);}catch(e){}
    document.getElementById('fen-text').textContent=d.fen;
    document.getElementById('fc').textContent=d.frame_count;
    var age=d.last_updated>0?(Date.now()/1000-d.last_updated):9999;
    var ae=document.getElementById('fen-age');
    ae.textContent=age<9999?age.toFixed(1):'never';
    ae.className=age<4?'ok':age<15?'warn':'err';
    var dot=document.getElementById('status-dot'),cs=document.getElementById('conn-stat');
    if(age<4){dot.className='live';cs.textContent='Live ✓';cs.className='val ok';}
    else if(age<30){dot.className='';cs.textContent='Stale';cs.className='val warn';}
    else{dot.className='dead';cs.textContent='No data';cs.className='val err';}
    var em=document.getElementById('err-msg');
    em.textContent=d.error||'–';
    showDiff(refFen,d.fen);
  }).catch(function(){document.getElementById('status-dot').className='dead';});
}

// Init
fetch('/api/params').then(r=>r.json()).then(function(p){curParams=p;buildControls(p);});
setInterval(refreshImages,2000);
setInterval(pollFen,1000);
refreshImages();pollFen();
</script>
</body>
</html>'''


@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/api/fen")
def api_fen():
    with _lock:
        return jsonify({
            "fen":          _state["fen"],
            "frame_count":  _state["frame_count"],
            "last_updated": _state["last_updated"],
            "error":        _state["error"],
        })


@app.route("/api/fen", methods=["POST"])
def api_fen_post():
    data = freq.get_json(silent=True) or {}
    fen = data.get("fen", "").strip()
    if not fen:
        return jsonify({"ok": False}), 400
    with _lock:
        _state["fen"] = fen
        _state["last_updated"] = time.time()
        _state["frame_count"] += 1
    return jsonify({"ok": True})


@app.route("/api/frame")
def api_frame():
    with _proc_lock:
        data = _processed.get("main")
    if not data:
        # Return blank grey frame
        blank = np.full((360, 640, 3), 50, dtype=np.uint8)
        cv2.putText(blank, "Waiting for camera...", (120, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (200,200,200), 2)
        _, buf = cv2.imencode('.jpg', blank)
        data = bytes(buf)
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/debug_frame")
def api_debug_frame():
    with _proc_lock:
        data = _processed.get("debug")
    if not data:
        blank = np.full((360, 640, 3), 30, dtype=np.uint8)
        cv2.putText(blank, "Waiting for debug frame...", (80, 180),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (100,200,100), 2)
        _, buf = cv2.imencode('.jpg', blank)
        data = bytes(buf)
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/params", methods=["GET"])
def api_params_get():
    with _lock:
        return jsonify(dict(_params))


@app.route("/api/params", methods=["POST"])
def api_params_post():
    data = freq.get_json(silent=True) or {}
    with _lock:
        for k, v in data.items():
            if k in _params:
                _params[k] = type(_params[k])(v)
    # Trigger immediate reprocess
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})


def _force_reprocess():
    """Immediately reprocess the latest frame when params change."""
    with _lock:
        frame = _state["raw_frame"]
        p = dict(_params)
    if frame is None:
        return
    try:
        main_img  = _render_main(frame, p)
        debug_img = _render_debug(frame, p)
        with _lock:
            corners = _state["corners"]
        if corners is not None:
            sz = p["warp_size"]
            dst = np.float32([[0,0],[sz-1,0],[sz-1,sz-1],[0,sz-1]])
            M = cv2.getPerspectiveTransform(corners, dst)
            warped = cv2.warpPerspective(frame, M, (sz, sz))
            pieces64 = _analyze_squares(warped, p)
            fen = _build_fen(pieces64)
            with _lock:
                _state["fen"] = fen
                _state["last_updated"] = time.time()
                _state["frame_count"] += 1
        with _proc_lock:
            _processed["main"]  = _frame_to_jpeg(main_img)
            _processed["debug"] = _frame_to_jpeg(debug_img)
    except Exception as e:
        with _lock:
            _state["error"] = str(e)


@app.route("/api/snapshot")
def api_snapshot():
    """Return current raw frame as full-res JPEG for download."""
    with _lock:
        frame = _state["raw_frame"]
    if frame is None:
        return Response(b'', mimetype='image/jpeg')
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return Response(bytes(buf), mimetype='image/jpeg',
                    headers={"Content-Disposition": "attachment; filename=snapshot.jpg"})


# ─── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=5000)
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--fen", default=None)
    parser.add_argument("--image", default=None,
                        help="Load a static image instead of camera (for offline testing)")
    args = parser.parse_args()

    if args.fen:
        with _lock:
            _state["fen"] = args.fen
            _state["last_updated"] = time.time()
            _state["frame_count"] = 1

    if args.image:
        frame = cv2.imread(args.image)
        if frame is not None:
            with _lock:
                _state["raw_frame"] = frame
            print(f"Loaded static image: {args.image}  {frame.shape}")

    # Start background processing
    threading.Thread(target=_processing_loop, daemon=True).start()

    # Start ROS subscriber
    if not args.no_ros:
        if not ROS_AVAILABLE:
            print("ERROR: rclpy not found. Use --no-ros"); sys.exit(1)
        rclpy.init()
        ros_node = FenNode()
        threading.Thread(target=_ros_thread_fn, args=(ros_node,), daemon=True).start()
        print("✓ ROS subscriber started")
    else:
        print("⚠  No-ROS mode. Use --image <path> or POST /api/fen")

    print(f"\nOpen browser: http://localhost:{args.port}")
    print(f"Network URL:  http://<pi-ip>:{args.port}\n")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
