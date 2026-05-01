#!/usr/bin/env python3
"""
FEN Live Board Visualizer — Corner-Pin Edition
================================================
Board corners are manually defined (draggable in the browser).
No edge/Hough detection is used for board finding.

Pipeline:
  1. User drags 4 corner handles on the live camera image.
  2. Perspective warp → 480×480 top-down board view.
  3. 8×8 grid is applied (chess pattern is known; no color detection needed).
  4. White/black piece blobs detected in each row's warped image.
  5. Each blob is assigned to ONE square using the BOTTOM pixel of the blob
     (important: camera is angled ~45° from player side, so far pieces lean back).
  6. FEN is emitted.

Usage:
  python3 src/chess_perception/scripts/fen_visualizer.py
  python3 src/chess_perception/scripts/fen_visualizer.py --no-ros --image /path/to/frame.jpg
"""

import argparse, threading, time, sys, json, os
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

# ─── Shared state ─────────────────────────────────────────────────────────────
_lock = threading.Lock()

# Corners: [TL, TR, BR, BL] in pixel coords of the raw camera frame.
# Default spread assumes ~640×480 frame with board roughly filling it.
_DEFAULT_CORNERS = [
    [80.0,  60.0],   # TL
    [560.0, 60.0],   # TR
    [560.0, 420.0],  # BR
    [80.0,  420.0],  # BL
]

_state = {
    "fen":          "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "frame_count":  0,
    "last_updated": 0.0,
    "error":        "",
    "raw_frame":    None,       # latest BGR ndarray
    "cam_info":     "–",
    # 4-point corners [[x,y]×4] in raw-frame pixels; user sets via UI
    "corners":      [list(c) for c in _DEFAULT_CORNERS],
    # Per-square piece map: 64-elem list of '' | 'W' | 'B'
    "pieces64":     [""] * 64,
}

# Tunable detection params — all editable from the browser
_params = {
    # Piece colour thresholds (in warped grayscale)
    "white_thresh":     200,   # pixels >= this are "white piece candidate"
    "black_thresh":      60,   # pixels <= this are "black piece candidate"
    # Minimum blob area (px²) in the warped image to count as a piece
    "min_blob_area":     80,
    # Fraction of square height used as look-ahead above the square (0..1)
    # Rows further from camera (smaller row index = farther) use their bottom
    # anchor for assignment; this controls how far UP we look.
    "bottom_anchor_bias": 0.85,  # 0 = centre, 1 = full bottom-of-blob
    # Undistortion (disabled by default)
    "undistort_enable": 0,
    "undistort_k1":    -25,
    "undistort_k2":      8,
    "undistort_focal":  83,
    # Warp output size (square)
    "warp_size":        480,
    # Display toggles
    "show_grid":        1,
    "show_pieces":      1,
    "show_white_mask":  0,
    "show_black_mask":  0,
}

# Corners file path (persists across restarts)
_CORNERS_FILE = os.path.join(os.path.dirname(__file__), "board_corners.json")


def _load_corners():
    try:
        with open(_CORNERS_FILE) as f:
            data = json.load(f)
        if isinstance(data, list) and len(data) == 4:
            with _lock:
                _state["corners"] = data
            print(f"✓ Loaded corners from {_CORNERS_FILE}")
    except FileNotFoundError:
        pass
    except Exception as e:
        print(f"⚠  Could not load corners: {e}")


def _save_corners(corners):
    try:
        with open(_CORNERS_FILE, "w") as f:
            json.dump(corners, f, indent=2)
    except Exception as e:
        print(f"⚠  Could not save corners: {e}")


# ─── Lens undistortion ────────────────────────────────────────────────────────
def _undistort_frame(frame: np.ndarray, p: dict) -> np.ndarray:
    if not p.get("undistort_enable", 0):
        return frame
    h, w = frame.shape[:2]
    f = p["undistort_focal"] / 100.0 * w
    K = np.array([[f, 0, w / 2.0],
                  [0, f, h / 2.0],
                  [0, 0, 1.0]], dtype=np.float64)
    D = np.array([p["undistort_k1"] / 100.0,
                  p["undistort_k2"] / 100.0,
                  0.0, 0.0], dtype=np.float64)
    return cv2.undistort(frame, K, D)


# ─── Perspective warp ─────────────────────────────────────────────────────────
def _get_warp_matrix(corners, warp_size):
    """Return M (raw→warp) and M_inv (warp→raw) from 4 corner points."""
    src = np.float32(corners)          # [TL, TR, BR, BL]
    sz = warp_size - 1
    dst = np.float32([[0, 0], [sz, 0], [sz, sz], [0, sz]])
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv


def _warp_frame(frame, corners, warp_size):
    M, M_inv = _get_warp_matrix(corners, warp_size)
    warped = cv2.warpPerspective(frame, M, (warp_size, warp_size))
    return warped, M, M_inv


# ─── Piece blob detection ─────────────────────────────────────────────────────
def _make_piece_masks(warped_gray, p):
    """Return binary masks for white and black pieces."""
    white_mask = cv2.threshold(
        warped_gray, p["white_thresh"], 255, cv2.THRESH_BINARY)[1]
    black_mask = cv2.threshold(
        warped_gray, p["black_thresh"], 255, cv2.THRESH_BINARY_INV)[1]
    # Morphological clean-up
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, k)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, k)
    return white_mask, black_mask


def _assign_blob_to_square(blob_mask, sq_size, p, piece_color):
    """
    Given a binary mask of one blob (in the full warped image),
    find which single square it belongs to by using the BOTTOM-ANCHOR rule:

    - Compute the bounding box of the blob.
    - Use the bottom-most point of the blob (y_max) weighted by bias param
      to choose the row.
    - Use the horizontal centroid to choose the column.
    - This corrects for camera angle: far pieces appear higher on screen,
      their tops lean back, so the base is the reliable anchor.

    Returns (row, col) 0-indexed, or None if blob is too small.
    """
    contours, _ = cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assignments = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < p["min_blob_area"]:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        # Moments for centroid
        M_cnt = cv2.moments(cnt)
        if M_cnt["m00"] == 0:
            continue
        cx = int(M_cnt["m10"] / M_cnt["m00"])
        # Bottom-anchor: blend between centroid-y and bottom of bounding box
        cy_center = int(M_cnt["m01"] / M_cnt["m00"])
        cy_bottom = y + h
        bias = p["bottom_anchor_bias"]
        cy_anchor = int(cy_center * (1.0 - bias) + cy_bottom * bias)
        # Clamp to image bounds
        cy_anchor = min(cy_anchor, sq_size * 8 - 1)
        cx      = min(max(cx, 0), sq_size * 8 - 1)
        row = cy_anchor // sq_size
        col = cx // sq_size
        assignments.append((row, col, area))
    return assignments


def _detect_pieces(warped, p):
    """
    Returns list of 64 strings: '' | 'W' | 'B'.
    Each blob is assigned to exactly ONE square.
    If two blobs compete for the same square, the larger blob wins.
    """
    sq = p["warp_size"] // 8
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    white_mask, black_mask = _make_piece_masks(gray, p)

    # square → best (area, color)
    best = {}   # (row,col) → (area, 'W'|'B')

    for color_char, mask in [('W', white_mask), ('B', black_mask)]:
        assignments = _assign_blob_to_square(mask, sq, p, color_char)
        for row, col, area in assignments:
            key = (row, col)
            if key not in best or area > best[key][0]:
                best[key] = (area, color_char)

    pieces64 = [""] * 64
    for (row, col), (area, color_char) in best.items():
        if 0 <= row < 8 and 0 <= col < 8:
            pieces64[row * 8 + col] = color_char
    return pieces64, white_mask, black_mask


# ─── FEN builder ──────────────────────────────────────────────────────────────
def _build_fen(pieces64):
    """
    pieces64[0] = top-left of warped image = rank 8, file a  (far side).
    Row 0 in warped = rank 8; row 7 = rank 1 (player side, near camera).
    """
    rows = []
    for rank_i in range(7, -1, -1):   # rank 8 → 1
        warp_row = 7 - rank_i         # warp row 0 = rank 8
        empty, row_str = 0, ''
        for file_i in range(8):
            idx = warp_row * 8 + file_i
            pc = pieces64[idx]
            if not pc:
                empty += 1
            else:
                if empty:
                    row_str += str(empty)
                    empty = 0
                # White pieces = uppercase P; black pieces = lowercase p
                row_str += 'P' if pc == 'W' else 'p'
        if empty:
            row_str += str(empty)
        rows.append(row_str)
    return '/'.join(rows) + ' w KQkq - 0 1'


# ─── Render helpers ───────────────────────────────────────────────────────────
def _draw_grid(img, sq):
    """Draw an 8×8 grid on a square image where each cell is sq×sq pixels."""
    size = sq * 8
    for i in range(9):
        cv2.line(img, (i * sq, 0),    (i * sq, size),  (0, 200, 200), 1)
        cv2.line(img, (0,    i * sq), (size,   i * sq), (0, 200, 200), 1)


def _draw_rank_file_labels(img, sq):
    for r in range(8):
        cv2.putText(img, str(8 - r), (3, r * sq + sq - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 50), 1)
    for fi, lbl in enumerate('abcdefgh'):
        cv2.putText(img, lbl, (fi * sq + sq // 2 - 4, sq * 8 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 50), 1)


def _frame_to_jpeg(frame, scale=0.8, quality=82):
    h, w = frame.shape[:2]
    if scale != 1.0:
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return bytes(buf) if ok else b''


def _blank_jpeg(w, h, msg, color=(200, 200, 200)):
    blank = np.full((h, w, 3), 50, dtype=np.uint8)
    cv2.putText(blank, msg, (20, h // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    _, buf = cv2.imencode('.jpg', blank)
    return bytes(buf)


# ─── Three rendered views ──────────────────────────────────────────────────────
def _render_raw(frame, corners, p, pieces64, M_inv):
    """Raw camera frame with corner handles + projected grid overlay."""
    out = frame.copy()
    sz = p["warp_size"]
    sq = sz // 8

    # Project grid lines back into raw-frame space
    if p["show_grid"] and M_inv is not None:
        for i in range(9):
            # vertical lines
            p0w = np.float32([[[i * sq, 0]]])
            p1w = np.float32([[[i * sq, sz]]])
            p0 = cv2.perspectiveTransform(p0w, M_inv)[0, 0].astype(int)
            p1 = cv2.perspectiveTransform(p1w, M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(p0), tuple(p1), (0, 200, 200), 1)
            # horizontal lines
            q0w = np.float32([[[0, i * sq]]])
            q1w = np.float32([[[sz, i * sq]]])
            q0 = cv2.perspectiveTransform(q0w, M_inv)[0, 0].astype(int)
            q1 = cv2.perspectiveTransform(q1w, M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(q0), tuple(q1), (0, 200, 200), 1)

    # Project piece labels back
    if p["show_pieces"] and M_inv is not None:
        for row in range(8):
            for col in range(8):
                pc = pieces64[row * 8 + col]
                if pc:
                    cx_w = col * sq + sq // 2
                    cy_w = row * sq + sq // 2
                    pt = cv2.perspectiveTransform(
                        np.float32([[[cx_w, cy_w]]]), M_inv)[0, 0].astype(int)
                    color = (255, 255, 255) if pc == 'W' else (30, 30, 30)
                    border = (0, 0, 0) if pc == 'W' else (200, 200, 200)
                    cv2.putText(out, pc, tuple(pt),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, border, 4)
                    cv2.putText(out, pc, tuple(pt),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Corner handles
    labels = ['TL', 'TR', 'BR', 'BL']
    handle_colors = [(255, 80, 80), (80, 255, 80), (80, 80, 255), (255, 255, 80)]
    for i, (cx, cy) in enumerate(corners):
        cv2.circle(out, (int(cx), int(cy)), 10, handle_colors[i], -1)
        cv2.circle(out, (int(cx), int(cy)), 10, (255, 255, 255), 1)
        cv2.putText(out, labels[i], (int(cx) + 12, int(cy) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, handle_colors[i], 2)
    # Board outline
    pts = np.array(corners, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, (0, 255, 100), 2)
    return out


def _render_warp(warped, p, pieces64, white_mask, black_mask):
    """Top-down corrected board with grid, checkerboard tint, piece circles."""
    out = warped.copy()
    sz = p["warp_size"]
    sq = sz // 8

    # Checkerboard tint
    for r in range(8):
        for c in range(8):
            if (r + c) % 2 == 0:
                roi = out[r*sq:(r+1)*sq, c*sq:(c+1)*sq].astype(np.float32)
                roi = cv2.addWeighted(roi, 0.82, np.full_like(roi, 190), 0.18, 0)
                out[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = roi.astype(np.uint8)

    if p["show_grid"]:
        _draw_grid(out, sq)
        _draw_rank_file_labels(out, sq)

    # Optional mask overlays (semi-transparent)
    if p["show_white_mask"] and white_mask is not None:
        overlay = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 0] = 0; overlay[:, :, 1] = 0   # red channel only
        out = cv2.addWeighted(out, 0.7, overlay, 0.3, 0)
    if p["show_black_mask"] and black_mask is not None:
        overlay = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 1] = 0; overlay[:, :, 2] = 0   # blue channel only
        out = cv2.addWeighted(out, 0.7, overlay, 0.3, 0)

    # Piece circles
    if p["show_pieces"]:
        for row in range(8):
            for col in range(8):
                pc = pieces64[row * 8 + col]
                if pc:
                    cx, cy = col * sq + sq // 2, row * sq + sq // 2
                    fill   = (240, 240, 240) if pc == 'W' else (25, 25, 25)
                    border = (80, 80, 80)
                    cv2.circle(out, (cx, cy), sq // 3, fill, -1)
                    cv2.circle(out, (cx, cy), sq // 3, border, 1)
                    tc = (30, 30, 30) if pc == 'W' else (220, 220, 220)
                    cv2.putText(out, pc, (cx - 8, cy + 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, tc, 2)

    cv2.rectangle(out, (0, 0), (sz - 1, sz - 1), (0, 255, 100), 2)
    return out


# ─── Background processing ────────────────────────────────────────────────────
_processed = {"raw": None, "warp": None}
_proc_lock = threading.Lock()


def _process_frame(frame, corners, p):
    try:
        warped, M, M_inv = _warp_frame(frame, corners, p["warp_size"])
        pieces64, white_mask, black_mask = _detect_pieces(warped, p)
        fen = _build_fen(pieces64)

        raw_img  = _render_raw(frame, corners, p, pieces64, M_inv)
        warp_img = _render_warp(warped, p, pieces64, white_mask, black_mask)

        with _lock:
            _state["pieces64"]     = pieces64
            _state["fen"]          = fen
            _state["frame_count"] += 1
            _state["last_updated"] = time.time()
            _state["error"]        = ""

        with _proc_lock:
            _processed["raw"]  = _frame_to_jpeg(raw_img,  scale=0.8)
            _processed["warp"] = _frame_to_jpeg(warp_img, scale=1.0)

    except Exception as e:
        with _lock:
            _state["error"] = str(e)


def _processing_loop():
    while True:
        time.sleep(1.5)
        with _lock:
            frame   = _state["raw_frame"]
            corners = _state["corners"]
            p       = dict(_params)
        if frame is None:
            continue
        frame = _undistort_frame(frame, p)
        _process_frame(frame, corners, p)


def _force_reprocess():
    with _lock:
        frame   = _state["raw_frame"]
        corners = _state["corners"]
        p       = dict(_params)
    if frame is None:
        return
    frame = _undistort_frame(frame, p)
    _process_frame(frame, corners, p)


# ─── ROS subscriber ────────────────────────────────────────────────────────────
class FenNode(Node):
    def __init__(self):
        super().__init__("fen_visualizer")
        try:
            try:
                from chess_perception.msg import BoardState
            except ImportError:
                from chess_interfaces.msg import BoardState
            self.create_subscription(BoardState, "/perception/board_state",
                                     self._on_bs, 10)
        except ImportError:
            self.create_subscription(String, "/perception/board_state_fen",
                                     self._on_str, 10)

        self.create_subscription(Image, "/camera/image_raw", self._on_img, 10)
        self.get_logger().info("FenNode ready — corner-pin mode")

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
            enc  = msg.encoding.lower()
            h, w = msg.height, msg.width
            step = msg.step
            data = np.frombuffer(msg.data, dtype=np.uint8)

            if enc in ('bgr8', 'bgr888'):
                frame = data.reshape((h, step))[:, :w * 3].reshape(h, w, 3)
            elif enc in ('rgb8', 'rgb888'):
                frame = data.reshape((h, step))[:, :w * 3].reshape(h, w, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif enc in ('yuv422', 'yuyv', 'yuyv422'):
                yuyv  = data.reshape((h, step))[:, :w * 2].reshape(h, w, 2)
                frame = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
            elif enc in ('uyvy', 'uyvy422'):
                uyvy  = data.reshape((h, step))[:, :w * 2].reshape(h, w, 2)
                frame = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
            elif enc in ('mono8', '8uc1'):
                mono  = data.reshape((h, step))[:, :w].reshape(h, w)
                frame = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
            else:
                bpp = len(msg.data) // (h * w) if h * w > 0 else 3
                frame = data.reshape((h, w, bpp if bpp in (3, 4) else 3)).copy()
                if bpp == 4:
                    frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            with _lock:
                _state["raw_frame"] = frame
                _state["cam_info"]  = f"{w}×{h} {msg.encoding}"
                _state["error"]     = ""
        except Exception as e:
            with _lock:
                _state["error"] = f"Image decode: {e}"


def _ros_thread_fn(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ─── Flask app + API ───────────────────────────────────────────────────────────
app = Flask(__name__)
app.logger.setLevel("ERROR")


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
            "cam_info":     _state.get("cam_info", "–"),
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
        data = _processed.get("raw")
    if not data:
        data = _blank_jpeg(640, 360, "Waiting for camera...")
    return Response(data, mimetype="image/jpeg",
                    headers={"Cache-Control": "no-store"})


@app.route("/api/warp_frame")
def api_warp_frame():
    with _proc_lock:
        data = _processed.get("warp")
    if not data:
        data = _blank_jpeg(480, 480, "No corners set", (80, 80, 80))
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
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/corners", methods=["GET"])
def api_corners_get():
    with _lock:
        return jsonify({"corners": _state["corners"]})


@app.route("/api/corners", methods=["POST"])
def api_corners_post():
    data = freq.get_json(silent=True) or {}
    corners = data.get("corners")
    if not corners or len(corners) != 4:
        return jsonify({"ok": False, "msg": "Need 4 corners"}), 400
    # Validate each is [x, y]
    try:
        corners = [[float(c[0]), float(c[1])] for c in corners]
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 400
    with _lock:
        _state["corners"] = corners
    _save_corners(corners)
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/snapshot")
def api_snapshot():
    with _lock:
        frame = _state["raw_frame"]
    if frame is None:
        return Response(b'', mimetype='image/jpeg')
    _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    return Response(bytes(buf), mimetype='image/jpeg',
                    headers={"Content-Disposition": "attachment; filename=snapshot.jpg"})


# ─── HTML_TEMPLATE placeholder (filled in Part 3) ─────────────────────────────
HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Chess Perception — Corner-Pin</title>
<link rel="stylesheet" href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
<script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
<script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px}
h1{color:#58a6ff;font-size:1.05em;padding:10px 16px;border-bottom:1px solid #21262d;display:flex;align-items:center;gap:10px}
#status-dot{width:10px;height:10px;border-radius:50%;background:#d29922;flex-shrink:0}
#status-dot.live{background:#3fb950;animation:pulse 2s infinite}
#status-dot.dead{background:#f85149}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}
.layout{display:grid;grid-template-columns:1fr 360px 280px;grid-template-rows:auto auto;gap:8px;padding:8px}
.cam-wrap{grid-column:1;grid-row:1;background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.cam-wrap h2{font-size:.68em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;padding:5px 10px;border-bottom:1px solid #21262d;display:flex;justify-content:space-between;align-items:center}
#cam-canvas-wrap{position:relative;width:100%}
#cam-img{width:100%;display:block;cursor:crosshair}
#corner-overlay{position:absolute;top:0;left:0;width:100%;height:100%;pointer-events:none}
.warp-panel{grid-column:2;grid-row:1;background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.warp-panel h2{font-size:.68em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;padding:5px 10px;border-bottom:1px solid #21262d}
.warp-panel img{width:100%;display:block;image-rendering:pixelated}
.right-col{grid-column:3;grid-row:1/3;display:flex;flex-direction:column;gap:8px}
.card{background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.card h2{font-size:.68em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;padding:5px 10px;border-bottom:1px solid #21262d}
.card-body{padding:8px}
#board-wrap{padding:8px;display:flex;justify-content:center}
#board{width:240px}
#fen-text{font-family:monospace;font-size:.65em;color:#3fb950;word-break:break-all;background:#0d1117;border:1px solid #21262d;border-radius:4px;padding:5px;margin-top:5px}
.stat-row{display:flex;justify-content:space-between;padding:3px 0;border-bottom:1px solid #21262d;font-size:.76em}
.stat-row:last-child{border-bottom:none}
.lbl{color:#8b949e}.val{font-family:monospace}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}
.ctrl-panel{grid-column:1/3;grid-row:2;background:#161b22;border:1px solid #21262d;border-radius:6px;padding:10px}
.ctrl-panel h2{font-size:.68em;text-transform:uppercase;letter-spacing:.08em;color:#8b949e;margin-bottom:8px}
.ctrl-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(200px,1fr));gap:7px 14px}
.ctrl-group{background:#0d1117;border:1px solid #21262d;border-radius:5px;padding:7px}
.ctrl-group-title{font-size:.65em;text-transform:uppercase;color:#58a6ff;margin-bottom:5px;font-weight:700}
.slider-row{display:flex;align-items:center;gap:5px;margin:3px 0}
.slider-row label{flex:0 0 130px;font-size:.7em;color:#8b949e;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.slider-row input[type=range]{flex:1;accent-color:#58a6ff}
.slider-row .vb{flex:0 0 38px;text-align:right;font-family:monospace;font-size:.7em;color:#c9d1d9}
.tog-row{display:flex;align-items:center;gap:7px;margin:3px 0}
.tog-row label{font-size:.7em;color:#8b949e}
.tog-row input[type=checkbox]{accent-color:#58a6ff;width:13px;height:13px}
.btn{background:#21262d;color:#c9d1d9;border:1px solid #30363d;padding:3px 9px;border-radius:4px;cursor:pointer;font-size:.73em;text-decoration:none;display:inline-block}
.btn:hover{background:#30363d}
.btn.blue{background:#1f6feb;color:#fff;border-color:#1f6feb}
.btn.blue:hover{background:#388bfd}
.btn.green{background:#196127;color:#fff;border-color:#196127}
.btn.green:hover{background:#238636}
.corner-info{font-size:.68em;color:#8b949e;padding:5px 10px 0}
.handle{cursor:grab;position:absolute;width:20px;height:20px;border-radius:50%;transform:translate(-50%,-50%);border:2px solid white;display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:bold;color:#fff;user-select:none;z-index:10}
.handle:active{cursor:grabbing}
</style>
</head>
<body>
<h1>
  <span id="status-dot"></span>
  ♟ Chess Perception — Corner-Pin Mode
  <span style="margin-left:auto;display:flex;gap:6px">
    <button class="btn blue" onclick="forceReprocess()">⟳ Reprocess</button>
    <button class="btn green" onclick="saveCorners()">📌 Save Corners</button>
    <button class="btn green" onclick="resetCorners()">↺ Reset Corners</button>
    <a href="/api/snapshot" class="btn">📷 Snapshot</a>
  </span>
</h1>

<div class="layout">

  <!-- Camera + draggable corners -->
  <div class="cam-wrap">
    <h2>Live Camera — drag corners to define board <span id="cam-ts" class="warn"></span></h2>
    <div id="cam-canvas-wrap">
      <img id="cam-img" src="/api/frame" alt="camera" draggable="false">
      <!-- Corner handles injected by JS -->
    </div>
    <div class="corner-info" id="corner-coords">Corners: loading…</div>
  </div>

  <!-- Warped board -->
  <div class="warp-panel">
    <h2>Top-Down Board View</h2>
    <img id="warp-img" src="/api/warp_frame" alt="warp">
  </div>

  <!-- Right column -->
  <div class="right-col">
    <div class="card">
      <h2>Status</h2>
      <div class="card-body">
        <div class="stat-row"><span class="lbl">Feed</span><span class="val" id="conn-stat">–</span></div>
        <div class="stat-row"><span class="lbl">Camera</span><span class="val" id="cam-info">–</span></div>
        <div class="stat-row"><span class="lbl">Frames</span><span class="val" id="fc">0</span></div>
        <div class="stat-row"><span class="lbl">FEN age</span><span class="val"><span id="fen-age">–</span>s</span></div>
        <div class="stat-row"><span class="lbl">Error</span><span class="val err" id="err-msg" style="font-size:.65em;word-break:break-all">–</span></div>
      </div>
    </div>
    <div class="card">
      <h2>Board State</h2>
      <div id="board-wrap"><div id="board"></div></div>
      <div style="padding:0 8px 8px"><div id="fen-text">–</div></div>
    </div>
    <div class="card">
      <h2>Inject FEN (offline)</h2>
      <div class="card-body" style="display:flex;gap:6px">
        <input id="inj-in" type="text" style="flex:1;background:#0d1117;color:#c9d1d9;border:1px solid #21262d;border-radius:4px;padding:4px 7px;font-family:monospace;font-size:.7em"
          value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
        <button class="btn blue" onclick="injectFen()">Send</button>
      </div>
    </div>
  </div>

  <!-- Parameter controls -->
  <div class="ctrl-panel">
    <h2>Detection Parameters</h2>
    <div class="ctrl-grid" id="ctrl-grid"></div>
  </div>

</div>

<script>
// ── Chess board UI ──────────────────────────────────────────────────────────
var chessboard = Chessboard('board', {
  position: 'start', showNotation: true,
  pieceTheme: 'https://lichess1.org/assets/piece/cburnett/{piece}.svg'
});

// ── Corner management ───────────────────────────────────────────────────────
// corners[i] = {x, y} in IMAGE pixel space (before any browser scaling)
var corners = [{x:80,y:60},{x:560,y:60},{x:560,y:420},{x:80,y:420}];
var CORNER_LABELS = ['TL','TR','BR','BL'];
var CORNER_COLORS = ['#ff5050','#50ff50','#5050ff','#ffff50'];
var handles = [];
var dragging = null;  // {index, startMouseX, startMouseY, startCornerX, startCornerY}

function imgToDisplay(x, y) {
  // Convert image-pixel coords → display-pixel coords relative to cam-img element
  var img = document.getElementById('cam-img');
  var scaleX = img.clientWidth  / (img.naturalWidth  || img.clientWidth);
  var scaleY = img.clientHeight / (img.naturalHeight || img.clientHeight);
  return {x: x * scaleX, y: y * scaleY};
}

function displayToImg(dx, dy) {
  var img = document.getElementById('cam-img');
  var scaleX = img.clientWidth  / (img.naturalWidth  || img.clientWidth);
  var scaleY = img.clientHeight / (img.naturalHeight || img.clientHeight);
  return {x: dx / scaleX, y: dy / scaleY};
}

function buildHandles() {
  var wrap = document.getElementById('cam-canvas-wrap');
  // Remove old handles
  handles.forEach(function(h){ if(h.parentNode) h.parentNode.removeChild(h); });
  handles = [];
  corners.forEach(function(c, i) {
    var el = document.createElement('div');
    el.className = 'handle';
    el.style.background = CORNER_COLORS[i];
    el.textContent = CORNER_LABELS[i];
    el.style.boxShadow = '0 0 6px rgba(0,0,0,0.8)';
    wrap.appendChild(el);
    handles.push(el);

    el.addEventListener('mousedown', function(e) {
      e.preventDefault();
      dragging = {index: i, startMouseX: e.clientX, startMouseY: e.clientY,
                  startCornerX: corners[i].x, startCornerY: corners[i].y};
    });
  });
  positionHandles();
}

function positionHandles() {
  var img = document.getElementById('cam-img');
  var rect = img.getBoundingClientRect();
  var wrapRect = document.getElementById('cam-canvas-wrap').getBoundingClientRect();
  corners.forEach(function(c, i) {
    var d = imgToDisplay(c.x, c.y);
    handles[i].style.left = d.x + 'px';
    handles[i].style.top  = d.y + 'px';
  });
}

document.addEventListener('mousemove', function(e) {
  if (!dragging) return;
  var img = document.getElementById('cam-img');
  var rect = img.getBoundingClientRect();
  var dx = e.clientX - dragging.startMouseX;
  var dy = e.clientY - dragging.startMouseY;
  // Convert delta in display px → image px
  var scaleX = img.clientWidth  / (img.naturalWidth  || img.clientWidth);
  var scaleY = img.clientHeight / (img.naturalHeight || img.clientHeight);
  corners[dragging.index].x = dragging.startCornerX + dx / scaleX;
  corners[dragging.index].y = dragging.startCornerY + dy / scaleY;
  positionHandles();
  updateCornerInfo();
});

document.addEventListener('mouseup', function(e) {
  if (!dragging) return;
  dragging = null;
  sendCorners();
});

// Touch support
document.addEventListener('touchmove', function(e) {
  if (!dragging) return;
  e.preventDefault();
  var t = e.touches[0];
  var img = document.getElementById('cam-img');
  var scaleX = img.clientWidth  / (img.naturalWidth  || img.clientWidth);
  var scaleY = img.clientHeight / (img.naturalHeight || img.clientHeight);
  var dx = t.clientX - dragging.startMouseX;
  var dy = t.clientY - dragging.startMouseY;
  corners[dragging.index].x = dragging.startCornerX + dx / scaleX;
  corners[dragging.index].y = dragging.startCornerY + dy / scaleY;
  positionHandles();
  updateCornerInfo();
}, {passive: false});

document.addEventListener('touchend', function() {
  if (!dragging) return;
  dragging = null;
  sendCorners();
});

handles.forEach(function(h, i) {
  h.addEventListener('touchstart', function(e) {
    e.preventDefault();
    var t = e.touches[0];
    dragging = {index: i, startMouseX: t.clientX, startMouseY: t.clientY,
                startCornerX: corners[i].x, startCornerY: corners[i].y};
  }, {passive: false});
});

function updateCornerInfo() {
  var txt = corners.map(function(c, i){
    return CORNER_LABELS[i]+'('+Math.round(c.x)+','+Math.round(c.y)+')';
  }).join('  ');
  document.getElementById('corner-coords').textContent = 'Corners: ' + txt;
}

function sendCorners() {
  var payload = corners.map(function(c){ return [c.x, c.y]; });
  fetch('/api/corners', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({corners: payload})});
}

function saveCorners() { sendCorners(); }

function resetCorners() {
  corners = [{x:80,y:60},{x:560,y:60},{x:560,y:420},{x:80,y:420}];
  positionHandles();
  updateCornerInfo();
  sendCorners();
}

function loadCornersFromServer() {
  fetch('/api/corners').then(function(r){ return r.json(); }).then(function(d) {
    if (d.corners && d.corners.length === 4) {
      corners = d.corners.map(function(c){ return {x:c[0], y:c[1]}; });
      positionHandles();
      updateCornerInfo();
    }
  });
}

// Re-position handles whenever camera img loads (size may change)
document.getElementById('cam-img').addEventListener('load', function() {
  buildHandles();
  positionHandles();
});
window.addEventListener('resize', positionHandles);

// ── Parameter controls ──────────────────────────────────────────────────────
var curParams = {};
var GROUPS = [
  {t:'Piece Detection', p:[
    {k:'white_thresh',       l:'White threshold',      mn:100, mx:255, st:1},
    {k:'black_thresh',       l:'Black threshold',      mn:0,   mx:120, st:1},
    {k:'min_blob_area',      l:'Min blob area (px²)',  mn:10,  mx:1000,st:10},
    {k:'bottom_anchor_bias', l:'Bottom anchor bias',   mn:0,   mx:100, st:1, scale:100}
  ]},
  {t:'Camera Correction', p:[
    {k:'undistort_k1',    l:'k1 (×100)',       mn:-80, mx:80,  st:1},
    {k:'undistort_k2',    l:'k2 (×100)',       mn:-50, mx:50,  st:1},
    {k:'undistort_focal', l:'Focal % width',   mn:40,  mx:130, st:1}
  ], togs:[{k:'undistort_enable', l:'Enable undistortion'}]},
  {t:'Warp', p:[
    {k:'warp_size', l:'Warp size (px)', mn:240, mx:800, st:40}
  ]},
  {t:'Overlays', togs:[
    {k:'show_grid',       l:'Grid overlay'},
    {k:'show_pieces',     l:'Piece circles'},
    {k:'show_white_mask', l:'White mask debug'},
    {k:'show_black_mask', l:'Black mask debug'}
  ]}
];

function buildControls(p) {
  var grid = document.getElementById('ctrl-grid');
  grid.innerHTML = '';
  GROUPS.forEach(function(g) {
    var box = document.createElement('div');
    box.className = 'ctrl-group';
    var html = '<div class="ctrl-group-title">'+g.t+'</div>';
    (g.p||[]).forEach(function(pr) {
      // bottom_anchor_bias is stored 0..1 but slider shows 0..100
      var raw = p[pr.k] !== undefined ? p[pr.k] : 0;
      var display = pr.scale ? Math.round(raw * pr.scale) : raw;
      html += '<div class="slider-row"><label title="'+pr.k+'">'+pr.l+'</label>'+
              '<input type="range" id="s_'+pr.k+'" min="'+pr.mn+'" max="'+pr.mx+'" step="'+pr.st+'" value="'+display+'"'+
              ' oninput="document.getElementById(\'vb_'+pr.k+'\').textContent=this.value"'+
              ' onchange="onSlider(\''+pr.k+'\',+this.value,'+(pr.scale||1)+')">'+
              '<span class="vb" id="vb_'+pr.k+'">'+display+'</span></div>';
    });
    (g.togs||[]).forEach(function(t) {
      html += '<div class="tog-row"><input type="checkbox" id="t_'+t.k+'" '+(p[t.k]?'checked':'')+
              ' onchange="onTog(\''+t.k+'\',this.checked)"><label for="t_'+t.k+'">'+t.l+'</label></div>';
    });
    box.innerHTML = html;
    grid.appendChild(box);
  });
}

function onSlider(k, v, scale) {
  var real = scale > 1 ? v / scale : v;
  curParams[k] = real;
  sendParam({[k]: real});
}
function onTog(k, c) { curParams[k] = c?1:0; sendParam({[k]:c?1:0}); }
function sendParam(d) {
  fetch('/api/params', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify(d)});
}
function forceReprocess() { sendParam({}); }
function injectFen() {
  var f = document.getElementById('inj-in').value.trim();
  fetch('/api/fen', {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({fen: f})});
}

// ── Image refresh ────────────────────────────────────────────────────────────
function refreshImages() {
  var t = Date.now();
  document.getElementById('cam-img').src  = '/api/frame?t='+t;
  document.getElementById('warp-img').src = '/api/warp_frame?t='+t;
  document.getElementById('cam-ts').textContent = new Date().toLocaleTimeString();
}

// ── FEN poll ─────────────────────────────────────────────────────────────────
function pollFen() {
  fetch('/api/fen').then(function(r){return r.json();}).then(function(d) {
    try { chessboard.position(d.fen.split(' ')[0], false); } catch(e) {}
    document.getElementById('fen-text').textContent = d.fen;
    document.getElementById('fc').textContent = d.frame_count;
    var age = d.last_updated > 0 ? (Date.now()/1000 - d.last_updated) : 9999;
    var ae = document.getElementById('fen-age');
    ae.textContent = age < 9999 ? age.toFixed(1) : 'never';
    ae.className = age < 4 ? 'ok' : age < 15 ? 'warn' : 'err';
    var dot = document.getElementById('status-dot');
    var cs  = document.getElementById('conn-stat');
    if (age < 4)  { dot.className='live'; cs.textContent='Live ✓'; cs.className='val ok'; }
    else if (age < 30) { dot.className=''; cs.textContent='Stale'; cs.className='val warn'; }
    else { dot.className='dead'; cs.textContent='No data'; cs.className='val err'; }
    document.getElementById('err-msg').textContent = d.error || '–';
    if (d.cam_info) document.getElementById('cam-info').textContent = d.cam_info;
  }).catch(function(){ document.getElementById('status-dot').className='dead'; });
}

// ── Boot ──────────────────────────────────────────────────────────────────────
fetch('/api/params').then(function(r){return r.json();}).then(function(p){
  curParams = p; buildControls(p);
});
buildHandles();
loadCornersFromServer();
updateCornerInfo();
setInterval(refreshImages, 1500);
setInterval(pollFen, 1000);
refreshImages(); pollFen();
</script>
</body>
</html>'''


# ─── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",   type=int, default=5000)
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--fen",    default=None)
    parser.add_argument("--image",  default=None, help="Static image for offline test")
    args = parser.parse_args()

    _load_corners()

    if args.fen:
        with _lock:
            _state["fen"] = args.fen
            _state["last_updated"] = time.time()
            _state["frame_count"]  = 1

    if args.image:
        frame = cv2.imread(args.image)
        if frame is not None:
            with _lock:
                _state["raw_frame"] = frame
            print(f"✓ Loaded static image: {args.image}  {frame.shape[1]}×{frame.shape[0]}")
        else:
            print(f"⚠  Could not load: {args.image}")

    threading.Thread(target=_processing_loop, daemon=True).start()

    if not args.no_ros:
        if not ROS_AVAILABLE:
            print("ERROR: rclpy not found. Use --no-ros"); sys.exit(1)
        rclpy.init()
        ros_node = FenNode()
        threading.Thread(target=_ros_thread_fn, args=(ros_node,), daemon=True).start()
        print("✓ ROS subscriber started")
    else:
        print("⚠  No-ROS mode — use --image <path> or drag corners in browser")

    print(f"\nOpen browser: http://localhost:{args.port}")
    print(f"Network URL:  http://<pi-ip>:{args.port}\n")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()

