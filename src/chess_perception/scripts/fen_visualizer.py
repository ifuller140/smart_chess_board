#!/usr/bin/env python3
"""
FEN Live Board Visualizer — Full Game Dashboard
================================================
Live dashboard showing camera feed, warped board, chess board state with
actual piece types, and game manager state (when ROS is running).

Board corners are manually defined (draggable in the browser). The chess
board display uses the authoritative game FEN from game_manager_node when
connected, falling back to the perception pipeline FEN, then local detection.

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

_DEFAULT_CORNERS = [
    [0.10, 0.10],   # TL
    [0.90, 0.10],   # TR
    [0.90, 0.90],   # BR
    [0.10, 0.90],   # BL
]

_state = {
    # Vision / detection
    "detected_fen":   "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "last_move":      "",    # last detected changed squares e.g. "e2,e4"
    "frame_count":    0,
    "last_updated":   0.0,
    "error":          "",
    "raw_frame":      None,
    "cam_info":       "–",
    "corners":        [list(c) for c in _DEFAULT_CORNERS],
    "pieces64":       [""] * 64,
    # Game manager state (updated via ROS)
    "game_fen":       "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "game_state":     "OFFLINE",
    "game_turn":      "",
    "fen_source":     "local",   # "game_mgr" | "local"
}

# Tunable detection params
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


def _best_fen():
    """Return the most authoritative FEN available."""
    src = _state.get("fen_source", "local")
    if src == "game_mgr" and _state.get("game_fen"):
        return _state["game_fen"]
    return _state.get("detected_fen", "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")


# ─── Lens undistortion ────────────────────────────────────────────────────────
def _undistort_frame(frame, p):
    if not p.get("undistort_enable", 0):
        return frame
    h, w = frame.shape[:2]
    f = p["undistort_focal"] / 100.0 * w
    K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]], dtype=np.float64)
    D = np.array([p["undistort_k1"] / 100.0, p["undistort_k2"] / 100.0, 0.0, 0.0], dtype=np.float64)
    return cv2.undistort(frame, K, D)


# ─── Perspective warp ─────────────────────────────────────────────────────────
def _corners_to_px(corners_norm, frame):
    h, w = frame.shape[:2]
    return [[c[0] * w, c[1] * h] for c in corners_norm]


def _get_warp_matrix(corners_px, warp_size):
    src = np.float32(corners_px)
    sz = warp_size - 1
    dst = np.float32([[0, 0], [sz, 0], [sz, sz], [0, sz]])
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    return M, M_inv


def _warp_frame(frame, corners_norm, warp_size):
    corners_px = _corners_to_px(corners_norm, frame)
    M, M_inv = _get_warp_matrix(corners_px, warp_size)
    warped = cv2.warpPerspective(frame, M, (warp_size, warp_size))
    return warped, M, M_inv, corners_px


# ─── Piece blob detection ─────────────────────────────────────────────────────
def _make_piece_masks(warped_gray, p):
    white_mask = cv2.threshold(warped_gray, p["white_thresh"], 255, cv2.THRESH_BINARY)[1]
    black_mask = cv2.threshold(warped_gray, p["black_thresh"], 255, cv2.THRESH_BINARY_INV)[1]
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    white_mask = cv2.morphologyEx(white_mask, cv2.MORPH_OPEN, k)
    black_mask = cv2.morphologyEx(black_mask, cv2.MORPH_OPEN, k)
    return white_mask, black_mask


def _assign_blob_to_square(blob_mask, sq_size, p, piece_color):
    contours, _ = cv2.findContours(blob_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    assignments = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < p["min_blob_area"]:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        M_cnt = cv2.moments(cnt)
        if M_cnt["m00"] == 0:
            continue
        cx = int(M_cnt["m10"] / M_cnt["m00"])
        cy_center = int(M_cnt["m01"] / M_cnt["m00"])
        cy_bottom = y + h
        bias = p["bottom_anchor_bias"]
        cy_anchor = int(cy_center * (1.0 - bias) + cy_bottom * bias)
        cy_anchor = min(cy_anchor, sq_size * 8 - 1)
        cx = min(max(cx, 0), sq_size * 8 - 1)
        row = cy_anchor // sq_size
        col = cx // sq_size
        assignments.append((row, col, area))
    return assignments


def _detect_pieces(warped, p):
    sq = p["warp_size"] // 8
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    white_mask, black_mask = _make_piece_masks(gray, p)
    best = {}
    for color_char, mask in [('W', white_mask), ('B', black_mask)]:
        for row, col, area in _assign_blob_to_square(mask, sq, p, color_char):
            key = (row, col)
            if key not in best or area > best[key][0]:
                best[key] = (area, color_char)
    pieces64 = [""] * 64
    for (row, col), (area, color_char) in best.items():
        if 0 <= row < 8 and 0 <= col < 8:
            pieces64[row * 8 + col] = color_char
    return pieces64, white_mask, black_mask


# ─── FEN builder (local/offline detection only — produces P/p) ────────────────
def _build_fen_local(pieces64):
    """
    Builds a FEN from local color detection. Uses generic 'P'/'p' symbols
    because color thresholding cannot distinguish piece types.
    The game_manager's authoritative FEN (game_fen) is used for the chess
    board display when ROS is connected.
    """
    rows = []
    for rank_i in range(7, -1, -1):
        warp_row = 7 - rank_i
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
                row_str += 'P' if pc == 'W' else 'p'
        if empty:
            row_str += str(empty)
        rows.append(row_str)
    return '/'.join(rows) + ' w KQkq - 0 1'


# ─── Render helpers ───────────────────────────────────────────────────────────
def _draw_grid(img, sq):
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


def _render_raw(frame, corners, p, pieces64, M_inv):
    out = frame.copy()
    sz = p["warp_size"]
    sq = sz // 8
    if p["show_grid"] and M_inv is not None:
        for i in range(9):
            p0w = np.float32([[[i * sq, 0]]])
            p1w = np.float32([[[i * sq, sz]]])
            p0 = cv2.perspectiveTransform(p0w, M_inv)[0, 0].astype(int)
            p1 = cv2.perspectiveTransform(p1w, M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(p0), tuple(p1), (0, 200, 200), 1)
            q0w = np.float32([[[0, i * sq]]])
            q1w = np.float32([[[sz, i * sq]]])
            q0 = cv2.perspectiveTransform(q0w, M_inv)[0, 0].astype(int)
            q1 = cv2.perspectiveTransform(q1w, M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(q0), tuple(q1), (0, 200, 200), 1)
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
                    cv2.putText(out, pc, tuple(pt), cv2.FONT_HERSHEY_SIMPLEX, 0.7, border, 4)
                    cv2.putText(out, pc, tuple(pt), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
    labels = ['TL', 'TR', 'BR', 'BL']
    handle_colors = [(255, 80, 80), (80, 255, 80), (80, 80, 255), (255, 255, 80)]
    for i, (cx, cy) in enumerate(corners):
        cv2.circle(out, (int(cx), int(cy)), 10, handle_colors[i], -1)
        cv2.circle(out, (int(cx), int(cy)), 10, (255, 255, 255), 1)
        cv2.putText(out, labels[i], (int(cx) + 12, int(cy) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, handle_colors[i], 2)
    pts = np.array(corners, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, (0, 255, 100), 2)
    return out


def _render_warp(warped, p, pieces64, white_mask, black_mask):
    out = warped.copy()
    sz = p["warp_size"]
    sq = sz // 8
    for r in range(8):
        for c in range(8):
            if (r + c) % 2 == 0:
                roi = out[r*sq:(r+1)*sq, c*sq:(c+1)*sq].astype(np.float32)
                roi = cv2.addWeighted(roi, 0.82, np.full_like(roi, 190), 0.18, 0)
                out[r*sq:(r+1)*sq, c*sq:(c+1)*sq] = roi.astype(np.uint8)
    if p["show_grid"]:
        _draw_grid(out, sq)
        _draw_rank_file_labels(out, sq)
    if p["show_white_mask"] and white_mask is not None:
        overlay = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 0] = 0; overlay[:, :, 1] = 0
        out = cv2.addWeighted(out, 0.7, overlay, 0.3, 0)
    if p["show_black_mask"] and black_mask is not None:
        overlay = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2BGR)
        overlay[:, :, 1] = 0; overlay[:, :, 2] = 0
        out = cv2.addWeighted(out, 0.7, overlay, 0.3, 0)
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
                    cv2.putText(out, pc, (cx - 8, cy + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, tc, 2)
    cv2.rectangle(out, (0, 0), (sz - 1, sz - 1), (0, 255, 100), 2)
    return out


# ─── Background processing ────────────────────────────────────────────────────
_processed = {"raw": None, "warp": None}
_proc_lock = threading.Lock()


def _process_frame(frame, corners, p):
    try:
        warped, M, M_inv, corners_px = _warp_frame(frame, corners, p["warp_size"])
        pieces64, white_mask, black_mask = _detect_pieces(warped, p)
        fen = _build_fen_local(pieces64)

        raw_img  = _render_raw(frame, corners_px, p, pieces64, M_inv)
        warp_img = _render_warp(warped, p, pieces64, white_mask, black_mask)

        with _lock:
            _state["pieces64"]      = pieces64
            _state["detected_fen"]  = fen
            _state["frame_count"]  += 1
            _state["last_updated"]  = time.time()
            _state["error"]         = ""
            # Only promote detected_fen if no better source is available
            if _state["fen_source"] == "local":
                pass  # detected_fen is already the fallback in _best_fen()

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

        # Changed squares from piece_detector_node (frame-diff detection)
        self.create_subscription(String, "/perception/changed_squares",
                                 self._on_changed_squares, 10)

        # Camera image for local processing
        self.create_subscription(Image, "/camera/image_raw", self._on_img, 10)

        # Game manager topics — authoritative game state
        self.create_subscription(String, "/game_manager/board_fen",
                                 self._on_game_fen, 10)
        self.create_subscription(String, "/game_manager/state",
                                 self._on_game_state, 10)
        self.create_subscription(String, "/game_manager/turn",
                                 self._on_game_turn, 10)

        self.get_logger().info("FenNode ready — subscribed to perception + game manager topics")

    def _on_changed_squares(self, msg):
        raw = msg.data.strip()
        with _lock:
            _state["last_move"]    = raw
            _state["last_updated"] = time.time()
            _state["frame_count"] += 1

    def _on_game_fen(self, msg):
        fen = msg.data.strip()
        if fen:
            with _lock:
                _state["game_fen"]     = fen
                _state["fen_source"]   = "game_mgr"
                _state["last_updated"] = time.time()
                _state["frame_count"] += 1

    def _on_game_state(self, msg):
        with _lock:
            _state["game_state"] = msg.data.strip()

    def _on_game_turn(self, msg):
        with _lock:
            _state["game_turn"] = msg.data.strip()

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
        fen = _best_fen()
        return jsonify({
            "fen":          fen,
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
        _state["game_fen"]     = fen
        _state["fen_source"]   = "local"
        _state["last_updated"] = time.time()
        _state["frame_count"] += 1
    return jsonify({"ok": True})


@app.route("/api/game_state")
def api_game_state():
    with _lock:
        return jsonify({
            "state":     _state.get("game_state", "OFFLINE"),
            "turn":      _state.get("game_turn", ""),
            "source":    _state.get("fen_source", "local"),
            "fen":       _best_fen(),
            "last_move": _state.get("last_move", ""),
        })


@app.route("/api/frame")
def api_frame():
    with _proc_lock:
        data = _processed.get("raw")
    if not data:
        data = _blank_jpeg(640, 360, "Waiting for camera...")
    return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/warp_frame")
def api_warp_frame():
    with _proc_lock:
        data = _processed.get("warp")
    if not data:
        data = _blank_jpeg(480, 480, "No corners set", (80, 80, 80))
    return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


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
    try:
        corners = [[float(c[0]), float(c[1])] for c in corners]
        for fx, fy in corners:
            if not (0.0 <= fx <= 1.0 and 0.0 <= fy <= 1.0):
                return jsonify({"ok": False, "msg": "Fractions must be 0..1"}), 400
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


# ─── HTML Template ─────────────────────────────────────────────────────────────
HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>Smart Chess Board — Live Dashboard</title>
<link rel="stylesheet" href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
<script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
<script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
<style>
/* ── Reset & base ───────────────────────────────────────────────── */
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px;min-height:100vh}

/* ── Header ─────────────────────────────────────────────────────── */
.header{background:#161b22;border-bottom:1px solid #21262d;padding:8px 16px;display:flex;align-items:center;gap:10px;flex-wrap:wrap}
.header h1{color:#58a6ff;font-size:1.0em;font-weight:600;display:flex;align-items:center;gap:8px;margin:0}
#status-dot{width:10px;height:10px;border-radius:50%;background:#d29922;flex-shrink:0;transition:background .3s}
#status-dot.live{background:#3fb950;animation:pulse 2s infinite}
#status-dot.dead{background:#f85149}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.35}}
.header-actions{margin-left:auto;display:flex;gap:6px;flex-wrap:wrap}

/* ── Layout grid ─────────────────────────────────────────────────── */
.layout{display:grid;grid-template-columns:1fr 380px 300px;grid-template-rows:auto auto;gap:8px;padding:8px;align-items:start}

/* ── Generic card ────────────────────────────────────────────────── */
.card{background:#161b22;border:1px solid #21262d;border-radius:6px;overflow:hidden}
.card-header{font-size:.65em;text-transform:uppercase;letter-spacing:.09em;color:#8b949e;padding:6px 10px;border-bottom:1px solid #21262d;display:flex;justify-content:space-between;align-items:center;font-weight:600}
.card-body{padding:8px 10px}

/* ── Camera panel ────────────────────────────────────────────────── */
.cam-panel{grid-column:1;grid-row:1}
#cam-canvas-wrap{position:relative;width:100%}
#cam-img{width:100%;display:block;cursor:crosshair}
.corner-info{font-size:.65em;color:#8b949e;padding:4px 10px 6px}
.handle{cursor:grab;position:absolute;width:20px;height:20px;border-radius:50%;transform:translate(-50%,-50%);border:2px solid white;display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:bold;color:#fff;user-select:none;z-index:10}
.handle:active{cursor:grabbing}

/* ── Warp panel ──────────────────────────────────────────────────── */
.warp-panel{grid-column:2;grid-row:1}
.warp-panel img{width:100%;display:block;image-rendering:pixelated}

/* ── Right column ────────────────────────────────────────────────── */
.right-col{grid-column:3;grid-row:1/3;display:flex;flex-direction:column;gap:8px}

/* ── Stat rows ───────────────────────────────────────────────────── */
.stat-row{display:flex;justify-content:space-between;align-items:center;padding:4px 0;border-bottom:1px solid #21262d;font-size:.75em;gap:6px}
.stat-row:last-child{border-bottom:none}
.stat-lbl{color:#8b949e;white-space:nowrap;flex-shrink:0}
.stat-val{font-family:monospace;text-align:right;word-break:break-all}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}

/* ── Game state badges ───────────────────────────────────────────── */
.state-badge{padding:2px 7px;border-radius:3px;font-size:.65em;font-weight:700;text-transform:uppercase;letter-spacing:.05em;white-space:nowrap}
.s-offline,.s-startup     {background:#21262d;color:#8b949e}
.s-homing                 {background:#9e6a03;color:#fff}
.s-idle                   {background:#1c2742;color:#79b8ff}
.s-waiting_player_move    {background:#1f6feb;color:#fff}
.s-capturing_board,.s-validating_move{background:#6e40c9;color:#fff}
.s-calculating_response   {background:#d29922;color:#0d1117}
.s-executing_move,.s-hitting_clock{background:#e16f24;color:#fff}
.s-promotion_wait         {background:#8957e5;color:#fff}
.s-game_over              {background:#b91c1c;color:#fff}

.turn-white{color:#f0f0f0;font-weight:700}
.turn-black{color:#8b949e;font-weight:700}

.src-badge{padding:1px 5px;border-radius:3px;font-size:.63em;font-weight:600}
.src-game{background:#196127;color:#fff}
.src-perc{background:#1c2742;color:#79b8ff}
.src-local{background:#21262d;color:#8b949e}

/* ── Chess board ─────────────────────────────────────────────────── */
#board-wrap{padding:8px;display:flex;justify-content:center}
#board{width:256px}
#fen-text{font-family:monospace;font-size:.6em;color:#3fb950;word-break:break-all;background:#0d1117;border:1px solid #21262d;border-radius:4px;padding:5px;margin:0 8px 8px}

/* ── Controls ─────────────────────────────────────────────────────── */
.ctrl-panel{grid-column:1/3;grid-row:2;background:#161b22;border:1px solid #21262d;border-radius:6px;padding:12px 14px}
.ctrl-panel-header{font-size:.65em;text-transform:uppercase;letter-spacing:.09em;color:#8b949e;font-weight:600;margin-bottom:10px}
.ctrl-grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px 16px}
.ctrl-group{background:#0d1117;border:1px solid #21262d;border-radius:5px;padding:10px 12px}
.ctrl-group-title{font-size:.63em;text-transform:uppercase;letter-spacing:.08em;color:#58a6ff;margin-bottom:8px;font-weight:700;border-bottom:1px solid #21262d;padding-bottom:5px}
.slider-row{display:flex;align-items:center;gap:6px;margin:5px 0}
.slider-row label{flex:0 0 140px;font-size:.7em;color:#8b949e;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.slider-row input[type=range]{flex:1;accent-color:#58a6ff;height:4px}
.slider-row .vb{flex:0 0 38px;text-align:right;font-family:monospace;font-size:.7em;color:#c9d1d9}
.tog-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.tog-row label{font-size:.7em;color:#8b949e;cursor:pointer}
.tog-row input[type=checkbox]{accent-color:#58a6ff;width:13px;height:13px;cursor:pointer}

/* ── Buttons ─────────────────────────────────────────────────────── */
.btn{background:#21262d;color:#c9d1d9;border:1px solid #30363d;padding:4px 10px;border-radius:4px;cursor:pointer;font-size:.73em;text-decoration:none;display:inline-block;white-space:nowrap}
.btn:hover{background:#30363d}
.btn.blue{background:#1f6feb;color:#fff;border-color:#1f6feb}
.btn.blue:hover{background:#388bfd}
.btn.green{background:#196127;color:#fff;border-color:#196127}
.btn.green:hover{background:#238636}

/* ── FEN inject ──────────────────────────────────────────────────── */
.fen-inject-row{display:flex;gap:6px;padding:8px 10px}
.fen-inject-row input{flex:1;background:#0d1117;color:#c9d1d9;border:1px solid #21262d;border-radius:4px;padding:4px 7px;font-family:monospace;font-size:.68em}
</style>
</head>
<body>

<div class="header">
  <h1><span id="status-dot"></span>♟ Smart Chess Board — Live Dashboard</h1>
  <div class="header-actions">
    <button class="btn blue" onclick="forceReprocess()">⟳ Reprocess</button>
    <button class="btn green" onclick="saveCorners()">📌 Save Corners</button>
    <button class="btn" onclick="resetCorners()">↺ Reset</button>
    <a href="/api/snapshot" class="btn">📷 Snapshot</a>
  </div>
</div>

<div class="layout">

  <!-- ── Camera + draggable corners ─────────────────────────────── -->
  <div class="card cam-panel">
    <div class="card-header">
      Live Camera — drag corners to define board
      <span id="cam-ts" class="warn" style="font-size:.9em;text-transform:none;letter-spacing:0"></span>
    </div>
    <div id="cam-canvas-wrap">
      <img id="cam-img" src="/api/frame" alt="camera" draggable="false">
    </div>
    <div class="corner-info" id="corner-coords">Corners: loading…</div>
  </div>

  <!-- ── Top-down warped board ───────────────────────────────────── -->
  <div class="card warp-panel">
    <div class="card-header">Top-Down Board View</div>
    <img id="warp-img" src="/api/warp_frame" alt="warp">
  </div>

  <!-- ── Right column ────────────────────────────────────────────── -->
  <div class="right-col">

    <!-- Game State -->
    <div class="card">
      <div class="card-header">Game State</div>
      <div class="card-body">
        <div class="stat-row">
          <span class="stat-lbl">State</span>
          <span class="stat-val"><span id="gs-state" class="state-badge s-offline">OFFLINE</span></span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Turn</span>
          <span class="stat-val" id="gs-turn">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Full Move</span>
          <span class="stat-val" id="gs-fullmove">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Last Move Squares</span>
          <span class="stat-val" id="gs-lastmove" style="font-family:monospace;color:#58a6ff">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">FEN Source</span>
          <span class="stat-val"><span id="gs-source" class="src-badge src-local">Local</span></span>
        </div>
      </div>
    </div>

    <!-- Connection Status -->
    <div class="card">
      <div class="card-header">Connection</div>
      <div class="card-body">
        <div class="stat-row">
          <span class="stat-lbl">Feed</span>
          <span class="stat-val" id="conn-stat">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Camera</span>
          <span class="stat-val" id="cam-info">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Frames</span>
          <span class="stat-val" id="fc">0</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">FEN age</span>
          <span class="stat-val"><span id="fen-age">–</span>s</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Error</span>
          <span class="stat-val err" id="err-msg" style="font-size:.63em">–</span>
        </div>
      </div>
    </div>

    <!-- Chess Board -->
    <div class="card">
      <div class="card-header">Board State</div>
      <div id="board-wrap"><div id="board"></div></div>
      <div id="fen-text">–</div>
    </div>

    <!-- Inject FEN -->
    <div class="card">
      <div class="card-header">Inject FEN (offline test)</div>
      <div class="fen-inject-row">
        <input id="inj-in" type="text"
          value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
        <button class="btn blue" onclick="injectFen()">Send</button>
      </div>
    </div>

  </div><!-- /right-col -->

  <!-- ── Detection Parameters ────────────────────────────────────── -->
  <div class="ctrl-panel">
    <div class="ctrl-panel-header">Detection Parameters</div>
    <div class="ctrl-grid" id="ctrl-grid"></div>
  </div>

</div><!-- /layout -->

<script>
// ── Chess board ──────────────────────────────────────────────────────────────
var chessboard = Chessboard('board', {
  position: 'start',
  showNotation: true,
  pieceTheme: 'https://lichess1.org/assets/piece/cburnett/{piece}.svg'
});

// ── Corner management ────────────────────────────────────────────────────────
var corners = [{fx:0.10,fy:0.10},{fx:0.90,fy:0.10},{fx:0.90,fy:0.90},{fx:0.10,fy:0.90}];
var CORNER_LABELS  = ['TL','TR','BR','BL'];
var CORNER_COLORS  = ['#ff5050','#50ff50','#5050ff','#ffff50'];
var handles = [];
var dragging = null;

function positionHandles() {
  var img = document.getElementById('cam-img');
  var W = img.clientWidth, H = img.clientHeight;
  corners.forEach(function(c, i) {
    handles[i].style.left = (c.fx * W) + 'px';
    handles[i].style.top  = (c.fy * H) + 'px';
  });
}

function buildHandles() {
  var wrap = document.getElementById('cam-canvas-wrap');
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
      dragging = {index:i, startMouseX:e.clientX, startMouseY:e.clientY,
                  startFx:corners[i].fx, startFy:corners[i].fy};
    });
    el.addEventListener('touchstart', function(e) {
      e.preventDefault();
      var t = e.touches[0];
      dragging = {index:i, startMouseX:t.clientX, startMouseY:t.clientY,
                  startFx:corners[i].fx, startFy:corners[i].fy};
    }, {passive:false});
  });
  positionHandles();
}

function moveDragging(clientX, clientY) {
  if (!dragging) return;
  var img = document.getElementById('cam-img');
  var W = img.clientWidth, H = img.clientHeight;
  corners[dragging.index].fx = Math.min(1, Math.max(0,
      dragging.startFx + (clientX - dragging.startMouseX) / W));
  corners[dragging.index].fy = Math.min(1, Math.max(0,
      dragging.startFy + (clientY - dragging.startMouseY) / H));
  positionHandles();
  updateCornerInfo();
}

document.addEventListener('mousemove', function(e){ moveDragging(e.clientX, e.clientY); });
document.addEventListener('mouseup',   function()  { if(dragging){ dragging=null; sendCorners(); } });
document.addEventListener('touchmove', function(e){
  if(!dragging) return; e.preventDefault();
  moveDragging(e.touches[0].clientX, e.touches[0].clientY);
}, {passive:false});
document.addEventListener('touchend', function(){ if(dragging){ dragging=null; sendCorners(); } });

function updateCornerInfo() {
  var txt = corners.map(function(c,i){
    return CORNER_LABELS[i]+'('+c.fx.toFixed(3)+','+c.fy.toFixed(3)+')';
  }).join('  ');
  document.getElementById('corner-coords').textContent = 'Corners (normalized): '+txt;
}

function sendCorners() {
  var payload = corners.map(function(c){ return [c.fx, c.fy]; });
  fetch('/api/corners',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({corners:payload})});
}
function saveCorners()  { sendCorners(); }
function resetCorners() {
  corners=[{fx:0.10,fy:0.10},{fx:0.90,fy:0.10},{fx:0.90,fy:0.90},{fx:0.10,fy:0.90}];
  positionHandles(); updateCornerInfo(); sendCorners();
}
function loadCornersFromServer() {
  fetch('/api/corners').then(function(r){return r.json();}).then(function(d){
    if(d.corners && d.corners.length===4){
      corners = d.corners.map(function(c){return {fx:c[0],fy:c[1]};});
      positionHandles(); updateCornerInfo();
    }
  });
}

document.getElementById('cam-img').addEventListener('load', function(){ buildHandles(); positionHandles(); });
window.addEventListener('resize', positionHandles);

// ── Parameter controls ───────────────────────────────────────────────────────
var curParams = {};
var GROUPS = [
  {t:'Piece Detection', p:[
    {k:'white_thresh',        l:'White threshold',      mn:100, mx:255, st:1},
    {k:'black_thresh',        l:'Black threshold',      mn:0,   mx:120, st:1},
    {k:'min_blob_area',       l:'Min blob area (px²)',  mn:10,  mx:1000,st:10},
    {k:'bottom_anchor_bias',  l:'Bottom-anchor bias',   mn:0,   mx:100, st:1, scale:100}
  ]},
  {t:'Lens Correction', p:[
    {k:'undistort_k1',    l:'k1 (×100)',       mn:-80, mx:80,  st:1},
    {k:'undistort_k2',    l:'k2 (×100)',       mn:-50, mx:50,  st:1},
    {k:'undistort_focal', l:'Focal % width',   mn:40,  mx:130, st:1}
  ], togs:[{k:'undistort_enable', l:'Enable lens undistortion'}]},
  {t:'Transform', p:[
    {k:'warp_size', l:'Warp output size (px)', mn:240, mx:800, st:40}
  ]},
  {t:'Overlay Options', togs:[
    {k:'show_grid',       l:'Show grid overlay'},
    {k:'show_pieces',     l:'Show piece circles'},
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
      var raw = p[pr.k] !== undefined ? p[pr.k] : 0;
      var display = pr.scale ? Math.round(raw * pr.scale) : raw;
      html += '<div class="slider-row">'+
              '<label title="'+pr.k+'">'+pr.l+'</label>'+
              '<input type="range" id="s_'+pr.k+'" min="'+pr.mn+'" max="'+pr.mx+
              '" step="'+pr.st+'" value="'+display+'"'+
              ' oninput="document.getElementById(\'vb_'+pr.k+'\').textContent=this.value"'+
              ' onchange="onSlider(\''+pr.k+'\',+this.value,'+(pr.scale||1)+')">'+
              '<span class="vb" id="vb_'+pr.k+'">'+display+'</span></div>';
    });
    (g.togs||[]).forEach(function(t) {
      html += '<div class="tog-row">'+
              '<input type="checkbox" id="t_'+t.k+'" '+(p[t.k]?'checked':'')+
              ' onchange="onTog(\''+t.k+'\',this.checked)">'+
              '<label for="t_'+t.k+'">'+t.l+'</label></div>';
    });
    box.innerHTML = html;
    grid.appendChild(box);
  });
}

function onSlider(k, v, scale) { var real = scale>1 ? v/scale : v; curParams[k]=real; sendParam({[k]:real}); }
function onTog(k, c) { curParams[k]=c?1:0; sendParam({[k]:c?1:0}); }
function sendParam(d) {
  fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d)});
}
function forceReprocess() { sendParam({}); }
function injectFen() {
  var f = document.getElementById('inj-in').value.trim();
  fetch('/api/fen',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({fen:f})});
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
  fetch('/api/fen').then(function(r){return r.json();}).then(function(d){
    var fen = d.fen || '';
    try {
      if (fen) chessboard.position(fen.split(' ')[0], false);
      var parts = fen.split(' ');
      if (parts.length >= 6) {
        document.getElementById('gs-fullmove').textContent = 'Move '+parts[5];
      }
    } catch(e) {}
    document.getElementById('fen-text').textContent = fen;
    document.getElementById('fc').textContent = d.frame_count;

    var age = d.last_updated > 0 ? (Date.now()/1000 - d.last_updated) : 9999;
    var ae = document.getElementById('fen-age');
    ae.textContent = age < 9999 ? age.toFixed(1) : 'never';
    ae.className = 'stat-val '+(age<4?'ok':age<15?'warn':'err');

    var dot = document.getElementById('status-dot');
    var cs  = document.getElementById('conn-stat');
    if (age < 4)  { dot.className='live'; cs.textContent='Live ✓'; cs.className='stat-val ok'; }
    else if (age < 30){ dot.className=''; cs.textContent='Stale'; cs.className='stat-val warn'; }
    else { dot.className='dead'; cs.textContent='No data'; cs.className='stat-val err'; }

    document.getElementById('err-msg').textContent = d.error || '–';
    if (d.cam_info) document.getElementById('cam-info').textContent = d.cam_info;
  }).catch(function(){ document.getElementById('status-dot').className='dead'; });
}

// ── Game state poll ───────────────────────────────────────────────────────────
var STATE_LABEL = {
  'OFFLINE':'Offline','STARTUP':'Startup','HOMING':'Homing',
  'IDLE':'Idle — Ready','WAITING_PLAYER_MOVE':'Waiting Player',
  'CAPTURING_BOARD':'Capturing','VALIDATING_MOVE':'Validating',
  'CALCULATING_RESPONSE':'Thinking','EXECUTING_MOVE':'Executing',
  'HITTING_CLOCK':'Hitting Clock','PROMOTION_WAIT':'Promotion Wait',
  'GAME_OVER':'Game Over'
};

function pollGameState() {
  fetch('/api/game_state').then(function(r){return r.json();}).then(function(d){
    // State badge
    var s = (d.state || 'OFFLINE').toUpperCase();
    var stEl = document.getElementById('gs-state');
    stEl.textContent = STATE_LABEL[s] || s.replace(/_/g,' ');
    stEl.className   = 'state-badge s-'+s.toLowerCase();

    // Turn
    var turnEl = document.getElementById('gs-turn');
    var t = (d.turn || '').toUpperCase();
    if      (t === 'WHITE'){ turnEl.textContent='⬜ White'; turnEl.className='stat-val turn-white'; }
    else if (t === 'BLACK'){ turnEl.textContent='⬛ Black'; turnEl.className='stat-val turn-black'; }
    else { turnEl.textContent='–'; turnEl.className='stat-val'; }

    // Last detected changed squares
    var lmEl = document.getElementById('gs-lastmove');
    if (d.last_move) {
      lmEl.textContent = d.last_move.split(',').join(' → ');
    } else {
      lmEl.textContent = '–';
    }

    // FEN source
    var srcEl = document.getElementById('gs-source');
    if (d.source === 'game_mgr') { srcEl.textContent='Game Manager ✓'; srcEl.className='src-badge src-game'; }
    else                         { srcEl.textContent='Local (offline)'; srcEl.className='src-badge src-local'; }
  }).catch(function(){
    var el = document.getElementById('gs-state');
    el.textContent = 'Offline'; el.className = 'state-badge s-offline';
  });
}

// ── Boot ──────────────────────────────────────────────────────────────────────
fetch('/api/params').then(function(r){return r.json();}).then(function(p){
  curParams = p; buildControls(p);
});
buildHandles();
loadCornersFromServer();
updateCornerInfo();
setInterval(refreshImages,  1500);
setInterval(pollFen,        1000);
setInterval(pollGameState,  1000);
refreshImages();
pollFen();
pollGameState();
</script>
</body>
</html>'''


# ─── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",   type=int, default=5000)
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true")
    parser.add_argument("--fen",    default=None, help="Initial FEN to display")
    parser.add_argument("--image",  default=None, help="Static image for offline test")
    args = parser.parse_args()

    _load_corners()

    if args.fen:
        with _lock:
            _state["game_fen"]     = args.fen
            _state["fen_source"]   = "local"
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
        print("  Subscribing to: /perception/changed_squares, /camera/image_raw")
        print("  Subscribing to: /game_manager/board_fen, /game_manager/state, /game_manager/turn")
    else:
        print("⚠  No-ROS mode — use --image <path> or drag corners in browser")

    print(f"\nOpen browser: http://localhost:{args.port}")
    print(f"Network URL:  http://<pi-ip>:{args.port}\n")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
