#!/usr/bin/env python3
"""
Chess Board Debug Dashboard
============================
Tabbed web UI for diagnosing the smart chess board perception pipeline.

Tabs:
  📷 Live     — Raw unprocessed camera feed
  📐 Corners  — Drag board corners, tune lens correction, preview warp
  🔍 Diff     — Per-square diff heatmap, score grid, threshold tuning
  ♟  Game     — Chess board diagram, game state machine, FEN management

Always-visible header actions:
  CLOCK HIT     — Publishes Bool(True) to /limit_switch/clock_hit
                  Simulates the player pressing the chess clock → triggers move detection
  Capture Ref   — Calls /perception/capture_premove to snapshot current board as reference
  Save Corners  — Persists corner positions to board_corners.json
  Snapshot      — Downloads a full-resolution camera image

How the perception pipeline works (frame-diff approach):
  1. game_manager enters WAITING_PLAYER_MOVE → calls /perception/capture_premove
     piece_detector stores the current warped board as the pre-move reference.
  2. Human moves a piece, then presses the clock.
  3. game_manager receives the clock hit → transitions to CAPTURING_BOARD.
  4. piece_detector compares every incoming frame to the pre-move reference.
     Per-square LAB diff scores are published to /perception/square_scores (raw).
     After applying global shift compensation, squares above diff_threshold are
     published as changed_squares.
  5. game_manager matches the changed squares to a legal chess move.
  6. After the computer's move, a new pre-move reference is captured.

Usage:
  python3 fen_visualizer.py
  python3 fen_visualizer.py --no-ros --image /path/to/frame.jpg
  python3 fen_visualizer.py --port 8080
"""

import argparse, threading, time, sys, json, os
import numpy as np
import cv2

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    from sensor_msgs.msg import Image
    from std_srvs.srv import Trigger
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False

try:
    from rcl_interfaces.srv import SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
    _RCL_PARAMS_AVAILABLE = True
except ImportError:
    _RCL_PARAMS_AVAILABLE = False

try:
    from flask import Flask, jsonify, request as freq, Response, render_template_string
except ImportError:
    print("ERROR: pip3 install flask"); sys.exit(1)


# ─── Shared state ──────────────────────────────────────────────────────────────
_lock     = threading.Lock()
_ros_node = None  # FenNode instance, set in main()

_DEFAULT_CORNERS = [[0.10, 0.10], [0.90, 0.10], [0.90, 0.90], [0.10, 0.90]]
_CORNERS_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "board_corners.json")

_state = {
    "raw_frame":      None,    # latest BGR ndarray from camera
    "diff_frame":     None,    # JPEG bytes of /perception/piece_debug heatmap
    "square_scores":  {},      # {sq_name: float} raw LAB diff scores (pre-compensation)
    "cam_info":       "–",
    "frame_count":    0,
    "last_updated":   0.0,
    "error":          "",
    "corners":        [list(c) for c in _DEFAULT_CORNERS],
    "ref_status":     "No reference captured yet. Click 'Capture Ref' to start.",
    # From game_manager ROS topics
    "game_fen":       "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    "game_state":     "OFFLINE",
    "game_turn":      "",
    "fen_source":     "local",
    "last_move":      "",
    "clock_hit_time": 0.0,
}

_params = {
    # Lens correction (corners tab)
    "undistort_enable":     0,
    "undistort_k1":        -25,
    "undistort_k2":          8,
    "undistort_focal":      83,
    # Warp (corners tab)
    "warp_size":           480,
    "show_grid":             1,
    # Diff display thresholds (diff tab — preview only in visualizer)
    "display_threshold":   18.0,
    "shift_compensation":   1.0,
    "clump_enable":          0,
    "clump_keep_per_group":  1,
}

_processed = {"raw": None, "warp": None}
_proc_lock  = threading.Lock()


# ─── Persistence ───────────────────────────────────────────────────────────────
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


# ─── Image helpers ──────────────────────────────────────────────────────────────
def _frame_to_jpeg(frame, scale=1.0, quality=82):
    if scale != 1.0:
        h, w = frame.shape[:2]
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return bytes(buf) if ok else b''


def _blank_jpeg(w, h, msg, color=(160, 160, 160)):
    blank = np.full((h, w, 3), 25, dtype=np.uint8)
    (tw, th), _ = cv2.getTextSize(msg, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
    cv2.putText(blank, msg, (max(0, (w - tw) // 2), (h + th) // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1)
    _, buf = cv2.imencode('.jpg', blank)
    return bytes(buf)


# ─── Lens undistortion ──────────────────────────────────────────────────────────
def _undistort_frame(frame, p):
    if not p.get("undistort_enable", 0):
        return frame
    h, w = frame.shape[:2]
    f = p["undistort_focal"] / 100.0 * w
    K = np.array([[f, 0, w / 2.0], [0, f, h / 2.0], [0, 0, 1.0]], dtype=np.float64)
    D = np.array([p["undistort_k1"] / 100.0, p["undistort_k2"] / 100.0, 0.0, 0.0],
                 dtype=np.float64)
    return cv2.undistort(frame, K, D)


# ─── Perspective warp ───────────────────────────────────────────────────────────
def _corners_to_px(corners_norm, frame):
    h, w = frame.shape[:2]
    return [[c[0] * w, c[1] * h] for c in corners_norm]


def _warp_frame(frame, corners_norm, warp_size):
    corners_px = _corners_to_px(corners_norm, frame)
    src = np.float32(corners_px)
    sz  = warp_size - 1
    dst = np.float32([[0, 0], [sz, 0], [sz, sz], [0, sz]])
    M     = cv2.getPerspectiveTransform(src, dst)
    M_inv = cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(frame, M, (warp_size, warp_size))
    return warped, M, M_inv, corners_px


# ─── Render helpers ─────────────────────────────────────────────────────────────
def _draw_grid(img, sq):
    size = sq * 8
    for i in range(9):
        cv2.line(img, (i * sq, 0),    (i * sq, size),   (0, 200, 200), 1)
        cv2.line(img, (0,      i * sq), (size, i * sq), (0, 200, 200), 1)


def _draw_labels(img, sq):
    for r in range(8):
        cv2.putText(img, str(8 - r), (3, r * sq + sq - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 50), 1)
    for fi, lbl in enumerate('abcdefgh'):
        cv2.putText(img, lbl, (fi * sq + sq // 2 - 4, sq * 8 - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.32, (200, 200, 50), 1)


def _render_corner_cam(frame, corners_px, p, M_inv):
    """Raw camera with grid projected back to camera space + corner handles."""
    out = frame.copy()
    sz  = p["warp_size"]
    sq  = sz // 8
    if p["show_grid"] and M_inv is not None:
        for i in range(9):
            p0 = cv2.perspectiveTransform(np.float32([[[i * sq, 0]]]),  M_inv)[0, 0].astype(int)
            p1 = cv2.perspectiveTransform(np.float32([[[i * sq, sz]]]), M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(p0), tuple(p1), (0, 200, 200), 1)
            q0 = cv2.perspectiveTransform(np.float32([[[0,  i * sq]]]), M_inv)[0, 0].astype(int)
            q1 = cv2.perspectiveTransform(np.float32([[[sz, i * sq]]]), M_inv)[0, 0].astype(int)
            cv2.line(out, tuple(q0), tuple(q1), (0, 200, 200), 1)
    labels = ['TL', 'TR', 'BR', 'BL']
    colors = [(80, 80, 255), (80, 255, 80), (255, 80, 80), (80, 220, 255)]
    for i, (cx, cy) in enumerate(corners_px):
        cv2.circle(out, (int(cx), int(cy)), 11, colors[i], -1)
        cv2.circle(out, (int(cx), int(cy)), 11, (255, 255, 255), 1)
        cv2.putText(out, labels[i], (int(cx) + 14, int(cy) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, colors[i], 2)
    pts = np.array(corners_px, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(out, [pts], True, (0, 255, 100), 2)
    return out


def _render_warp(warped, p):
    """Top-down warped board with checkerboard tint + grid."""
    out = warped.copy()
    sz  = p["warp_size"]
    sq  = sz // 8
    for r in range(8):
        for c in range(8):
            if (r + c) % 2 == 0:
                roi = out[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq].astype(np.float32)
                roi = cv2.addWeighted(roi, 0.82, np.full_like(roi, 190), 0.18, 0)
                out[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = roi.astype(np.uint8)
    if p["show_grid"]:
        _draw_grid(out, sq)
        _draw_labels(out, sq)
    cv2.rectangle(out, (0, 0), (sz - 1, sz - 1), (0, 255, 100), 2)
    return out


# ─── Background processing ─────────────────────────────────────────────────────
def _process_frame(frame, corners, p):
    try:
        warped, M, M_inv, corners_px = _warp_frame(frame, corners, p["warp_size"])
        corner_img = _render_corner_cam(frame, corners_px, p, M_inv)
        warp_img   = _render_warp(warped, p)
        with _lock:
            _state["frame_count"]  += 1
            _state["last_updated"]  = time.time()
            _state["error"]         = ""
        with _proc_lock:
            _processed["raw"]  = _frame_to_jpeg(corner_img, scale=0.75, quality=80)
            _processed["warp"] = _frame_to_jpeg(warp_img,   scale=1.0,  quality=82)
    except Exception as e:
        with _lock:
            _state["error"] = str(e)


def _processing_loop():
    while True:
        time.sleep(1.0)
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


# ─── ROS Node ──────────────────────────────────────────────────────────────────
class FenNode(Node):

    def __init__(self):
        super().__init__("fen_visualizer")

        # Subscriptions
        self.create_subscription(Image,  "/camera/image_raw",
                                 self._on_img, 10)
        self.create_subscription(Image,  "/perception/piece_debug",
                                 self._on_diff_img, 10)
        self.create_subscription(String, "/perception/changed_squares",
                                 self._on_changed_squares, 10)
        self.create_subscription(String, "/perception/square_scores",
                                 self._on_square_scores, 10)
        self.create_subscription(String, "/perception/reference_status",
                                 self._on_ref_status, 10)
        self.create_subscription(String, "/game_manager/board_fen",
                                 self._on_game_fen, 10)
        self.create_subscription(String, "/game_manager/state",
                                 self._on_game_state, 10)
        self.create_subscription(String, "/game_manager/turn",
                                 self._on_game_turn, 10)

        # Publishers
        self._clock_pub = self.create_publisher(Bool, '/limit_switch/clock_hit', 10)

        # Service clients
        self._premove_client = self.create_client(Trigger, '/perception/capture_premove')
        if _RCL_PARAMS_AVAILABLE:
            self._detector_param_client = self.create_client(
                SetParameters, '/piece_detector_node/set_parameters')

        # Requests from Flask threads (set flag → ROS timer picks up)
        self._pending_capture         = False
        self._pending_detector_params = None
        self.create_timer(0.2, self._check_pending)

        self.get_logger().info("FenNode ready — subscribed to perception + game manager")

    def _check_pending(self):
        if self._pending_capture:
            self._pending_capture = False
            if self._premove_client.service_is_ready():
                future = self._premove_client.call_async(Trigger.Request())
                future.add_done_callback(self._on_premove_result)
            else:
                with _lock:
                    _state["ref_status"] = "⚠ /perception/capture_premove not available"

        if self._pending_detector_params and _RCL_PARAMS_AVAILABLE:
            params = self._pending_detector_params
            self._pending_detector_params = None
            if self._detector_param_client.service_is_ready():
                req = SetParameters.Request()
                type_map = {
                    "diff_threshold":            ParameterType.PARAMETER_DOUBLE,
                    "global_shift_compensation": ParameterType.PARAMETER_DOUBLE,
                    "clump_enable":              ParameterType.PARAMETER_BOOL,
                    "clump_keep_per_group":      ParameterType.PARAMETER_INTEGER,
                }
                for name, value in params.items():
                    pv = ParameterValue()
                    ptype = type_map.get(name, ParameterType.PARAMETER_DOUBLE)
                    pv.type = ptype
                    if ptype == ParameterType.PARAMETER_DOUBLE:
                        pv.double_value = float(value)
                    elif ptype == ParameterType.PARAMETER_BOOL:
                        pv.bool_value = bool(value)
                    elif ptype == ParameterType.PARAMETER_INTEGER:
                        pv.integer_value = int(value)
                    p = Parameter()
                    p.name = name
                    p.value = pv
                    req.parameters.append(p)
                self._detector_param_client.call_async(req)

    def _on_premove_result(self, future):
        try:
            result = future.result()
            msg = result.message if result.success else f"Failed: {result.message}"
        except Exception as e:
            msg = f"Error: {e}"
        with _lock:
            _state["ref_status"] = msg

    def publish_clock_hit(self):
        self._clock_pub.publish(Bool(data=True))
        with _lock:
            _state["clock_hit_time"] = time.time()
        self.get_logger().info("Clock hit published from UI")

    def request_capture_premove(self):
        self._pending_capture = True
        with _lock:
            _state["ref_status"] = "Capture requested…"

    def request_detector_params(self, threshold, compensation, clump_enable=False, clump_keep=1):
        self._pending_detector_params = {
            "diff_threshold":            threshold,
            "global_shift_compensation": compensation,
            "clump_enable":              clump_enable,
            "clump_keep_per_group":      clump_keep,
        }

    # ── Camera image ──────────────────────────────────────────────────────────

    def _on_img(self, msg: Image):
        try:
            enc  = msg.encoding.lower()
            h, w = msg.height, msg.width
            step = msg.step
            data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
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
                bpp   = len(msg.data) // (h * w) if h * w > 0 else 3
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

    def _on_diff_img(self, msg: Image):
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

    def _on_changed_squares(self, msg: String):
        with _lock:
            _state["last_move"]    = msg.data.strip()
            _state["last_updated"] = time.time()
            _state["frame_count"] += 1

    def _on_square_scores(self, msg: String):
        try:
            with _lock:
                _state["square_scores"] = json.loads(msg.data)
        except Exception:
            pass

    def _on_ref_status(self, msg: String):
        with _lock:
            _state["ref_status"] = msg.data

    def _on_game_fen(self, msg: String):
        fen = msg.data.strip()
        if fen:
            with _lock:
                _state["game_fen"]   = fen
                _state["fen_source"] = "game_mgr"

    def _on_game_state(self, msg: String):
        with _lock:
            _state["game_state"] = msg.data.strip()

    def _on_game_turn(self, msg: String):
        with _lock:
            _state["game_turn"] = msg.data.strip()


def _ros_thread_fn(node):
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# ─── Flask app ─────────────────────────────────────────────────────────────────
app = Flask(__name__)
app.logger.setLevel("ERROR")


@app.route("/")
def index():
    return render_template_string(HTML_TEMPLATE)


@app.route("/api/status")
def api_status():
    with _lock:
        return jsonify({
            "fen":            _state.get("game_fen", ""),
            "game_state":     _state.get("game_state", "OFFLINE"),
            "game_turn":      _state.get("game_turn", ""),
            "fen_source":     _state.get("fen_source", "local"),
            "last_move":      _state.get("last_move", ""),
            "frame_count":    _state["frame_count"],
            "last_updated":   _state["last_updated"],
            "cam_info":       _state.get("cam_info", "–"),
            "error":          _state.get("error", ""),
            "ref_status":     _state.get("ref_status", "–"),
            "clock_hit_time": _state.get("clock_hit_time", 0.0),
            "ros_connected":  _ros_node is not None,
        })


@app.route("/api/frame")
def api_frame():
    """Raw camera — no overlays."""
    with _lock:
        frame = _state["raw_frame"]
    if frame is None:
        return Response(_blank_jpeg(640, 360, "Waiting for camera..."),
                        mimetype="image/jpeg", headers={"Cache-Control": "no-store"})
    data = _frame_to_jpeg(frame, scale=0.75, quality=80)
    return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/corner_frame")
def api_corner_frame():
    """Camera with grid + corner handle circles rendered in."""
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
        data = _blank_jpeg(480, 480, "Set corners first", (80, 80, 80))
    return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/diff_frame")
def api_diff_frame():
    """Piece detector heatmap from /perception/piece_debug."""
    with _lock:
        data = _state.get("diff_frame")
    if not data:
        data = _blank_jpeg(400, 400, "Waiting for piece_detector...", (100, 180, 255))
    return Response(data, mimetype="image/jpeg", headers={"Cache-Control": "no-store"})


@app.route("/api/square_scores")
def api_square_scores():
    with _lock:
        return jsonify({
            "scores":            dict(_state.get("square_scores", {})),
            "display_threshold": _params.get("display_threshold", 18.0),
            "shift_compensation": _params.get("shift_compensation", 1.0),
            "clump_enable":      int(_params.get("clump_enable", 0)),
            "clump_keep_per_group": int(_params.get("clump_keep_per_group", 1)),
        })


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
                return jsonify({"ok": False, "msg": "Fractions must be 0–1"}), 400
    except Exception as e:
        return jsonify({"ok": False, "msg": str(e)}), 400
    with _lock:
        _state["corners"] = corners
    _save_corners(corners)
    threading.Thread(target=_force_reprocess, daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/clock_hit", methods=["POST"])
def api_clock_hit():
    if _ros_node:
        _ros_node.publish_clock_hit()
        return jsonify({"ok": True, "msg": "Clock hit published to /limit_switch/clock_hit"})
    return jsonify({"ok": False, "msg": "ROS not connected — start with ROS running"}), 503


@app.route("/api/capture_premove", methods=["POST"])
def api_capture_premove():
    if _ros_node:
        _ros_node.request_capture_premove()
        return jsonify({"ok": True, "msg": "Capture request queued"})
    return jsonify({"ok": False, "msg": "ROS not connected"}), 503


@app.route("/api/detector_params", methods=["POST"])
def api_detector_params():
    """Push threshold, compensation, and clump settings to piece_detector_node."""
    data = freq.get_json(silent=True) or {}
    thresh       = float(data.get("diff_threshold", 18.0))
    comp         = float(data.get("shift_compensation", 1.0))
    clump_enable = bool(data.get("clump_enable", False))
    clump_keep   = int(data.get("clump_keep_per_group", 1))
    if _ros_node and _RCL_PARAMS_AVAILABLE:
        _ros_node.request_detector_params(thresh, comp, clump_enable, clump_keep)
        return jsonify({"ok": True,
                        "msg": f"Applied threshold={thresh}, compensation={comp}, "
                               f"clump={clump_enable}(keep={clump_keep}) to piece_detector_node"})
    elif _ros_node and not _RCL_PARAMS_AVAILABLE:
        return jsonify({"ok": False,
                        "msg": "rcl_interfaces not available — cannot set parameters remotely"}), 503
    return jsonify({"ok": False, "msg": "ROS not connected"}), 503


@app.route("/api/fen", methods=["GET"])
def api_fen():
    with _lock:
        return jsonify({
            "fen":          _state.get("game_fen", ""),
            "frame_count":  _state["frame_count"],
            "last_updated": _state["last_updated"],
            "error":        _state.get("error", ""),
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


# ─── HTML Dashboard ────────────────────────────────────────────────────────────
HTML_TEMPLATE = r'''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Chess Board Debug Dashboard</title>
<link rel="stylesheet" href="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.css">
<script src="https://code.jquery.com/jquery-3.5.1.min.js"></script>
<script src="https://unpkg.com/@chrisoakman/chessboardjs@1.0.0/dist/chessboard-1.0.0.min.js"></script>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{background:#0d1117;color:#c9d1d9;font-family:'Segoe UI',system-ui,sans-serif;font-size:13px;min-height:100vh}
code{font-family:monospace;font-size:.9em;background:#161b22;padding:1px 4px;border-radius:3px}

/* ── Header ─────────────────────────────────────────────────────────── */
.hdr{background:#161b22;border-bottom:2px solid #21262d;padding:8px 14px;display:flex;align-items:center;gap:8px;flex-wrap:wrap;position:sticky;top:0;z-index:100}
.hdr-left{display:flex;align-items:center;gap:8px;flex:1;min-width:0;flex-wrap:wrap}
.hdr-right{display:flex;gap:5px;flex-wrap:wrap;align-items:center}
.title{color:#58a6ff;font-size:.95em;font-weight:700;white-space:nowrap}
#status-dot{width:9px;height:9px;border-radius:50%;background:#d29922;flex-shrink:0;transition:background .3s}
#status-dot.live{background:#3fb950;animation:pulse 2s infinite}
#status-dot.dead{background:#f85149}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.3}}

/* ── Status bar ──────────────────────────────────────────────────────── */
.status-bar{background:#0d1117;border-bottom:1px solid #21262d;padding:4px 14px;display:flex;gap:14px;flex-wrap:wrap;font-size:.68em;color:#8b949e}
.status-bar span{white-space:nowrap}
.status-bar .hl{color:#c9d1d9;font-family:monospace}
.status-bar .ok{color:#3fb950}
.status-bar .warn{color:#d29922}
.status-bar .err{color:#f85149}

/* ── Tabs ────────────────────────────────────────────────────────────── */
.tab-nav{display:flex;background:#161b22;border-bottom:2px solid #21262d;padding:0 8px}
.tab-btn{background:none;border:none;border-bottom:3px solid transparent;padding:8px 16px;color:#8b949e;cursor:pointer;font-size:.78em;font-weight:600;transition:color .15s,border-color .15s;margin-bottom:-2px;white-space:nowrap}
.tab-btn:hover{color:#c9d1d9}
.tab-btn.active{color:#58a6ff;border-bottom-color:#58a6ff}
.tab-pane{display:none;padding:12px 14px}
.tab-pane.active{display:block}

/* ── Buttons ─────────────────────────────────────────────────────────── */
.btn{background:#21262d;color:#c9d1d9;border:1px solid #30363d;padding:5px 11px;border-radius:5px;cursor:pointer;font-size:.73em;white-space:nowrap;display:inline-flex;align-items:center;gap:4px;font-weight:500}
.btn:hover{background:#30363d}
.btn.clock{background:#7c3aed;color:#fff;border-color:#7c3aed;font-weight:700;font-size:.8em;letter-spacing:.02em}
.btn.clock:hover{background:#8b5cf6}
.btn.clock.flash{animation:clockflash .4s}
@keyframes clockflash{0%{background:#10b981}50%{background:#059669}100%{background:#7c3aed}}
.btn.blue{background:#1f6feb;color:#fff;border-color:#1f6feb}
.btn.blue:hover{background:#388bfd}
.btn.green{background:#196127;color:#fff;border-color:#196127}
.btn.green:hover{background:#238636}
.btn.orange{background:#9a3412;color:#fff;border-color:#c2410c}
.btn.orange:hover{background:#c2410c}

/* ── Section labels ──────────────────────────────────────────────────── */
.sec-lbl{font-size:.63em;text-transform:uppercase;letter-spacing:.09em;color:#58a6ff;font-weight:700;margin-bottom:6px;padding-bottom:4px;border-bottom:1px solid #21262d}
.instr{background:#161b22;border:1px solid #21262d;border-radius:5px;padding:8px 12px;font-size:.7em;color:#8b949e;line-height:1.7;margin-bottom:10px}
.instr b{color:#c9d1d9}
.instr code{color:#79b8ff}

/* ── Game state badges ───────────────────────────────────────────────── */
.state-badge{padding:2px 8px;border-radius:3px;font-size:.65em;font-weight:700;text-transform:uppercase;letter-spacing:.05em;white-space:nowrap}
.s-offline,.s-startup{background:#21262d;color:#8b949e}
.s-homing{background:#9e6a03;color:#fff}
.s-idle{background:#1c2742;color:#79b8ff}
.s-waiting_player_move{background:#1f6feb;color:#fff}
.s-capturing_board,.s-validating_move{background:#6e40c9;color:#fff}
.s-calculating_response{background:#d29922;color:#0d1117}
.s-executing_move,.s-hitting_clock{background:#e16f24;color:#fff}
.s-promotion_wait{background:#8957e5;color:#fff}
.s-game_over{background:#b91c1c;color:#fff}
.src-badge{padding:1px 6px;border-radius:3px;font-size:.63em;font-weight:600}
.src-game{background:#196127;color:#fff}
.src-local{background:#21262d;color:#8b949e}

/* ── Stat table ──────────────────────────────────────────────────────── */
.stat-table{border:1px solid #21262d;border-radius:5px;overflow:hidden}
.stat-row{display:flex;justify-content:space-between;align-items:center;padding:5px 10px;border-bottom:1px solid #21262d;font-size:.75em;gap:8px}
.stat-row:last-child{border-bottom:none}
.stat-lbl{color:#8b949e;white-space:nowrap;flex-shrink:0}
.stat-val{font-family:monospace;text-align:right;word-break:break-all}
.ok{color:#3fb950}.warn{color:#d29922}.err{color:#f85149}

/* ── Controls ────────────────────────────────────────────────────────── */
.ctrl-section{display:flex;flex-wrap:wrap;gap:10px;margin-top:10px}
.ctrl-group{background:#0d1117;border:1px solid #21262d;border-radius:5px;padding:10px 12px;min-width:240px;flex:1}
.ctrl-title{font-size:.63em;text-transform:uppercase;letter-spacing:.08em;color:#58a6ff;margin-bottom:8px;font-weight:700;border-bottom:1px solid #21262d;padding-bottom:5px}
.slider-row{display:flex;align-items:center;gap:6px;margin:5px 0}
.slider-row label{flex:0 0 150px;font-size:.7em;color:#8b949e;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.slider-row input[type=range]{flex:1;accent-color:#58a6ff;height:4px;min-width:80px}
.slider-row .vb{flex:0 0 42px;text-align:right;font-family:monospace;font-size:.72em;color:#c9d1d9}
.tog-row{display:flex;align-items:center;gap:8px;margin:5px 0}
.tog-row label{font-size:.7em;color:#8b949e;cursor:pointer}
.tog-row input[type=checkbox]{accent-color:#58a6ff;width:13px;height:13px;cursor:pointer}

/* ── Live tab ────────────────────────────────────────────────────────── */
#live-img{width:100%;max-height:78vh;object-fit:contain;display:block;background:#000;border-radius:4px}

/* ── Corners tab ─────────────────────────────────────────────────────── */
.corners-layout{display:grid;grid-template-columns:2fr 1fr;gap:10px;align-items:start}
.cam-wrap{position:relative;width:100%;overflow:hidden;border-radius:4px}
#corner-cam-img{width:100%;display:block;cursor:crosshair;user-select:none}
.handle{position:absolute;width:22px;height:22px;border-radius:50%;transform:translate(-50%,-50%);border:2px solid rgba(255,255,255,0.9);cursor:grab;display:flex;align-items:center;justify-content:center;font-size:9px;font-weight:bold;color:#fff;user-select:none;z-index:10;box-shadow:0 2px 8px rgba(0,0,0,.8)}
.handle:active{cursor:grabbing}
.warp-wrap{border-radius:4px;overflow:hidden;border:1px solid #21262d}
#corner-coords{font-size:.63em;color:#8b949e;padding:5px 8px;background:#0d1117}

/* ── Diff tab ────────────────────────────────────────────────────────── */
.diff-layout{display:grid;grid-template-columns:1fr 1fr;gap:12px;align-items:start}
#diff-img{width:100%;display:block;image-rendering:pixelated;border-radius:4px;border:1px solid #21262d}
#score-canvas{display:block;border:1px solid #21262d;border-radius:4px;image-rendering:pixelated;width:100%;max-width:360px;aspect-ratio:1}
.ref-row{display:flex;align-items:flex-start;gap:8px;padding:6px 0;font-size:.72em;flex-wrap:wrap}
.ref-lbl{color:#8b949e;white-space:nowrap;flex-shrink:0;padding-top:1px}
#ref-status-text{color:#d29922;font-family:monospace;word-break:break-all;flex:1}
.flagged-row{padding:5px 0;font-size:.78em}
.flagged-row .lbl{color:#8b949e}
#flagged-squares{color:#3fb950;font-family:monospace;font-weight:700;letter-spacing:.05em}
.median-row{padding:2px 0;font-size:.68em;color:#8b949e}
#median-floor{color:#79b8ff;font-family:monospace}
.apply-note{font-size:.63em;color:#8b949e;margin-top:4px;line-height:1.5}

/* ── Game tab ────────────────────────────────────────────────────────── */
.game-layout{display:grid;grid-template-columns:300px 1fr;gap:14px;align-items:start}
#board-wrap{padding:4px;display:flex;justify-content:center}
#fen-text{font-family:monospace;font-size:.6em;color:#3fb950;word-break:break-all;background:#0d1117;border:1px solid #21262d;border-radius:4px;padding:5px;margin-top:6px}
.fen-inject-row{display:flex;gap:6px;margin-top:6px}
.fen-inject-row input{flex:1;background:#0d1117;color:#c9d1d9;border:1px solid #21262d;border-radius:4px;padding:4px 7px;font-family:monospace;font-size:.68em}
.pipeline-steps{font-size:.68em;color:#8b949e;line-height:1.9;margin-top:8px}
.pipeline-steps b{color:#c9d1d9}
.pipeline-steps .step-active{color:#3fb950;font-weight:700}

@media (max-width:700px){
  .corners-layout,.diff-layout,.game-layout{grid-template-columns:1fr}
}
</style>
</head>
<body>

<!-- ── Header ──────────────────────────────────────────────────────────────── -->
<div class="hdr">
  <div class="hdr-left">
    <span id="status-dot"></span>
    <span class="title">♟ Chess Board Debug Dashboard</span>
    <span id="hdr-state" class="state-badge s-offline">OFFLINE</span>
    <span id="hdr-turn" style="font-size:.72em;color:#8b949e"></span>
  </div>
  <div class="hdr-right">
    <button class="btn clock" id="clock-btn" onclick="clockHit()" title="Simulate player pressing chess clock — triggers move detection">
      🕐 CLOCK HIT
    </button>
    <button class="btn orange" onclick="captureRef()" title="Capture current board as pre-move reference frame">
      📸 Capture Ref
    </button>
    <button class="btn green" onclick="saveCorners()" title="Save corner positions to board_corners.json">
      📌 Save Corners
    </button>
    <a href="/api/snapshot" class="btn" title="Download full-resolution camera image">
      📷 Snapshot
    </a>
  </div>
</div>

<!-- ── Status bar ──────────────────────────────────────────────────────────── -->
<div class="status-bar">
  <span>Camera: <span class="hl" id="sb-cam">–</span></span>
  <span>Frames: <span class="hl" id="sb-frames">0</span></span>
  <span>Feed age: <span id="sb-age">–</span></span>
  <span>Last detected: <span class="hl" id="sb-squares">–</span></span>
  <span>Ref: <span id="sb-ref">–</span></span>
  <span id="sb-error" class="err"></span>
</div>

<!-- ── Tabs ────────────────────────────────────────────────────────────────── -->
<div class="tab-nav">
  <button class="tab-btn active" data-tab="live"    onclick="showTab('live')">📷 Live Camera</button>
  <button class="tab-btn"        data-tab="corners" onclick="showTab('corners')">📐 Corner Setup</button>
  <button class="tab-btn"        data-tab="diff"    onclick="showTab('diff')">🔍 Diff Analysis</button>
  <button class="tab-btn"        data-tab="game"    onclick="showTab('game')">♟ Game State</button>
</div>

<!-- ═══════════════════════════════════════════════════════════════════════════
     Tab: Live Camera
════════════════════════════════════════════════════════════════════════════ -->
<div id="tab-live" class="tab-pane active">
  <div class="instr">
    <b>Raw camera feed</b> — no processing applied. Check camera focus, lighting, and board visibility here.
    The board should be evenly lit with no harsh shadows. <b>Go to Corner Setup</b> to align the board boundaries.
  </div>
  <img id="live-img" src="/api/frame" alt="Live camera">
</div>

<!-- ═══════════════════════════════════════════════════════════════════════════
     Tab: Corner Setup
════════════════════════════════════════════════════════════════════════════ -->
<div id="tab-corners" class="tab-pane">
  <div class="instr">
    <b>Drag the 4 colored handles</b> to the corners of the chess board in the camera image.
    <b style="color:#5050ff">■ TL</b> = top-left &nbsp;
    <b style="color:#50ff50">■ TR</b> = top-right &nbsp;
    <b style="color:#ff5050">■ BR</b> = bottom-right &nbsp;
    <b style="color:#50ddff">■ BL</b> = bottom-left (as seen from the camera, not from above).
    The <b>Warped View</b> on the right should show a perfectly square top-down board when aligned correctly.
    Click <b>Save Corners</b> in the header to persist across restarts.
  </div>
  <div class="corners-layout">
    <div>
      <div class="sec-lbl">Live Camera — drag corner handles</div>
      <div class="cam-wrap" id="corner-cam-wrap">
        <img id="corner-cam-img" src="/api/corner_frame" draggable="false" alt="corners">
      </div>
      <div id="corner-coords">Corners: loading…</div>
    </div>
    <div>
      <div class="sec-lbl">Warped Top-Down View</div>
      <div class="warp-wrap">
        <img id="warp-img" src="/api/warp_frame" alt="warp" style="width:100%;display:block;image-rendering:pixelated">
        <div id="warp-status" style="font-size:.63em;color:#8b949e;padding:4px 8px;background:#0d1117"></div>
      </div>
    </div>
  </div>
  <div class="ctrl-section">
    <div class="ctrl-group">
      <div class="ctrl-title">Lens Correction</div>
      <div class="tog-row">
        <input type="checkbox" id="t_undistort_enable"
               onchange="onTog('undistort_enable',this.checked)">
        <label for="t_undistort_enable">Enable lens undistortion</label>
      </div>
      <div class="slider-row">
        <label title="k1: barrel (negative) or pincushion (positive) distortion">k1 barrel/pincushion</label>
        <input type="range" id="s_undistort_k1" min="-80" max="80" step="1" value="-25"
               oninput="onSlider('undistort_k1',+this.value);document.getElementById('vb_undistort_k1').textContent=this.value">
        <span class="vb" id="vb_undistort_k1">-25</span>
      </div>
      <div class="slider-row">
        <label title="k2: higher-order radial distortion correction">k2 higher-order</label>
        <input type="range" id="s_undistort_k2" min="-50" max="50" step="1" value="8"
               oninput="onSlider('undistort_k2',+this.value);document.getElementById('vb_undistort_k2').textContent=this.value">
        <span class="vb" id="vb_undistort_k2">8</span>
      </div>
      <div class="slider-row">
        <label title="Effective focal length as % of frame width">Focal length (% width)</label>
        <input type="range" id="s_undistort_focal" min="40" max="130" step="1" value="83"
               oninput="onSlider('undistort_focal',+this.value);document.getElementById('vb_undistort_focal').textContent=this.value">
        <span class="vb" id="vb_undistort_focal">83</span>
      </div>
    </div>
    <div class="ctrl-group">
      <div class="ctrl-title">Warp & Display</div>
      <div class="slider-row">
        <label title="Resolution of the warped top-down board image in pixels">Warp size (px)</label>
        <input type="range" id="s_warp_size" min="240" max="800" step="40" value="480"
               oninput="onSlider('warp_size',+this.value);document.getElementById('vb_warp_size').textContent=this.value">
        <span class="vb" id="vb_warp_size">480</span>
      </div>
      <div class="tog-row">
        <input type="checkbox" id="t_show_grid" checked
               onchange="onTog('show_grid',this.checked)">
        <label for="t_show_grid">Show rank/file grid overlay</label>
      </div>
      <div style="margin-top:8px;display:flex;gap:6px">
        <button class="btn" onclick="resetCorners()">↺ Reset corners</button>
        <button class="btn blue" onclick="saveCorners()">📌 Save corners</button>
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════════════════════════════════════════════════════════════════
     Tab: Diff Analysis
════════════════════════════════════════════════════════════════════════════ -->
<div id="tab-diff" class="tab-pane">
  <div class="instr">
    <b>Left panel:</b> Heatmap from <code>piece_detector_node</code> — shows per-square LAB color difference
    between the pre-move reference frame and the current frame. Green outlines = squares flagged as changed
    by the actual detector (using the node's own threshold and compensation settings).
    <br>
    <b>Right panel:</b> Interactive score grid using the <b>sliders below</b> — adjust threshold and
    shift compensation to preview how detection would behave without restarting the node.
    <b>Global shift compensation</b> subtracts <code>median_score × factor</code> from every square.
    Camera vibration raises all squares equally, so after subtraction all ≈ 0. A moved piece raises only
    its square, so it stands out clearly after subtraction.
    Use <b>Apply to Detector</b> to push your tuned values to the actual node.
    Use <b>Capture Ref</b> in the header to take a fresh reference frame before testing.
  </div>
  <div class="diff-layout">
    <!-- Left: local warped board + heatmap from ROS -->
    <div>
      <div class="sec-lbl">Current Warped Board (local — matches what detector sees)</div>
      <img id="warp-diff-img" src="/api/warp_frame" style="width:100%;display:block;border-radius:4px;border:1px solid #21262d;margin-bottom:8px;image-rendering:pixelated" alt="warped board">
      <div class="sec-lbl">Piece Detector Heatmap (live from ROS)</div>
      <img id="diff-img" src="/api/diff_frame" alt="diff heatmap">
      <div class="ref-row">
        <span class="ref-lbl">Reference status:</span>
        <span id="ref-status-text">–</span>
      </div>
      <button class="btn orange" onclick="captureRef()" style="margin-top:4px">
        📸 Capture New Pre-move Reference
      </button>
      <div style="font-size:.63em;color:#8b949e;margin-top:5px;line-height:1.6">
        Click <b>Capture Ref</b> to snapshot the current board as the reference frame.
        Then move a piece and watch the heatmap — moved squares should light up bright.
        Press <b>Clock Hit</b> to trigger the game state machine.
      </div>
    </div>
    <!-- Right: interactive score grid -->
    <div>
      <div class="sec-lbl">Interactive Score Grid (preview — uses sliders below)</div>
      <canvas id="score-canvas" width="400" height="400"></canvas>
      <div class="flagged-row">
        <span class="lbl">Would flag: </span>
        <span id="flagged-squares">–</span>
      </div>
      <div class="median-row">
        Compensation floor (median × factor): <span id="median-floor">–</span>
      </div>

      <div class="ctrl-group" style="margin-top:10px">
        <div class="ctrl-title">Detection Thresholds
          <small style="font-weight:400;text-transform:none;letter-spacing:0">— preview only</small>
        </div>
        <div class="slider-row">
          <label title="Adjusted score above this = square is flagged as changed. Default: 18">
            Diff threshold
          </label>
          <input type="range" id="s_display_threshold" min="5" max="80" step="0.5" value="18"
                 oninput="onSlider('display_threshold',+this.value);document.getElementById('vb_display_threshold').textContent=(+this.value).toFixed(1)">
          <span class="vb" id="vb_display_threshold">18.0</span>
        </div>
        <div class="slider-row">
          <label title="0 = no compensation. 1 = subtract full median. >1 = aggressive. Default: 1.0">
            Shift compensation
          </label>
          <input type="range" id="s_shift_compensation" min="0" max="2.5" step="0.05" value="1.0"
                 oninput="onSlider('shift_compensation',+this.value);document.getElementById('vb_shift_compensation').textContent=(+this.value).toFixed(2)">
          <span class="vb" id="vb_shift_compensation">1.00</span>
        </div>
        <div class="tog-row">
          <input type="checkbox" id="t_clump_enable" onchange="onTog('clump_enable',this.checked)">
          <label for="t_clump_enable" title="Group adjacent flagged squares and keep only the hottest per cluster. Reduces false positives when a far-end piece bleeds across multiple dewarped squares. NOTE: castling touches 4 adjacent squares — set Keep ≥ 4 or disable clump mode during castling.">Clump mode (reduce perspective bleed)</label>
        </div>
        <div class="slider-row">
          <label title="How many squares to keep per clump. Use 1 for tightest filtering. Use 2-4 if castling is needed.">Keep per clump</label>
          <input type="range" id="s_clump_keep_per_group" min="1" max="4" step="1" value="1"
                 oninput="onSlider('clump_keep_per_group',+this.value);document.getElementById('vb_clump_keep').textContent=this.value">
          <span class="vb" id="vb_clump_keep">1</span>
        </div>
        <button class="btn blue" onclick="applyToDetector()" style="margin-top:8px">
          ⚙ Apply to Actual Detector
        </button>
        <div class="apply-note">
          Pushes <code>diff_threshold</code>, <code>global_shift_compensation</code>,
          and <code>clump_enable</code> to <code>piece_detector_node</code> via ROS set_parameters.
          The heatmap (left) will reflect the new values after the next detection cycle.
        </div>
      </div>
    </div>
  </div>
</div>

<!-- ═══════════════════════════════════════════════════════════════════════════
     Tab: Game State
════════════════════════════════════════════════════════════════════════════ -->
<div id="tab-game" class="tab-pane">
  <div class="game-layout">
    <!-- Left: board diagram -->
    <div>
      <div class="sec-lbl">Board State</div>
      <div id="board-wrap"><div id="board"></div></div>
      <div id="fen-text">–</div>
      <div style="margin-top:10px">
        <div class="sec-lbl">Inject FEN (offline test)</div>
        <div class="fen-inject-row">
          <input id="inj-in" type="text"
                 value="rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1">
          <button class="btn blue" onclick="injectFen()">Send</button>
        </div>
      </div>
    </div>
    <!-- Right: game info + pipeline guide -->
    <div>
      <div class="sec-lbl">Game Info</div>
      <div class="stat-table">
        <div class="stat-row">
          <span class="stat-lbl">State</span>
          <span class="stat-val"><span id="gi-state" class="state-badge s-offline">OFFLINE</span></span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Turn</span>
          <span class="stat-val" id="gi-turn">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Move #</span>
          <span class="stat-val" id="gi-fullmove">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Last detected squares</span>
          <span class="stat-val" id="gi-lastmove" style="color:#58a6ff;font-family:monospace">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">FEN source</span>
          <span class="stat-val"><span id="gi-source" class="src-badge src-local">Local</span></span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Camera</span>
          <span class="stat-val" id="gi-cam">–</span>
        </div>
        <div class="stat-row">
          <span class="stat-lbl">Error</span>
          <span class="stat-val err" id="gi-err" style="font-size:.63em">–</span>
        </div>
      </div>

      <div style="margin-top:14px">
        <div class="sec-lbl">Detection Pipeline — How It Works</div>
        <div class="pipeline-steps" id="pipeline-steps">
          <div id="ps1">1. game_manager enters <b>WAITING_PLAYER_MOVE</b></div>
          <div id="ps2">2. <b>Pre-move reference</b> captured by piece_detector_node</div>
          <div id="ps3">3. Human moves a piece on the board</div>
          <div id="ps4">4. Human presses <b>clock</b> (or use CLOCK HIT button)</div>
          <div id="ps5">5. game_manager → <b>CAPTURING_BOARD</b></div>
          <div id="ps6">6. piece_detector compares current frame to reference</div>
          <div id="ps7">7. Changed squares matched to a legal chess move</div>
          <div id="ps8">8. Computer responds, new reference captured → repeat</div>
        </div>
      </div>

      <div style="margin-top:12px">
        <button class="btn clock" onclick="clockHit()">🕐 CLOCK HIT</button>
        <span style="font-size:.65em;color:#8b949e;margin-left:8px">
          Triggers the same action as the physical clock button
        </span>
      </div>
    </div>
  </div>
</div>

<script>
// ── Chess board ──────────────────────────────────────────────────────────────
var chessboard = Chessboard('board', {
  position: 'start',
  showNotation: true,
  pieceTheme: 'https://lichess1.org/assets/piece/cburnett/{piece}.svg'
});

// ── Tab management ───────────────────────────────────────────────────────────
var currentTab = 'live';
function showTab(id) {
  document.querySelectorAll('.tab-pane').forEach(function(el){ el.classList.remove('active'); });
  document.querySelectorAll('.tab-btn').forEach(function(el){ el.classList.remove('active'); });
  document.getElementById('tab-' + id).classList.add('active');
  document.querySelector('[data-tab="' + id + '"]').classList.add('active');
  currentTab = id;
  if (id === 'corners') { positionHandles(); }
}

// ── Corner drag handles ──────────────────────────────────────────────────────
var corners = [{fx:0.10,fy:0.10},{fx:0.90,fy:0.10},{fx:0.90,fy:0.90},{fx:0.10,fy:0.90}];
var CORNER_LABELS = ['TL','TR','BR','BL'];
var CORNER_COLORS = ['#5050ff','#50ff50','#ff5050','#50ddff'];
var handles = [];
var dragging = null;       // integer index (0–3) or null — NOT an object
var handlesBuilt = false;  // only create DOM elements once; survive image reloads

function positionHandles() {
  var img = document.getElementById('corner-cam-img');
  if (!img || !img.clientWidth) return;
  var W = img.clientWidth, H = img.clientHeight;
  corners.forEach(function(c, i) {
    if (!handles[i]) return;
    handles[i].style.left = (c.fx * W) + 'px';
    handles[i].style.top  = (c.fy * H) + 'px';
  });
}

function buildHandles() {
  // Guard: build handle DOM elements only once.
  // refreshImages() reloads the image src every 800 ms, firing the 'load' event
  // each time.  Without this guard each reload destroys and recreates the handles,
  // interrupting any drag in progress.
  if (handlesBuilt) { positionHandles(); return; }
  var wrap = document.getElementById('corner-cam-wrap');
  if (!wrap) return;
  for (var i = 0; i < 4; i++) {
    (function(idx) {
      var el = document.createElement('div');
      el.className = 'handle';
      el.style.background = CORNER_COLORS[idx];
      el.textContent = CORNER_LABELS[idx];
      wrap.appendChild(el);
      handles.push(el);
      el.addEventListener('mousedown', function(e) {
        e.preventDefault(); e.stopPropagation(); dragging = idx;
      });
      el.addEventListener('touchstart', function(e) {
        e.preventDefault(); dragging = idx;
      }, {passive:false});
    })(i);
  }
  handlesBuilt = true;
  positionHandles();
}

function moveDragging(clientX, clientY) {
  if (dragging === null) return;
  var img = document.getElementById('corner-cam-img');
  if (!img) return;
  // Use absolute cursor position within the image element.
  // getBoundingClientRect() is stable throughout the drag regardless of image
  // reload or layout reflow, making this more reliable than delta-based tracking.
  var rect = img.getBoundingClientRect();
  corners[dragging].fx = Math.min(1, Math.max(0, (clientX - rect.left) / rect.width));
  corners[dragging].fy = Math.min(1, Math.max(0, (clientY - rect.top)  / rect.height));
  positionHandles();
  updateCornerInfo();
}

document.addEventListener('mousemove', function(e){ moveDragging(e.clientX, e.clientY); });
document.addEventListener('mouseup',   function(){ if(dragging !== null){ dragging=null; sendCorners(); } });
document.addEventListener('touchmove', function(e){
  if(dragging === null) return; e.preventDefault();
  moveDragging(e.touches[0].clientX, e.touches[0].clientY);
}, {passive:false});
document.addEventListener('touchend', function(){ if(dragging !== null){ dragging=null; sendCorners(); } });

function updateCornerInfo() {
  var txt = corners.map(function(c,i){
    return CORNER_LABELS[i]+'('+c.fx.toFixed(3)+','+c.fy.toFixed(3)+')';
  }).join('  ');
  var el = document.getElementById('corner-coords');
  if (el) el.textContent = 'Corners (normalized 0–1): '+txt;
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
document.getElementById('corner-cam-img').addEventListener('load', function() {
  // On first load build the handles; on subsequent reloads just reposition.
  if (!handlesBuilt) { buildHandles(); } else { positionHandles(); }
});
window.addEventListener('resize', positionHandles);

// ── Score canvas ─────────────────────────────────────────────────────────────
var latestScores = {};

// BFS 8-connected components over a set of square names (e.g. ["e2","e3"]).
// Returns array of arrays, each inner array is one clump.
function findClumps(flaggedArr) {
  var remaining = {};
  flaggedArr.forEach(function(sq){ remaining[sq] = true; });
  var clumps = [];
  while (Object.keys(remaining).length > 0) {
    var keys = Object.keys(remaining);
    var start = keys[0];
    var clump = [], queue = [start];
    while (queue.length > 0) {
      var sq = queue.shift();
      if (!remaining[sq]) continue;
      delete remaining[sq];
      clump.push(sq);
      var col = sq.charCodeAt(0) - 97;   // 'a'=0
      var row = parseInt(sq[1]);          // 1–8
      for (var dc = -1; dc <= 1; dc++) {
        for (var dr = -1; dr <= 1; dr++) {
          if (dc === 0 && dr === 0) continue;
          var nc = col + dc, nr = row + dr;
          if (nc >= 0 && nc < 8 && nr >= 1 && nr <= 8) {
            var nsq = 'abcdefgh'[nc] + nr;
            if (remaining[nsq]) queue.push(nsq);
          }
        }
      }
    }
    clumps.push(clump);
  }
  return clumps;
}

// Build a map of: repSet (squares kept as representative) and clumpIdMap.
// adjScores: {sqName: adjustedScore}  clumpKeep: how many to keep per clump.
function buildClumpMap(flaggedArr, adjScores, clumpKeep) {
  var clumps = findClumps(flaggedArr);
  var repSet = {}, clumpIdMap = {};
  clumps.forEach(function(clump, ci) {
    var sorted = clump.slice().sort(function(a, b) {
      return (adjScores[b] || 0) - (adjScores[a] || 0);
    });
    sorted.forEach(function(sq, rank) {
      clumpIdMap[sq] = ci;
      if (rank < clumpKeep) repSet[sq] = true;
    });
  });
  return { repSet: repSet, clumpIdMap: clumpIdMap };
}

function drawScoreGrid(scores, threshold, compensation, clumpEnable, clumpKeep) {
  var canvas = document.getElementById('score-canvas');
  if (!canvas) return;
  var ctx = canvas.getContext('2d');
  var sz = canvas.width;
  var sq = sz / 8;

  // Compute compensation floor
  var vals = Object.values(scores);
  var floor = 0;
  if (compensation > 0 && vals.length > 0) {
    var sorted = vals.slice().sort(function(a,b){return a-b;});
    var mid = Math.floor(sorted.length / 2);
    var median = sorted.length % 2 === 0
      ? (sorted[mid-1] + sorted[mid]) / 2
      : sorted[mid];
    floor = median * compensation;
  }
  document.getElementById('median-floor').textContent =
    floor > 0 ? floor.toFixed(1) : '0 (disabled)';

  // Compute adjusted scores for all squares and build flagged list
  var adjScores = {}, flagged = [];
  Object.keys(scores).forEach(function(sqn) {
    var adj = Math.max(0, (scores[sqn] || 0) - floor);
    adjScores[sqn] = adj;
    if (adj > threshold) flagged.push(sqn);
  });

  // Clump analysis
  var repSet = null, effectiveFlagged = flagged;
  if (clumpEnable && flagged.length > 0) {
    var cm = buildClumpMap(flagged, adjScores, clumpKeep || 1);
    repSet = cm.repSet;
    effectiveFlagged = Object.keys(repSet);
  }

  ctx.clearRect(0, 0, sz, sz);

  for (var row = 0; row < 8; row++) {
    for (var col = 0; col < 8; col++) {
      var sqName = 'abcdefgh'[col] + (8 - row);
      var raw = scores[sqName] || 0;
      var adj = adjScores[sqName] || 0;
      var isFlagged = adj > threshold;
      var isRep  = repSet ? !!repSet[sqName]       : isFlagged;
      var isSat  = isFlagged && repSet && !repSet[sqName]; // suppressed by clump

      // Checkerboard base tint
      var isDark = (row + col) % 2 === 1;
      var baseBg = isDark ? 25 : 35;

      // Score heat color: low=dark-green, mid=yellow, high=red
      var t = Math.min(1.0, adj / Math.max(1, threshold * 1.5));
      var r, g, b;
      if (t < 0.5) {
        r = Math.round(t * 2 * 200);
        g = Math.round(120 + t * 2 * (200 - 120));
        b = baseBg;
      } else {
        r = 200;
        g = Math.round(200 * (1 - (t - 0.5) * 2));
        b = baseBg;
      }
      ctx.fillStyle = 'rgb('+r+','+g+','+b+')';
      ctx.fillRect(col * sq, row * sq, sq, sq);

      // Outlines: green=representative/normal-flagged, orange=satellite suppressed by clump
      if (isSat) {
        ctx.strokeStyle = '#ff8c00';
        ctx.lineWidth = 2;
        ctx.strokeRect(col * sq + 1.5, row * sq + 1.5, sq - 3, sq - 3);
      } else if (isRep && isFlagged) {
        ctx.strokeStyle = '#00ff80';
        ctx.lineWidth = 3;
        ctx.strokeRect(col * sq + 1.5, row * sq + 1.5, sq - 3, sq - 3);
      }

      // Adjusted score (center)
      var lblColor = (isRep && isFlagged) ? '#00ff80' : isSat ? '#ff8c00' : (adj > threshold * 0.5 ? '#fff' : '#aaa');
      ctx.fillStyle = lblColor;
      ctx.font = 'bold ' + Math.round(sq * 0.28) + 'px monospace';
      ctx.textAlign = 'center';
      ctx.fillText(adj.toFixed(0), col * sq + sq / 2, row * sq + sq / 2 + 2);

      // Raw score (top-left, small)
      ctx.fillStyle = '#666';
      ctx.font = Math.round(sq * 0.18) + 'px monospace';
      ctx.textAlign = 'left';
      ctx.fillText(raw.toFixed(0), col * sq + 2, row * sq + Math.round(sq * 0.22));

      // Square name (bottom-right, small)
      ctx.fillStyle = '#555';
      ctx.font = Math.round(sq * 0.18) + 'px monospace';
      ctx.textAlign = 'right';
      ctx.fillText(sqName, (col + 1) * sq - 2, (row + 1) * sq - 2);

      // Star mark on representative square when clump mode is active
      if (clumpEnable && isRep && isFlagged) {
        ctx.fillStyle = '#ffe066';
        ctx.font = Math.round(sq * 0.22) + 'px monospace';
        ctx.textAlign = 'right';
        ctx.fillText('★', (col + 1) * sq - 2, row * sq + Math.round(sq * 0.28));
      }
    }
  }

  // Legend bar
  ctx.fillStyle = '#21262d';
  ctx.fillRect(0, sz - 14, sz, 14);
  ctx.fillStyle = '#8b949e';
  ctx.font = '10px monospace';
  ctx.textAlign = 'left';
  var legend = clumpEnable
    ? '■green=representative  ■orange=clump-suppressed  ★=kept  thr='+threshold.toFixed(1)
    : 'large=adj  small=raw  ■green=flagged (adj > '+threshold.toFixed(1)+')';
  ctx.fillText(legend, 4, sz - 3);

  var el = document.getElementById('flagged-squares');
  el.textContent = effectiveFlagged.length > 0 ? effectiveFlagged.join(' ') : '(none)';
}

function _clumpKeepVal() {
  var el = document.getElementById('s_clump_keep_per_group');
  return el ? parseInt(el.value) || 1 : 1;
}

function pollScores() {
  if (currentTab !== 'diff') return;
  fetch('/api/square_scores').then(function(r){return r.json();}).then(function(d){
    latestScores = d.scores || {};
    var thresh      = d.display_threshold || 18;
    var comp        = d.shift_compensation || 1.0;
    var clumpEnable = !!d.clump_enable;
    var clumpKeep   = d.clump_keep_per_group || _clumpKeepVal();
    drawScoreGrid(latestScores, thresh, comp, clumpEnable, clumpKeep);
  }).catch(function(){});
}

// ── Params ───────────────────────────────────────────────────────────────────
function onSlider(k, v) {
  fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[k]:v})});
  if (k === 'display_threshold' || k === 'shift_compensation' || k === 'clump_keep_per_group') {
    fetch('/api/square_scores').then(function(r){return r.json();}).then(function(d){
      latestScores = d.scores || {};
      drawScoreGrid(latestScores, d.display_threshold || 18, d.shift_compensation || 1.0,
                    !!d.clump_enable, _clumpKeepVal());
    });
  }
}
function onTog(k, checked) {
  fetch('/api/params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({[k]: checked ? 1 : 0})}).then(function() {
    if (k === 'clump_enable') {
      fetch('/api/square_scores').then(function(r){return r.json();}).then(function(d){
        latestScores = d.scores || {};
        drawScoreGrid(latestScores, d.display_threshold || 18, d.shift_compensation || 1.0,
                      checked, _clumpKeepVal());
      });
    }
  });
}

function applyToDetector() {
  var thresh = parseFloat(document.getElementById('s_display_threshold').value);
  var comp   = parseFloat(document.getElementById('s_shift_compensation').value);
  var clumpEl = document.getElementById('t_clump_enable');
  var clumpEnable = clumpEl ? clumpEl.checked : false;
  fetch('/api/detector_params',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({diff_threshold: thresh, shift_compensation: comp,
                         clump_enable: clumpEnable, clump_keep_per_group: _clumpKeepVal()})
  }).then(function(r){return r.json();}).then(function(d){
    alert(d.ok ? '✓ ' + d.msg : '✗ ' + d.msg);
  });
}

// ── Header actions ───────────────────────────────────────────────────────────
function clockHit() {
  fetch('/api/clock_hit',{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    if (!d.ok) { alert('Clock hit failed: '+d.msg); return; }
    var btn = document.getElementById('clock-btn');
    btn.classList.add('flash');
    setTimeout(function(){ btn.classList.remove('flash'); }, 450);
  });
}
function captureRef() {
  fetch('/api/capture_premove',{method:'POST'}).then(function(r){return r.json();}).then(function(d){
    if (!d.ok) alert('Capture failed: '+d.msg);
  });
}
function injectFen() {
  var f = document.getElementById('inj-in').value.trim();
  fetch('/api/fen',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify({fen:f})});
}

// ── Image refresh ─────────────────────────────────────────────────────────────
function refreshImages() {
  var t = Date.now();
  if (currentTab === 'live') {
    document.getElementById('live-img').src = '/api/frame?t=' + t;
  } else if (currentTab === 'corners') {
    document.getElementById('corner-cam-img').src = '/api/corner_frame?t=' + t;
    document.getElementById('warp-img').src        = '/api/warp_frame?t=' + t;
  } else if (currentTab === 'diff') {
    document.getElementById('diff-img').src = '/api/diff_frame?t=' + t;
    var wdi = document.getElementById('warp-diff-img');
    if (wdi) wdi.src = '/api/warp_frame?t=' + t;
    pollScores();
  }
}

// ── Status poll ───────────────────────────────────────────────────────────────
var STATE_LABEL = {
  'OFFLINE':'Offline','STARTUP':'Startup','HOMING':'Homing',
  'IDLE':'Idle — Ready','WAITING_PLAYER_MOVE':'Waiting for Player',
  'CAPTURING_BOARD':'Capturing Board','VALIDATING_MOVE':'Validating Move',
  'CALCULATING_RESPONSE':'Engine Thinking','EXECUTING_MOVE':'Executing Move',
  'HITTING_CLOCK':'Hitting Clock','PROMOTION_WAIT':'Promotion Wait','GAME_OVER':'Game Over'
};

function pollStatus() {
  fetch('/api/status').then(function(r){return r.json();}).then(function(d){
    var s = (d.game_state || 'OFFLINE').toUpperCase();
    var lbl = STATE_LABEL[s] || s.replace(/_/g,' ');

    // Header badges
    var hdrState = document.getElementById('hdr-state');
    hdrState.textContent = lbl;
    hdrState.className   = 'state-badge s-' + s.toLowerCase();
    var t = (d.game_turn || '').toUpperCase();
    document.getElementById('hdr-turn').textContent =
      t === 'WHITE' ? '⬜ White to move' : t === 'BLACK' ? '⬛ Black to move' : '';

    // Status bar
    document.getElementById('sb-cam').textContent    = d.cam_info || '–';
    document.getElementById('sb-frames').textContent  = d.frame_count || 0;
    document.getElementById('sb-ref').textContent     = (d.ref_status || '–').substring(0, 60);
    var sq = d.last_move;
    document.getElementById('sb-squares').textContent = sq ? sq.split(',').join(' → ') : '–';
    var err = d.error || '';
    document.getElementById('sb-error').textContent   = err ? ('⚠ ' + err) : '';

    var age = d.last_updated > 0 ? (Date.now()/1000 - d.last_updated) : 9999;
    var ageEl = document.getElementById('sb-age');
    ageEl.textContent = age < 9999 ? age.toFixed(1) + 's' : 'never';
    ageEl.className   = age < 5 ? 'ok' : age < 20 ? 'warn' : 'err';

    var dot = document.getElementById('status-dot');
    dot.className = age < 5 ? 'live' : age < 30 ? '' : 'dead';

    // Game tab stats
    document.getElementById('gi-state').textContent = lbl;
    document.getElementById('gi-state').className   = 'state-badge s-' + s.toLowerCase();
    if (t==='WHITE'){ document.getElementById('gi-turn').textContent='⬜ White'; document.getElementById('gi-turn').className='stat-val turn-white'; }
    else if(t==='BLACK'){document.getElementById('gi-turn').textContent='⬛ Black';document.getElementById('gi-turn').className='stat-val turn-black';}
    else { document.getElementById('gi-turn').textContent='–'; document.getElementById('gi-turn').className='stat-val'; }

    document.getElementById('gi-lastmove').textContent = sq ? sq.split(',').join(' → ') : '–';
    document.getElementById('gi-cam').textContent      = d.cam_info || '–';
    document.getElementById('gi-err').textContent      = err || '–';

    var srcEl = document.getElementById('gi-source');
    if(d.fen_source==='game_mgr'){srcEl.textContent='Game Manager ✓';srcEl.className='src-badge src-game';}
    else{srcEl.textContent='Local (offline)';srcEl.className='src-badge src-local';}

    // Ref status on diff tab
    var rsEl = document.getElementById('ref-status-text');
    if (rsEl) {
      rsEl.textContent = d.ref_status || '–';
      rsEl.style.color = (d.ref_status||'').includes('captured') ? '#3fb950' :
                         (d.ref_status||'').includes('⚠') ? '#f85149' : '#d29922';
    }

    // FEN + board
    var fen = d.fen || '';
    if (fen) {
      try { chessboard.position(fen.split(' ')[0], false); } catch(e) {}
      document.getElementById('fen-text').textContent = fen;
      var parts = fen.split(' ');
      if (parts.length >= 6)
        document.getElementById('gi-fullmove').textContent = 'Move ' + parts[5];
    }

    // Pipeline step highlight
    var stateStepMap = {
      'WAITING_PLAYER_MOVE':'ps1,ps2','CAPTURING_BOARD':'ps5','VALIDATING_MOVE':'ps6,ps7',
      'CALCULATING_RESPONSE':'ps7','EXECUTING_MOVE':'ps8','HITTING_CLOCK':'ps8'
    };
    document.querySelectorAll('.pipeline-steps div').forEach(function(el){ el.className=''; });
    var active = stateStepMap[s] || '';
    if (active) active.split(',').forEach(function(id){
      var el = document.getElementById(id);
      if (el) el.className = 'step-active';
    });

  }).catch(function(){
    document.getElementById('status-dot').className = 'dead';
  });
}

// ── Initial params load ───────────────────────────────────────────────────────
fetch('/api/params').then(function(r){return r.json();}).then(function(p){
  if (p.undistort_enable) document.getElementById('t_undistort_enable').checked = true;
  if (p.show_grid !== undefined) document.getElementById('t_show_grid').checked = !!p.show_grid;
  ['undistort_k1','undistort_k2','undistort_focal','warp_size'].forEach(function(k){
    var el = document.getElementById('s_'+k);
    var vb = document.getElementById('vb_'+k);
    if (el && p[k] !== undefined) { el.value = p[k]; if(vb) vb.textContent = p[k]; }
  });
  ['display_threshold','shift_compensation'].forEach(function(k){
    var el = document.getElementById('s_'+k);
    var vb = document.getElementById('vb_'+k);
    if (el && p[k] !== undefined) { el.value = p[k]; if(vb) vb.textContent = parseFloat(p[k]).toFixed(k==='shift_compensation'?2:1); }
  });
  var clumpChk = document.getElementById('t_clump_enable');
  if (clumpChk && p.clump_enable !== undefined) clumpChk.checked = !!p.clump_enable;
  var clumpKpEl = document.getElementById('s_clump_keep_per_group');
  var clumpKpVb = document.getElementById('vb_clump_keep');
  if (clumpKpEl && p.clump_keep_per_group !== undefined) {
    clumpKpEl.value = p.clump_keep_per_group;
    if (clumpKpVb) clumpKpVb.textContent = p.clump_keep_per_group;
  }
});

// ── Boot ──────────────────────────────────────────────────────────────────────
buildHandles();
loadCornersFromServer();
updateCornerInfo();
setInterval(refreshImages, 800);
setInterval(pollStatus,    1000);
refreshImages();
pollStatus();
</script>
</body>
</html>'''


# ─── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Chess Board Debug Dashboard")
    parser.add_argument("--port",   type=int, default=5000)
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true",
                        help="Skip ROS init — use with --image for offline testing")
    parser.add_argument("--fen",    default=None, help="Initial FEN to display")
    parser.add_argument("--image",  default=None, help="Static image for offline testing")
    args = parser.parse_args()

    _load_corners()

    if args.fen:
        with _lock:
            _state["game_fen"]     = args.fen
            _state["fen_source"]   = "local"
            _state["last_updated"] = time.time()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is not None:
            with _lock:
                _state["raw_frame"] = frame
            print(f"✓ Loaded static image: {args.image}  {frame.shape[1]}×{frame.shape[0]}")
        else:
            print(f"⚠  Could not load: {args.image}")

    threading.Thread(target=_processing_loop, daemon=True).start()

    global _ros_node
    if not args.no_ros:
        if not ROS_AVAILABLE:
            print("ERROR: rclpy not found — run with --no-ros for offline mode")
            sys.exit(1)
        rclpy.init()
        _ros_node = FenNode()
        threading.Thread(target=_ros_thread_fn, args=(_ros_node,), daemon=True).start()
        print("✓ ROS node started")
        print("  Subscribing: /camera/image_raw, /perception/piece_debug")
        print("  Subscribing: /perception/changed_squares, /perception/square_scores")
        print("  Subscribing: /perception/reference_status")
        print("  Subscribing: /game_manager/{board_fen,state,turn}")
        print("  Publishing:  /limit_switch/clock_hit")
        print("  Service:     /perception/capture_premove")
    else:
        print("⚠  No-ROS mode — CLOCK HIT and Capture Ref buttons will be disabled")

    print(f"\n  Open in browser: http://localhost:{args.port}")
    print(f"  Network URL:     http://<pi-ip>:{args.port}\n")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
