"""Shared mutable state for Chess OS: the `_state` dict, vision-tuning
`_params`, board-calibration file I/O, and the square-to-mm helper used
by the manual Square Nav debug tool (Gantry tab)."""

import json
import threading
import time
from pathlib import Path
from typing import Dict, List

# ── Paths ─────────────────────────────────────────────────────────────────────
_ROOT         = Path(__file__).resolve().parents[3]
_CALIB_FILE   = _ROOT / "board_calibration.json"
# Shared with board_detector_node, which loads this same file at its own
# startup so a manual override survives a node restart even without chess_ui
# running — see board_detector_node.py's _load_manual_corners_file().
# NOT named manual_corners.json: two stale tracked files with that name
# already exist (repo root and src/chess_perception/scripts/), leftover from
# an old, removed manual-corner feature, and store normalized 0-1
# coordinates rather than the full-resolution pixel coordinates this override
# actually needs — reusing that name would silently load garbage corners.
_CORNERS_FILE = _ROOT / "manual_corners.json"

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
    "vision": ["full", "corners", "board", "squares"],
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

# ── Shared state ──────────────────────────────────────────────────────────────
_lock = threading.Lock()

_state = {
    # Camera / vision
    "raw_frame":      None,
    "frame_count":    0,
    "last_updated":   0.0,
    "cam_info":       "–",
    "error":          "",
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
    # Exposure/white-balance lock — camera_ros's continuous auto-exposure/AWB
    # otherwise re-adjusts brightness/color between the pre-move reference and
    # post-move captures even with nothing physically moved, showing up as
    # board-wide "phantom" diff. Locking freezes it at whatever it had
    # converged to. Tracked here (not read back from ROS) because camera_ros's
    # ExposureTime/AnalogueGain/ColourGains parameters only report a real
    # value after being explicitly set at least once — AeEnable/AwbEnable
    # themselves are readable, but this avoids depending on that at all.
    "exposure_locked":            False,
    # Live status during game_manager_node's post-move stability wait, e.g.
    # "Stabilizing: 2/3 consistent reads" — see /game_manager/capture_progress.
    "capture_progress":           "",
    # Top-3 scored candidate legal moves from the last inference, e.g.
    # [{"uci":"e2e4","score":142.3}, ...] — see /game_manager/move_candidates.
    "move_candidates":            [],
    # Annotated corner-detection overlay from board_detector_node
    # (/perception/board_debug) — previously never displayed anywhere.
    "corner_frame":                None,
    # True once the manual corner-drag override has been applied (cleared
    # locally when "Clear" is pressed — board_detector_node is the actual
    # source of truth for whether an override is active).
    "manual_corners_active":       False,
    # The applied override's 4 [x,y] points (TL/TR/BR/BL, native camera-frame
    # pixels), persisted to manual_corners.json — lets the UI re-populate the
    # drag handles at the last-applied position on page load instead of
    # always resetting to the default inset rectangle.
    "manual_corners_points":       None,
}

# Diff detection params — pushed to piece_detector_node via SetParameters
_params = {
    "display_threshold":   18.0,
    "shift_compensation":   1.0,
    "clump_enable":          0,
    "clump_keep_per_group":  1,
    "roi_bias":              0.0,
    "edge_weight":           0.0,
}

# ── Jog state — background thread publishes velocity to /stepper/velocity ───────
# last_refresh is a watchdog heartbeat: the frontend re-POSTs jog/start
# periodically while a direction is held, and jog_loop() auto-stops if it
# hasn't heard one in JOG_WATCHDOG_TIMEOUT_S — otherwise a closed tab /
# dropped connection (no mouseup/keyup ever fires) would leave the gantry
# jogging indefinitely.
JOG_WATCHDOG_TIMEOUT_S = 0.8
_jog: dict = {"dx": 0.0, "dy": 0.0, "speed": 50.0, "active": False, "last_refresh": 0.0}
_jog_lock = threading.Lock()


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


def load_calibration():
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


def load_manual_corners():
    """Load manual_corners.json into _state at startup, if a manual override
    was previously applied and saved. Does NOT push it to board_detector_node
    over ROS — that node loads the same file independently at its own
    startup so the override applies whether or not chess_ui is running."""
    if not _CORNERS_FILE.exists():
        return None
    try:
        pts = json.loads(_CORNERS_FILE.read_text())
        if isinstance(pts, list) and len(pts) == 4:
            with _lock:
                _state["manual_corners_points"] = pts
                _state["manual_corners_active"] = True
            print(f"  ✓ Manual corners loaded from {_CORNERS_FILE}")
            return pts
    except Exception as e:
        print(f"  ⚠  Manual corners load failed: {e}")
    return None


def save_manual_corners(points):
    """Persist an applied manual-corner override so it survives restarts."""
    try:
        _CORNERS_FILE.write_text(json.dumps(points, indent=2))
    except Exception as e:
        print(f"  ⚠  Could not save manual corners: {e}")


def clear_manual_corners_file():
    """Remove the persisted override — the next restart resumes automatic
    detection instead of re-applying a corner set the user explicitly
    cleared."""
    try:
        if _CORNERS_FILE.exists():
            _CORNERS_FILE.unlink()
    except Exception as e:
        print(f"  ⚠  Could not clear manual corners file: {e}")


def best_fen() -> str:
    return _state.get("game_fen",
                      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1")
