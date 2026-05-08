#!/usr/bin/env python3
"""
Piece Detector Node — detects chess moves via frame differencing.

Detection Strategy:
  At the start of each player's turn, game_manager_node calls the
  /perception/capture_premove service. This stores the current warped board
  image as the "pre-move reference" frame (multi-frame average for stability).

  After the player moves and presses the clock, game_manager triggers a fresh
  camera capture. The piece detector compares the new frame to the pre-move
  reference using per-square LAB color-space absolute difference. Squares with
  a mean diff score above diff_threshold are flagged as changed.

  The list of changed square names (e.g. "e2,e4") is published on
  /perception/changed_squares. game_manager_node uses this list to find the
  unique legal move whose board footprint matches the changed squares — handling
  standard moves, captures, castling (4 squares), and en passant (3 squares)
  without needing any color or piece-type classification from vision.

Subscribed Topics:
  /camera/image_raw/compressed (CompressedImage) — JPEG camera stream
  /perception/board_geometry   (BoardState)      — board corners from board_detector

Published Topics:
  /perception/changed_squares  (String) — comma-separated changed square names, e.g. "e2,e4"
  /perception/piece_debug      (Image)  — diff heatmap overlay for debugging

Services:
  /perception/capture_premove  (Trigger) — store current board as pre-move reference

Parameters:
  diff_threshold     (float) — mean per-square LAB diff to flag as changed (default 18.0)
  premove_avg_count  (int)   — frames to average for a stable reference (default 1)
  warp_size          (int)   — warped board side length in pixels (default 400)
  detection_hz       (float) — processing rate in Hz (default 2.0)
"""

import json
import threading
from typing import Optional

import chess
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import SetParametersResult

from chess_interfaces.msg import BoardState


class PieceDetectorNode(Node):

    def __init__(self):
        super().__init__('piece_detector_node')

        self.declare_parameter('diff_threshold',          18.0)
        self.declare_parameter('premove_avg_count',       1)
        self.declare_parameter('warp_size',               400)
        self.declare_parameter('detection_hz',            2.0)
        self.declare_parameter('global_shift_compensation', 1.0)
        self.declare_parameter('clump_enable',              False)
        self.declare_parameter('clump_keep_per_group',      1)

        self._diff_threshold     = float(self.get_parameter('diff_threshold').value)
        self._premove_count      = int(self.get_parameter('premove_avg_count').value)
        self._warp_size          = int(self.get_parameter('warp_size').value)
        det_hz                   = float(self.get_parameter('detection_hz').value)
        self._shift_compensation = float(self.get_parameter('global_shift_compensation').value)
        self._clump_enable       = bool(self.get_parameter('clump_enable').value)
        self._clump_keep         = int(self.get_parameter('clump_keep_per_group').value)

        self._latest_msg:     Optional[object]      = None
        self._latest_image:   Optional[np.ndarray]  = None
        self._board_corners:  Optional[np.ndarray]  = None
        self._premove_warped: Optional[np.ndarray]  = None
        self._premove_frames: list                   = []
        self._lock = threading.Lock()

        self.create_subscription(
            CompressedImage, '/camera/image_raw/compressed',
            self._on_image_raw, 10)
        self.create_subscription(
            BoardState, '/perception/board_geometry',
            self._on_geometry, 10)

        self._changed_pub = self.create_publisher(
            String, '/perception/changed_squares', 10)
        self._debug_pub = self.create_publisher(
            Image, '/perception/piece_debug', 10)
        self._status_pub = self.create_publisher(
            String, '/perception/reference_status', 10)
        self._scores_pub = self.create_publisher(
            String, '/perception/square_scores', 10)

        self.create_service(
            Trigger, '/perception/capture_premove',
            self._srv_capture_premove)

        self.add_on_set_parameters_callback(self._on_params_changed)
        self.create_timer(1.0 / det_hz, self._detect_tick)

        self.get_logger().info(
            f'Piece Detector ready — frame-diff mode '
            f'(diff_threshold={self._diff_threshold}, '
            f'shift_compensation={self._shift_compensation}, '
            f'clump_enable={self._clump_enable}, '
            f'detection_hz={det_hz:.1f})'
        )

    # ── Subscriptions ──────────────────────────────────────────────────────────

    def _on_image_raw(self, msg):
        self._latest_msg = msg

    def _on_geometry(self, msg: BoardState):
        if len(msg.corners) == 4:
            pts = np.array([[p.x, p.y] for p in msg.corners], dtype='float32')
            with self._lock:
                self._board_corners = pts

    # ── Timer: detect and publish ──────────────────────────────────────────────

    def _detect_tick(self):
        msg = self._latest_msg
        if msg is None:
            return
        try:
            buf   = np.frombuffer(bytes(msg.data), dtype=np.uint8)
            frame = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            if frame is None:
                return
        except Exception as e:
            self.get_logger().error(f'Image decode error: {e}')
            return

        with self._lock:
            self._latest_image = frame
            corners  = self._board_corners.copy() if self._board_corners is not None else None
            premove  = self._premove_warped.copy() if self._premove_warped is not None else None

        if corners is None or premove is None:
            return

        warped = self._warp_board(frame, corners)
        if warped is None:
            return

        square_scores = self._compute_square_scores(warped, premove)

        # Publish raw per-square scores for the debug visualizer
        raw_map = {chess.square_name(sq): round(score, 1) for sq, score in square_scores}
        self._scores_pub.publish(String(data=json.dumps(raw_map)))

        # Global shift compensation: subtract (median × factor) from all scores.
        # If the whole camera vibrates, all 64 squares have similar diff → median is
        # high → after subtraction all adjusted scores ≈ 0 → no false detections.
        # If one piece moved, that square's raw score is much higher than the rest →
        # its adjusted score stays high and stands out clearly.
        if self._shift_compensation > 0.0:
            vals = [s for _, s in square_scores]
            floor = float(np.median(vals)) * self._shift_compensation
            adjusted = [(sq, max(0.0, s - floor)) for sq, s in square_scores]
        else:
            adjusted = square_scores

        changed = [sq for sq, score in adjusted if score > self._diff_threshold]
        if self._clump_enable and changed:
            changed = self._apply_clump_filter(changed, adjusted)

        sq_names = ','.join(chess.square_name(sq) for sq in changed)
        self._changed_pub.publish(String(data=sq_names))

        debug = self._draw_debug(warped, premove, adjusted, changed)
        self._publish_image(debug)

    # ── Pre-move Capture Service ───────────────────────────────────────────────

    def _srv_capture_premove(self, request, response):
        """
        Capture the current board as the pre-move reference frame.

        Called by game_manager_node when it's about to enter WAITING_PLAYER_MOVE
        (i.e. at game start and after the computer finishes each of its moves).

        The current frame is averaged with up to premove_avg_count previous
        captures for a stable reference. Each call resets the accumulation
        so the reference always reflects the current board state.
        """
        with self._lock:
            if self._latest_image is None or self._board_corners is None:
                response.success = False
                response.message = 'No image or board corners available'
                return response
            frame   = self._latest_image.copy()
            corners = self._board_corners.copy()

        warped = self._warp_board(frame, corners)
        if warped is None:
            response.success = False
            response.message = 'Perspective warp failed — check board corner calibration'
            return response

        # Reset accumulation on each service call so the reference is always fresh
        self._premove_frames = []
        self._premove_frames.append(warped.astype(np.float32))
        self._premove_warped = warped.copy()

        msg = f'Pre-move reference captured (1/{self._premove_count} frames)'
        self._status_pub.publish(String(data=msg))
        response.success = True
        response.message = msg
        self.get_logger().info(msg)
        return response

    # ── Parameter callback ─────────────────────────────────────────────────────

    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'diff_threshold':
                self._diff_threshold = float(p.value)
            elif p.name == 'global_shift_compensation':
                self._shift_compensation = float(p.value)
            elif p.name == 'clump_enable':
                self._clump_enable = bool(p.value)
            elif p.name == 'clump_keep_per_group':
                self._clump_keep = int(p.value)
        return SetParametersResult(successful=True)

    # ── Clump Filter ───────────────────────────────────────────────────────────

    def _apply_clump_filter(self, changed: list, scores: list) -> list:
        """Group adjacent flagged squares; keep only top _clump_keep per group.

        Perspective warp causes a piece at the far end of the board to bleed
        across multiple vertically-adjacent squares.  BFS 8-connectivity groups
        these into a single clump and keeps only the highest-scoring squares,
        dramatically reducing false detections from bleed while preserving
        isolated moves.  NOTE: castling spans 4 adjacent squares so it forms
        one clump — set clump_keep >= 4 or disable clump mode when castling
        detection is required.
        """
        score_map = {sq: score for sq, score in scores}
        remaining = set(changed)
        clumps = []
        while remaining:
            start = next(iter(remaining))
            clump, queue, seen = [], [start], set()
            while queue:
                sq = queue.pop(0)
                if sq in seen:
                    continue
                seen.add(sq)
                remaining.discard(sq)
                clump.append(sq)
                f, r = chess.square_file(sq), chess.square_rank(sq)
                for df in (-1, 0, 1):
                    for dr in (-1, 0, 1):
                        if df == 0 and dr == 0:
                            continue
                        nf, nr = f + df, r + dr
                        if 0 <= nf < 8 and 0 <= nr < 8:
                            nsq = chess.square(nf, nr)
                            if nsq in remaining:
                                queue.append(nsq)
            clumps.append(clump)
        result = []
        for clump in clumps:
            sorted_clump = sorted(clump, key=lambda sq: score_map.get(sq, 0.0), reverse=True)
            result.extend(sorted_clump[:self._clump_keep])
        return result

    # ── Warp ──────────────────────────────────────────────────────────────────

    def _warp_board(self, frame: np.ndarray, corners: np.ndarray) -> Optional[np.ndarray]:
        size = self._warp_size
        dst  = np.array([
            [0,        0       ],
            [size - 1, 0       ],
            [size - 1, size - 1],
            [0,        size - 1],
        ], dtype='float32')
        try:
            M = cv2.getPerspectiveTransform(corners, dst)
            return cv2.warpPerspective(frame, M, (size, size))
        except Exception as e:
            self.get_logger().debug(f'Warp error: {e}')
            return None

    # ── Frame Diff ──────────────────────────────────────────────────────────

    def _compute_square_scores(
        self,
        warped_cur: np.ndarray,
        warped_pre: np.ndarray,
    ) -> list:
        """
        Return list of (chess.Square, diff_score) for all 64 squares.

        diff_score is the mean per-pixel LAB distance within each square's
        interior region (with a small margin to avoid border artifacts).
        The L channel is weighted more heavily as it captures piece
        presence/absence reliably regardless of piece color.

        Squares are iterated in image order (row 0 = rank 8, row 7 = rank 1).
        """
        size   = self._warp_size
        sq_px  = size // 8
        margin = max(2, sq_px // 8)

        cur_lab = cv2.cvtColor(warped_cur, cv2.COLOR_BGR2LAB).astype(np.float32)
        pre_lab = cv2.cvtColor(warped_pre, cv2.COLOR_BGR2LAB).astype(np.float32)

        diff = np.abs(cur_lab - pre_lab)
        # L channel: piece presence/absence. a/b channels: color shifts (smaller weight)
        diff_map = diff[:, :, 0] + 0.4 * diff[:, :, 1] + 0.4 * diff[:, :, 2]

        results = []
        for row in range(8):
            for col in range(8):
                y1 = row * sq_px + margin
                y2 = (row + 1) * sq_px - margin
                x1 = col * sq_px + margin
                x2 = (col + 1) * sq_px - margin
                roi   = diff_map[y1:y2, x1:x2]
                score = float(np.mean(roi))
                # Image row 0 = rank 8 (top of board), row 7 = rank 1 (bottom)
                sq = chess.square(col, 7 - row)
                results.append((sq, score))

        return results

    # ── Debug Visualization ────────────────────────────────────────────────

    def _draw_debug(
        self,
        warped_cur: np.ndarray,
        warped_pre: np.ndarray,
        square_scores: list,
        changed: list,
    ) -> np.ndarray:
        size  = self._warp_size
        sq_px = size // 8

        # Diff heatmap blended over current frame
        cur_gray  = cv2.cvtColor(warped_cur, cv2.COLOR_BGR2GRAY)
        pre_gray  = cv2.cvtColor(warped_pre, cv2.COLOR_BGR2GRAY)
        diff_abs  = cv2.absdiff(cur_gray, pre_gray)
        diff_norm = cv2.normalize(diff_abs, None, 0, 255, cv2.NORM_MINMAX)
        heatmap   = cv2.applyColorMap(diff_norm, cv2.COLORMAP_INFERNO)
        debug     = cv2.addWeighted(warped_cur, 0.55, heatmap, 0.45, 0)

        # Grid lines
        for i in range(9):
            cv2.line(debug, (i * sq_px, 0), (i * sq_px, size), (80, 80, 80), 1)
            cv2.line(debug, (0, i * sq_px), (size, i * sq_px), (80, 80, 80), 1)

        # Per-square diff score annotation
        for sq, score in square_scores:
            col  = chess.square_file(sq)
            row  = 7 - chess.square_rank(sq)
            cx   = col * sq_px + sq_px // 2
            cy   = row * sq_px + sq_px // 3
            clr  = (0, 255, 0) if score > self._diff_threshold else (100, 100, 100)
            cv2.putText(debug, f'{score:.0f}', (cx - 10, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.28, clr, 1)

        # Changed squares: bright green rectangle + square name
        for sq in changed:
            col  = chess.square_file(sq)
            row  = 7 - chess.square_rank(sq)
            x1, y1 = col * sq_px, row * sq_px
            x2, y2 = x1 + sq_px, y1 + sq_px
            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(debug, chess.square_name(sq),
                        (x1 + 3, y2 - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, (0, 255, 0), 1)

        return debug

    # ── Publish Helpers ────────────────────────────────────────────────────

    def _publish_image(self, img: np.ndarray):
        msg             = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height      = img.shape[0]
        msg.width       = img.shape[1]
        msg.encoding    = 'bgr8'
        msg.is_bigendian = 0
        msg.step        = img.shape[1] * 3
        msg.data        = img.tobytes()
        self._debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PieceDetectorNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=None)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
