#!/usr/bin/env python3
"""
Piece Detector Node — detects chess pieces from warped board image and
generates a FEN string representing the current board position.

Detection Strategy:
  1. Receive warped board image from board_detector_node via /perception/board_geometry
  2. Receive raw camera image from /camera/image_raw
  3. Apply perspective transform to get bird's-eye view of board (400×400px)
  4. For each of the 64 squares, analyze the center ROI:
       a. Occupancy: compare pixel difference vs a reference empty frame.
          A square is OCCUPIED if its pixel variance significantly exceeds
          the empty-board baseline.
  5. Color classification: use local brightness relative to board statistics
       to determine if an occupied piece is WHITE or BLACK.
  6. Game-state-assisted piece typing: subscribe to /game_manager/state_fen
     from game_manager and use the known board state to constrain piece
     type assignment. Because the game manager knows the authoritative
     board position, the detector ONLY needs to report changed squares.
  7. Reconstruct FEN from piece array and publish BoardState.

Subscribed Topics:
  /camera/image_raw          (Image)      — raw camera stream
  /perception/board_geometry (BoardState) — board corners (from board_detector)
  /game_manager/board_fen    (String)     — current authoritative FEN (from game_manager)

Published Topics:
  /perception/board_state    (BoardState) — detected FEN + piece array
  /perception/piece_debug    (Image)      — annotated warped board for debugging

Parameters:
  occupancy_diff_threshold (int)   — pixel diff threshold for piece detection (default 25)
  white_piece_brightness   (float) — brightness percentile above which a piece is white (0.65)
  reference_capture_count  (int)   — how many frames to average for baseline (default 5)
"""

import threading
from typing import Optional

import chess
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from chess_interfaces.msg import BoardState

# Index mapping: row 0 = rank 8 (black back row in image top),
# row 7 = rank 1 (white back row in image bottom).
# Warped image: row 0 = a8, ..., row 7 = a1
# col 0 = a-file, ..., col 7 = h-file

# python-chess piece type constants
PIECE_SYMBOLS = {
    chess.PAWN:   ('P', 'p'),
    chess.KNIGHT: ('N', 'n'),
    chess.BISHOP: ('B', 'b'),
    chess.ROOK:   ('R', 'r'),
    chess.QUEEN:  ('Q', 'q'),
    chess.KING:   ('K', 'k'),
}


class PieceDetectorNode(Node):

    def __init__(self):
        super().__init__('piece_detector_node')

        # ── Parameters ───────────────────────────────────────────────────
        self.declare_parameter('occupancy_diff_threshold', 25)
        self.declare_parameter('white_piece_brightness', 0.65)
        self.declare_parameter('reference_capture_count', 5)
        self.declare_parameter('warp_size', 400)

        self._occ_threshold   = self.get_parameter('occupancy_diff_threshold').value
        self._white_threshold = self.get_parameter('white_piece_brightness').value
        self._ref_count       = self.get_parameter('reference_capture_count').value
        self._warp_size       = self.get_parameter('warp_size').value

        # ── State ────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._latest_image:     Optional[np.ndarray] = None
        self._board_corners:    Optional[np.ndarray] = None
        self._reference_warped: Optional[np.ndarray] = None  # Empty-board baseline
        self._ref_frames:       list = []     # Accumulate for averaging
        self._authoritative_fen = chess.STARTING_FEN  # From game_manager
        self._lock = threading.Lock()

        # ── Subscribers ─────────────────────────────────────────────────
        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(
            BoardState, '/perception/board_geometry', self._on_geometry, 10)
        self.create_subscription(
            String, '/game_manager/board_fen', self._on_board_fen, 10)

        # ── Publishers ───────────────────────────────────────────────────
        self._state_pub = self.create_publisher(
            BoardState, '/perception/board_state', 10)
        self._debug_pub = self.create_publisher(
            Image, '/perception/piece_debug', 10)
        self._ref_status_pub = self.create_publisher(
            String, '/perception/reference_status', 10)

        # ── Services for reference capture ───────────────────────────────
        from std_srvs.srv import Trigger
        self.create_service(Trigger, '/perception/capture_reference',
                            self._srv_capture_reference)

        self.get_logger().info(
            'Piece Detector ready (occ_threshold=%d, white_thresh=%.2f)',
            self._occ_threshold, self._white_threshold
        )

    # ─────────────────────────────────────────────────────────────────────
    # Subscriptions
    # ─────────────────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        """Store latest raw frame and process if corners are available."""
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV bridge error: {e}')
            return

        with self._lock:
            self._latest_image = frame

        self._try_process()

    def _on_geometry(self, msg: BoardState):
        """Receive board corners from board_detector_node."""
        if len(msg.corners) == 4:
            pts = np.array([[p.x, p.y] for p in msg.corners], dtype='float32')
            with self._lock:
                self._board_corners = pts

    def _on_board_fen(self, msg: String):
        """Receive authoritative game FEN from game_manager_node."""
        with self._lock:
            self._authoritative_fen = msg.data

    # ─────────────────────────────────────────────────────────────────────
    # Reference Capture Service
    # ─────────────────────────────────────────────────────────────────────

    def _srv_capture_reference(self, request, response):
        """
        Capture a reference (empty board) baseline.

        Call this service ONCE before the game starts, with the board
        completely empty. This baseline is used for occupancy detection.
        Point the camera at the empty board and call:
          ros2 service call /perception/capture_reference std_srvs/srv/Trigger {}
        """
        with self._lock:
            if self._latest_image is None or self._board_corners is None:
                response.success = False
                response.message = 'No image or corners available'
                return response
            frame = self._latest_image.copy()
            corners = self._board_corners.copy()

        warped = self._warp_board(frame, corners)
        if warped is None:
            response.success = False
            response.message = 'Warp failed'
            return response

        # Accumulate frames for a stable reference
        self._ref_frames.append(warped.astype(np.float32))
        if len(self._ref_frames) > self._ref_count:
            self._ref_frames.pop(0)

        self._reference_warped = np.mean(self._ref_frames, axis=0).astype(np.uint8)
        self._ref_status_pub.publish(
            String(data=f'Reference captured ({len(self._ref_frames)}/{self._ref_count})'))
        response.success = True
        response.message = (
            f'Reference updated ({len(self._ref_frames)}/{self._ref_count} frames averaged). '
            f'Call again {"" if len(self._ref_frames) >= self._ref_count else "more times "}to improve stability.'
        )
        self.get_logger().info(response.message)
        return response

    # ─────────────────────────────────────────────────────────────────────
    # Main Processing Pipeline
    # ─────────────────────────────────────────────────────────────────────

    def _try_process(self):
        """Run the detection pipeline if all inputs are available."""
        with self._lock:
            if self._latest_image is None or self._board_corners is None:
                return
            frame   = self._latest_image.copy()
            corners = self._board_corners.copy()
            ref     = self._reference_warped.copy() if self._reference_warped is not None else None
            auth_fen = self._authoritative_fen

        # 1. Warp board to bird's-eye view
        warped = self._warp_board(frame, corners)
        if warped is None:
            return

        # 2. Analyse each square
        occupancy, colors = self._analyse_squares(warped, ref)

        # 3. Build FEN using occupancy, colors, and authoritative game state
        fen = self._build_fen(occupancy, colors, auth_fen)

        # 4. Build pieces array (64 integers) and publish BoardState
        pieces = self._build_pieces_array(occupancy, colors, auth_fen)
        self._publish_board_state(fen, pieces, frame.shape)

        # 5. Publish debug image
        debug = self._draw_debug(warped, occupancy, colors)
        self._debug_pub.publish(self._bridge.cv2_to_imgmsg(debug, 'bgr8'))

    # ─────────────────────────────────────────────────────────────────────
    # Perspective Warp
    # ─────────────────────────────────────────────────────────────────────

    def _warp_board(
        self, frame: np.ndarray, corners: np.ndarray
    ) -> Optional[np.ndarray]:
        """Warp the board region to a square bird's-eye view."""
        size = self._warp_size
        dst = np.array([
            [0,        0],
            [size - 1, 0],
            [size - 1, size - 1],
            [0,        size - 1],
        ], dtype='float32')
        try:
            M = cv2.getPerspectiveTransform(corners, dst)
            return cv2.warpPerspective(frame, M, (size, size))
        except Exception as e:
            self.get_logger().debug(f'Warp error: {e}')
            return None

    # ─────────────────────────────────────────────────────────────────────
    # Square Analysis
    # ─────────────────────────────────────────────────────────────────────

    def _analyse_squares(
        self,
        warped: np.ndarray,
        reference: Optional[np.ndarray],
    ):
        """
        Analyse all 64 squares.

        Returns:
          occupancy (list[bool]): True if square has a piece, indexed [0..63]
                                   index 0 = a8 (top-left in image), index 63 = h1.
          colors    (list[str]):  'W', 'B', or '' for each square.
        """
        size   = warped.shape[0]
        sq_px  = size // 8
        margin = sq_px // 6   # Ignore board square borders

        gray_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
        gray_ref    = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY) if reference is not None else None

        # Global board brightness statistics for relative color classification
        board_mean = float(np.mean(gray_warped))

        occupancy: list = []
        colors:    list = []

        for row in range(8):
            for col in range(8):
                y1 = row * sq_px + margin
                y2 = (row + 1) * sq_px - margin
                x1 = col * sq_px + margin
                x2 = (col + 1) * sq_px - margin

                roi_gray = gray_warped[y1:y2, x1:x2]

                # ── Occupancy ──────────────────────────────────────────
                if gray_ref is not None:
                    ref_roi = gray_ref[y1:y2, x1:x2]
                    diff = cv2.absdiff(roi_gray, ref_roi)
                    occupied = int(np.mean(diff)) > self._occ_threshold
                else:
                    # No reference: fall back to variance-based detection
                    # High variance within a square suggests a piece boundary
                    std_val = float(np.std(roi_gray))
                    occupied = std_val > 20.0

                occupancy.append(occupied)

                # ── Color classification ───────────────────────────────
                if occupied:
                    roi_mean = float(np.mean(roi_gray))
                    # A bright piece is white; dark piece is black.
                    # Compare against board mean scaled by threshold.
                    is_white = roi_mean > (board_mean * (1.0 + (self._white_threshold - 0.5)))
                    colors.append('W' if is_white else 'B')
                else:
                    colors.append('')

        return occupancy, colors

    # ─────────────────────────────────────────────────────────────────────
    # FEN Construction
    # ─────────────────────────────────────────────────────────────────────

    def _build_fen(
        self,
        occupancy: list,
        colors: list,
        auth_fen: str,
    ) -> str:
        """
        Construct a FEN string using occupancy/color detection plus the
        authoritative game state from game_manager_node.

        Strategy:
          - For each square, determine if it's occupied (W/B/empty).
          - Use the authoritative python-chess board to look up the piece
            type on each occupied square. This means PIECE TYPE comes from
            the known game state; only OCCUPANCY and COLOR are derived from vision.
          - If vision reports a change that doesn't match the game state,
            log a warning but use the visually detected occupancy.

        This handles the limitation that pure color classification can't tell
        a rook from a queen — the game state constrains what piece is where.
        """
        try:
            auth_board = chess.Board(auth_fen)
        except Exception:
            auth_board = chess.Board()

        rows = []
        # Board image: row 0 = rank 8 (index 7 in chess notation top row)
        for rank_idx in range(7, -1, -1):
            empty_run = 0
            row_str   = ''
            for file_idx in range(8):
                # Convert rank_idx/file_idx to image row/col
                # Image row 0 = rank 8, image row 7 = rank 1
                # Image col 0 = a-file
                img_row = 7 - rank_idx
                img_col = file_idx
                arr_idx = img_row * 8 + img_col

                sq = chess.square(file_idx, rank_idx)
                occ  = occupancy[arr_idx]
                col  = colors[arr_idx]

                if not occ:
                    # Visually empty
                    empty_run += 1
                else:
                    if empty_run:
                        row_str   += str(empty_run)
                        empty_run = 0

                    # Try to get piece from authoritative board first
                    auth_piece = auth_board.piece_at(sq)
                    if auth_piece:
                        symbol = auth_piece.symbol()  # uppercase=white, lowercase=black
                    else:
                        # Not on auth board — guess generic piece
                        symbol = 'P' if col == 'W' else 'p'
                        self.get_logger().debug(
                            f'Vision sees piece at {chess.square_name(sq)} '
                            f'but auth board is empty — using fallback symbol {symbol}')

                    row_str += symbol

            if empty_run:
                row_str += str(empty_run)
            rows.append(row_str)

        position = '/'.join(rows)

        # Preserve turn, castling, en passant from authoritative FEN
        auth_parts = auth_fen.split(' ')
        suffix = ' '.join(auth_parts[1:]) if len(auth_parts) > 1 else 'w KQkq - 0 1'
        return f'{position} {suffix}'

    def _build_pieces_array(
        self,
        occupancy: list,
        colors: list,
        auth_fen: str,
    ) -> list:
        """
        Build the 64-element pieces array for BoardState.pieces.
        Uses the same game-state assisted logic as _build_fen().
        Values: 0=empty, +N=white piece type, -N=black piece type
        (matching the existing BoardState encoding from CONTEXT.md)
        """
        try:
            auth_board = chess.Board(auth_fen)
        except Exception:
            auth_board = chess.Board()

        PIECE_ID = {
            chess.PAWN: 1, chess.KNIGHT: 2, chess.BISHOP: 3,
            chess.ROOK: 4, chess.QUEEN: 5, chess.KING: 6,
        }

        pieces = []
        for rank_idx in range(7, -1, -1):
            for file_idx in range(8):
                img_row = 7 - rank_idx
                img_col = file_idx
                arr_idx = img_row * 8 + img_col

                sq = chess.square(file_idx, rank_idx)
                if not occupancy[arr_idx]:
                    pieces.append(0)
                    continue

                auth_piece = auth_board.piece_at(sq)
                if auth_piece:
                    pid = PIECE_ID.get(auth_piece.piece_type, 1)
                    pieces.append(pid if auth_piece.color == chess.WHITE else -pid)
                else:
                    pieces.append(1 if colors[arr_idx] == 'W' else -1)

        return pieces

    # ─────────────────────────────────────────────────────────────────────
    # Debug Visualization
    # ─────────────────────────────────────────────────────────────────────

    def _draw_debug(
        self,
        warped: np.ndarray,
        occupancy: list,
        colors: list,
    ) -> np.ndarray:
        """Annotate warped board with occupancy/color grid."""
        size   = warped.shape[0]
        sq_px  = size // 8
        debug  = warped.copy()

        for row in range(8):
            for col in range(8):
                arr_idx = row * 8 + col
                x1, y1  = col * sq_px, row * sq_px
                x2, y2  = x1 + sq_px, y1 + sq_px

                if occupancy[arr_idx]:
                    color_bgr = (0, 255, 0) if colors[arr_idx] == 'W' else (0, 0, 255)
                    cv2.rectangle(debug, (x1, y1), (x2, y2), color_bgr, 2)
                    label = 'W' if colors[arr_idx] == 'W' else 'B'
                    cv2.putText(debug, label,
                                (x1 + sq_px // 4, y1 + sq_px // 2),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color_bgr, 1)

        # Draw grid
        for i in range(9):
            cv2.line(debug, (i * sq_px, 0), (i * sq_px, size), (200, 200, 200), 1)
            cv2.line(debug, (0, i * sq_px), (size, i * sq_px), (200, 200, 200), 1)

        return debug

    # ─────────────────────────────────────────────────────────────────────
    # Publish
    # ─────────────────────────────────────────────────────────────────────

    def _publish_board_state(self, fen: str, pieces: list, _frame_shape):
        msg = BoardState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.fen   = fen
        msg.pieces = [int(p) for p in pieces]
        self._state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PieceDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
