#!/usr/bin/env python3
"""
Motion Planner Node — converts UCI chess moves into gantry sequences.

Receives commands on /motion/command as "UCI FEN" strings.
Uses the FEN to understand the full board position and execute
the correct multi-step sequence for:
  - Normal moves (pick-and-place with corner-based obstacle routing)
  - Captures (remove captured piece to graveyard first)
  - Castling (move king, then rook)
  - En passant (remove captured pawn, then move attacker)
  - Pawn promotion (move pawn, publish PROMOTION_WAIT if computer promotes)

Coordinate System:
  Origin (0,0): bottom-left corner (from player's perspective)
  +X = RIGHTWARD (toward h-file, toward X limit switch)
  +Y = BACKWARD / UPWARD (toward rank 8 / black's side)

  a-file (col 0): X = board_origin_x_mm
  h-file (col 7): X = board_origin_x_mm + 7 * square_size_mm
  rank 1 (row 0): Y = board_origin_y_mm
  rank 8 (row 7): Y = board_origin_y_mm + 7 * square_size_mm

Graveyard Routing (capture routing):
  Captured pieces are routed via a safe corridor outside the board
  to a graveyard zone, one slot per captured piece.
  Routing: source → board right edge (safe_x) → graveyard Y → slot X → drop.

Corner-Based Obstacle Routing:
  When carrying a piece, the gantry routes through "corner points"
  (intersections of file/rank boundary lines) to avoid disturbing
  other pieces. BFS on the 9×9 corner grid finds the shortest safe path.

Topics:
  Subscribes: /motion/command   (String) — "UCI FEN" move command
  Subscribes: /motion/abort     (Bool)   — True to cancel in-progress move
  Publishes:  /motion/done      (Bool)   — True on success, False on failure

Action Client:
  /gantry/move (chess_interfaces/action/MoveGantry) — move gantry to XY

Service Clients:
  /servo/engage  (Trigger) — lower magnet to board
  /servo/release (Trigger) — raise magnet clear of board
"""

from collections import deque
import time
from typing import List, Optional, Tuple

import chess
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from chess_interfaces.action import MoveGantry


# ─────────────────────────────────────────────────────────────────────────────
# Column letter → 0-indexed column number
# ─────────────────────────────────────────────────────────────────────────────
COL_MAP = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}


# ─────────────────────────────────────────────────────────────────────────────
# Graveyard Manager
# ─────────────────────────────────────────────────────────────────────────────

class GraveyardManager:
    """
    Tracks available slots in the graveyard zone (off the right side of the board).

    Slots fill from origin_x leftward (decreasing X) across each row,
    then move to the next row (increasing Y).
    """

    def __init__(
        self,
        origin_x_mm: float = 230.0,   # X of first slot (right side)
        origin_y_mm: float = 215.0,   # Y of graveyard row (above rank 8)
        slot_spacing_mm: float = 22.0,
        cols: int = 8,
        row_spacing_mm: float = 22.0,
    ):
        self._origin_x   = origin_x_mm
        self._origin_y   = origin_y_mm
        self._spacing    = slot_spacing_mm
        self._cols       = cols
        self._row_spacing = row_spacing_mm
        self._next_slot  = 0

    def reset(self):
        self._next_slot = 0

    def get_next_slot(self) -> Tuple[float, float]:
        """
        Get the (x_mm, y_mm) of the next available graveyard slot.
        Slots fill leftward (decreasing X) across each row, then move
        to the next row (increasing Y).
        """
        slot = self._next_slot
        self._next_slot += 1
        row = slot // self._cols
        col = slot % self._cols
        x = self._origin_x - col * self._spacing
        y = self._origin_y + row * self._row_spacing
        return x, y

    @property
    def slots_used(self) -> int:
        return self._next_slot


# ─────────────────────────────────────────────────────────────────────────────
# Motion Planner Node
# ─────────────────────────────────────────────────────────────────────────────

class MotionPlannerNode(Node):

    def __init__(self):
        super().__init__('motion_planner_node')

        # ── Parameters ─────────────────────────────────────────────────────
        # Coordinate system: origin at bottom-left (player's perspective)
        # +X = rightward toward h-file / X limit switch
        # +Y = backward toward rank 8 / camera tower
        self.declare_parameter('square_size_mm', 25.0)
        self.declare_parameter('board_origin_x_mm', 20.0)   # X of a1 center
        self.declare_parameter('board_origin_y_mm', 20.0)   # Y of a1 center
        self.declare_parameter('move_speed_mm_s', 50.0)

        # Safe routing corridor: X coordinate RIGHT of h-file (outside board)
        # Used to route captures out of the board without touching any pieces
        self.declare_parameter('board_edge_safe_x_mm', 230.0)

        # Graveyard: zone where captured pieces are deposited
        self.declare_parameter('graveyard_origin_x_mm', 230.0)
        self.declare_parameter('graveyard_origin_y_mm', 215.0)
        self.declare_parameter('graveyard_slot_spacing_mm', 22.0)
        self.declare_parameter('graveyard_cols', 8)

        self.sq_size     = self.get_parameter('square_size_mm').value
        self.origin_x    = self.get_parameter('board_origin_x_mm').value
        self.origin_y    = self.get_parameter('board_origin_y_mm').value
        self.move_speed  = self.get_parameter('move_speed_mm_s').value
        self.edge_x      = self.get_parameter('board_edge_safe_x_mm').value

        self._graveyard = GraveyardManager(
            origin_x_mm    = self.get_parameter('graveyard_origin_x_mm').value,
            origin_y_mm    = self.get_parameter('graveyard_origin_y_mm').value,
            slot_spacing_mm= self.get_parameter('graveyard_slot_spacing_mm').value,
            cols           = self.get_parameter('graveyard_cols').value,
        )

        # ── Publishers ──────────────────────────────────────────────────────
        self._done_pub  = self.create_publisher(Bool, '/motion/done', 10)

        # ── Abort flag (set by /motion/abort subscription) ──────────────────
        self._abort_requested = False

        # ── Action client ───────────────────────────────────────────────────
        self._gantry_client = ActionClient(self, MoveGantry, '/gantry/move')

        # ── Service clients ─────────────────────────────────────────────────
        self._servo_engage  = self.create_client(Trigger, '/servo/engage')
        self._servo_release = self.create_client(Trigger, '/servo/release')

        # ── Subscriptions ────────────────────────────────────────────────
        # Format: "UCI FEN"  e.g.  "e2e4 rnbqkbnr/.../w KQkq - 0 1"
        self.create_subscription(String, '/motion/command', self._command_cb, 10)
        self.create_subscription(Bool, '/motion/abort', self._on_abort, 10)

        self.get_logger().info(
            f'Motion Planner ready — '
            f'a1=({self.origin_x}, {self.origin_y})mm, '
            f'sq={self.sq_size}mm, speed={self.move_speed}mm/s, '
            f'safe_edge_x={self.edge_x}mm')

    # ─────────────────────────────────────────────────────────────────────
    # Command Handling
    # ─────────────────────────────────────────────────────────────────────

    def _on_abort(self, msg: Bool):
        """Receive abort signal from game_manager — stop gantry movement immediately."""
        if msg.data:
            self._abort_requested = True
            self.get_logger().warn('Motion abort requested')

    def _command_cb(self, msg: String):
        """
        Receive 'UCI FEN' command and execute the appropriate move sequence.

        Format:  "e2e4 rnbqkbnr/pppppppp/.../RNBQKBNR w KQkq - 0 1"
        """
        self._abort_requested = False  # clear any stale abort from previous new-game
        parts = msg.data.strip().split(' ', 1)
        if len(parts) < 1 or len(parts[0]) < 4:
            self.get_logger().error(f'Invalid command format: {msg.data!r}')
            self._publish_done(False)
            return

        uci = parts[0]
        fen = parts[1] if len(parts) > 1 else chess.STARTING_FEN

        try:
            board = chess.Board(fen)
        except ValueError as e:
            self.get_logger().error(f'Invalid FEN in command: {e}')
            self._publish_done(False)
            return

        try:
            move = chess.Move.from_uci(uci)
        except ValueError:
            self.get_logger().error(f'Invalid UCI: {uci!r}')
            self._publish_done(False)
            return

        self.get_logger().info(f'Executing move: {uci}  ({board.fen().split(" ")[0]})')

        try:
            success = self._execute_move(move, board)
            self._publish_done(success)
        except Exception as e:
            self.get_logger().error(f'Move execution failed: {e}')
            self._call_servo(self._servo_release)
            self._publish_done(False)

    # ─────────────────────────────────────────────────────────────────────
    # Move Classification and Execution
    # ─────────────────────────────────────────────────────────────────────

    def _execute_move(self, move: chess.Move, board: chess.Board) -> bool:
        is_capture    = board.is_capture(move)
        is_castling   = board.is_castling(move)
        is_en_passant = board.is_en_passant(move)
        is_promotion  = move.promotion is not None

        self.get_logger().info(
            f'  capture={is_capture} castle={is_castling} '
            f'ep={is_en_passant} promo={is_promotion}')

        if is_castling:
            return self._execute_castling(move, board)

        if is_en_passant:
            return self._execute_en_passant(move, board)

        if is_capture:
            ok = self._capture_piece_to_graveyard(move.to_square, board)
            if not ok:
                return False

        ok = self._pick_and_place_square(move.from_square, move.to_square, board)
        if not ok:
            return False

        return True

    # ─────────────────────────────────────────────────────────────────────
    # Special Move Implementations
    # ─────────────────────────────────────────────────────────────────────

    def _execute_castling(self, move: chess.Move, board: chess.Board) -> bool:
        king_from = move.from_square
        king_to   = move.to_square

        if board.is_kingside_castling(move):
            rook_from = chess.H1 if board.turn == chess.WHITE else chess.H8
            rook_to   = chess.F1 if board.turn == chess.WHITE else chess.F8
        else:
            rook_from = chess.A1 if board.turn == chess.WHITE else chess.A8
            rook_to   = chess.D1 if board.turn == chess.WHITE else chess.D8

        self.get_logger().info(
            f'Castling: King {chess.square_name(king_from)}→{chess.square_name(king_to)}, '
            f'Rook {chess.square_name(rook_from)}→{chess.square_name(rook_to)}')

        ok = self._pick_and_place_square(king_from, king_to, board)
        if not ok:
            return False
        return self._pick_and_place_square(rook_from, rook_to, board)

    def _execute_en_passant(self, move: chess.Move, board: chess.Board) -> bool:
        src = move.from_square
        dst = move.to_square

        # Captured pawn is on the same rank as the attacker, on the dest file
        src_rank    = chess.square_rank(src)
        dst_file    = chess.square_file(dst)
        captured_sq = chess.square(dst_file, src_rank)

        self.get_logger().info(
            f'En passant: {chess.square_name(src)}→{chess.square_name(dst)}, '
            f'capturing pawn at {chess.square_name(captured_sq)}')

        ok = self._capture_piece_to_graveyard(captured_sq, board)
        if not ok:
            return False
        return self._pick_and_place_square(src, dst, board)

    # ─────────────────────────────────────────────────────────────────────
    # Graveyard Routing
    # ─────────────────────────────────────────────────────────────────────

    def _capture_piece_to_graveyard(
        self,
        square: chess.Square,
        board: chess.Board,
    ) -> bool:
        """
        Lift piece from `square` and carry it to the next graveyard slot.

        Safe routing (magnet engaged, avoids board pieces):
          1. Engage magnet at source square
          2. Move to board_edge_safe_x at same Y → exits all board squares horizontally
          3. Move to graveyard Y at same X → moves vertically outside board area
          4. Move to slot X → final horizontal move to graveyard slot
          5. Release magnet
        """
        slot_x, slot_y = self._graveyard.get_next_slot()
        src_x,  src_y  = self._square_to_mm(chess.square_name(square))

        self.get_logger().info(
            f'  Removing piece from {chess.square_name(square)} '
            f'to graveyard slot ({slot_x:.0f}, {slot_y:.0f})mm')

        # 1. Move to piece square
        ok = self._gantry_move(src_x, src_y)
        if not ok:
            return False

        # 2. Engage magnet
        self._call_servo(self._servo_engage)

        # 3. Move to safe edge (outside all board squares, same Y)
        ok = self._gantry_move(self.edge_x, src_y)
        if not ok:
            self._call_servo(self._servo_release)
            return False

        # 4. Move vertically to graveyard Y (outside board in X, safe corridor)
        ok = self._gantry_move(self.edge_x, slot_y)
        if not ok:
            self._call_servo(self._servo_release)
            return False

        # 5. Move horizontally to graveyard slot X
        ok = self._gantry_move(slot_x, slot_y)
        if not ok:
            self._call_servo(self._servo_release)
            return False

        # 6. Release magnet
        self._call_servo(self._servo_release)
        return True

    # ─────────────────────────────────────────────────────────────────────
    # Standard Pick-and-Place with Corner Routing
    # ─────────────────────────────────────────────────────────────────────

    def _pick_and_place_square(
        self,
        src_sq: chess.Square,
        dst_sq: chess.Square,
        board: Optional[chess.Board] = None,
    ) -> bool:
        """
        Pick piece from src_sq and place it on dst_sq.

        When board is provided, uses corner-based obstacle routing to
        avoid bumping adjacent pieces while carrying the magnet-engaged piece.
        """
        src_x, src_y = self._square_to_mm(chess.square_name(src_sq))
        dst_x, dst_y = self._square_to_mm(chess.square_name(dst_sq))

        self.get_logger().info(
            f'  Pick-and-place: {chess.square_name(src_sq)}({src_x:.0f},{src_y:.0f})'
            f' → {chess.square_name(dst_sq)}({dst_x:.0f},{dst_y:.0f})')

        # 1. Move to source square (magnet up, no routing needed)
        ok = self._gantry_move(src_x, src_y)
        if not ok:
            return False

        # 2. Engage magnet
        self._call_servo(self._servo_engage)

        # 3. Navigate to destination via obstacle-avoiding path
        if board is not None:
            waypoints = self._route_via_corner(src_sq, dst_sq, board)
            # waypoints[0] is src center (already there), skip it
            for wx, wy in waypoints[1:]:
                ok = self._gantry_move(wx, wy)
                if not ok:
                    self._call_servo(self._servo_release)
                    return False
        else:
            ok = self._gantry_move(dst_x, dst_y)
            if not ok:
                self._call_servo(self._servo_release)
                return False

        # 4. Release magnet
        self._call_servo(self._servo_release)
        return True

    # ─────────────────────────────────────────────────────────────────────
    # Corner-Based Obstacle Routing
    # ─────────────────────────────────────────────────────────────────────

    def _route_via_corner(
        self,
        src_sq: chess.Square,
        dst_sq: chess.Square,
        board: chess.Board,
    ) -> List[Tuple[float, float]]:
        """
        Generate a collision-free path from src_sq center to dst_sq center.

        Returns a list of (x_mm, y_mm) waypoints:
          [src_center, corner_1, corner_2, ..., corner_n, dst_center]

        The gantry piece fits within half a square. By navigating through
        CORNER POINTS (where 4 squares meet), the magnet stays at distance
        sq_size/2 from each adjacent square center. This avoids disturbing
        pieces in those squares.

        Corner grid: (ci, cj) where ci, cj ∈ {0, 1, ..., 8}
          ci=0: left of a-file, ci=8: right of h-file
          cj=0: below rank 1,   cj=8: above rank 8

        A corner (ci, cj) is adjacent to up to 4 squares:
          file = ci-1 or ci, rank = cj-1 or cj (within 0..7)

        A corner is SAFE to visit if none of its adjacent squares (excluding
        the source and destination squares) is occupied.

        Search strategy: orthogonal-only BFS from the 4 corners of src_sq
        to any of the 4 corners of dst_sq.
        Falls back to direct path if no safe corner path exists.
        """
        src_f = chess.square_file(src_sq)
        src_r = chess.square_rank(src_sq)
        dst_f = chess.square_file(dst_sq)
        dst_r = chess.square_rank(dst_sq)

        src_mm = self._square_to_mm(chess.square_name(src_sq))
        dst_mm = self._square_to_mm(chess.square_name(dst_sq))

        # Build occupied set (excluding src and dst — those squares are involved in the move)
        occupied: set = set()
        for sq in chess.SQUARES:
            if sq != src_sq and sq != dst_sq and board.piece_at(sq):
                occupied.add((chess.square_file(sq), chess.square_rank(sq)))

        # If no other pieces, direct path is always safe
        if not occupied:
            return [src_mm, dst_mm]

        def corner_safe(ci: int, cj: int) -> bool:
            """Return True if no adjacent occupied square blocks this corner."""
            for df, dr in [(-1, -1), (0, -1), (-1, 0), (0, 0)]:
                f = ci + df
                r = cj + dr
                if 0 <= f <= 7 and 0 <= r <= 7 and (f, r) in occupied:
                    return False
            return True

        def corner_to_mm(ci: int, cj: int) -> Tuple[float, float]:
            """Convert corner grid index to mm coordinates."""
            half = self.sq_size / 2.0
            x = self.origin_x + ci * self.sq_size - half
            y = self.origin_y + cj * self.sq_size - half
            return (x, y)

        # 4 corners of src square (always accessible — magnet lifts straight up)
        start_corners = {
            (src_f, src_r), (src_f + 1, src_r),
            (src_f, src_r + 1), (src_f + 1, src_r + 1),
        }
        # 4 corners of dst square
        end_corners = {
            (dst_f, dst_r), (dst_f + 1, dst_r),
            (dst_f, dst_r + 1), (dst_f + 1, dst_r + 1),
        }

        # BFS — orthogonal moves only (horizontal / vertical between adjacent corners)
        queue: deque = deque()
        visited: dict = {}

        for sc in start_corners:
            if corner_safe(sc[0], sc[1]):
                queue.append((sc, [sc]))
                visited[sc] = None

        if not queue:
            # All src corners blocked — fall back to direct path
            self.get_logger().warn(
                f'All source corners blocked for {chess.square_name(src_sq)}, using direct path')
            return [src_mm, dst_mm]

        path_corners: Optional[list] = None

        while queue:
            (ci, cj), path = queue.popleft()

            if (ci, cj) in end_corners:
                path_corners = path
                break

            # Explore orthogonal neighbours only
            for ni, nj in [(ci - 1, cj), (ci + 1, cj),
                           (ci, cj - 1), (ci, cj + 1)]:
                if 0 <= ni <= 8 and 0 <= nj <= 8 and (ni, nj) not in visited:
                    if corner_safe(ni, nj):
                        visited[(ni, nj)] = (ci, cj)
                        queue.append(((ni, nj), path + [(ni, nj)]))

        if path_corners is None:
            # No safe path found — fall back to direct
            self.get_logger().warn(
                f'No safe corner path {chess.square_name(src_sq)}→'
                f'{chess.square_name(dst_sq)}, using direct path')
            return [src_mm, dst_mm]

        waypoints = [src_mm]
        for ci, cj in path_corners:
            waypoints.append(corner_to_mm(ci, cj))
        waypoints.append(dst_mm)

        if len(path_corners) > 2:
            self.get_logger().info(
                f'  Corner route: {len(path_corners)} waypoints for '
                f'{chess.square_name(src_sq)}→{chess.square_name(dst_sq)}')

        return waypoints

    # ─────────────────────────────────────────────────────────────────────
    # Coordinate Math
    # ─────────────────────────────────────────────────────────────────────

    def _square_to_mm(self, square: str) -> Tuple[float, float]:
        """
        Convert a chess square name (e.g. 'e4') to gantry XY in mm.

        Coordinate system (origin at bottom-left from player's view):
          +X = rightward toward h-file / X limit switch
          +Y = backward toward rank 8

        Formula:
          x = board_origin_x_mm + col_index * square_size_mm
          y = board_origin_y_mm + rank_index * square_size_mm
          where a=0, b=1, ..., h=7  and  rank1=0, rank8=7
        """
        col = COL_MAP.get(square[0].lower())
        row = int(square[1]) - 1
        if col is None or not (0 <= row <= 7):
            raise ValueError(f'Invalid square: {square!r}')
        x = self.origin_x + col * self.sq_size
        y = self.origin_y + row * self.sq_size
        return x, y

    # ─────────────────────────────────────────────────────────────────────
    # Gantry Action Client
    # ─────────────────────────────────────────────────────────────────────

    def _gantry_move(
        self,
        x_mm: float,
        y_mm: float,
        speed: Optional[float] = None,
        timeout: float = 30.0,
    ) -> bool:
        if self._abort_requested:
            self.get_logger().warn('Move skipped — abort requested')
            return False
        if not self._gantry_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/gantry/move action server not available')
            return False

        goal = MoveGantry.Goal()
        goal.target_x_mm   = float(x_mm)
        goal.target_y_mm   = float(y_mm)
        goal.speed_mm_s    = float(speed) if speed else self.move_speed
        goal.engage_magnet = False  # Servo service controls magnet separately

        goal_future = self._gantry_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(f'Gantry goal ({x_mm:.0f},{y_mm:.0f}) rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.get_logger().error(f'Gantry move ({x_mm:.0f},{y_mm:.0f}) timed out')
            return False

        result = result_future.result()
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        if not success:
            self.get_logger().error(f'Gantry move failed: {result.result.message}')
        return success

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'  Gantry: {fb.percent_complete:.0f}%  '
            f'({fb.current_x_mm:.1f}, {fb.current_y_mm:.1f})mm')

    # ─────────────────────────────────────────────────────────────────────
    # Servo Helpers
    # ─────────────────────────────────────────────────────────────────────

    def _call_servo(self, client, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn('Servo service not available — skipping')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        result = future.result()
        if result and not result.success:
            self.get_logger().warn(f'Servo call: {result.message}')

    # ─────────────────────────────────────────────────────────────────────
    # Publishing Helpers
    # ─────────────────────────────────────────────────────────────────────

    def _publish_done(self, success: bool):
        self._done_pub.publish(Bool(data=success))
        self.get_logger().info(f'Move {"complete" if success else "FAILED"}')


def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
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
