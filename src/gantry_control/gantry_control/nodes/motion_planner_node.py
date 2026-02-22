#!/usr/bin/env python3
"""
Motion Planner Node — converts UCI chess moves into gantry sequences.

Receives commands on /motion/command as "UCI FEN" strings.
Uses the FEN to understand the full board position and execute
the correct multi-step sequence for:
  - Normal moves (pick-and-place)
  - Captures (remove captured piece to graveyard first)
  - Castling (move king, then rook)
  - En passant (remove captured pawn, then move attacker)
  - Pawn promotion (move pawn, publish PROMOTION_WAIT if computer promotes)

Coordinate System:
  Origin (0,0): bottom-right corner (homing position)
  +X = LEFT (toward a-file)
  +Y = UP/BACK (toward rank 8 / black's side)

Graveyard Routing:
  Captured pieces are moved FIRST to the board's right edge (X=5mm),
  then UP past rank 8 to the graveyard zone (Y=215mm), then to the
  next available slot. This clears all board pieces in two straight moves.

Topics:
  Subscribes: /motion/command   (String) — "UCI FEN" move command
  Publishes:  /motion/done      (Bool)   — True on success, False on failure
  Publishes:  /game_manager/state (String) — PROMOTION_WAIT during promotion

Action Client:
  /gantry/move (chess_interfaces/action/MoveGantry) — move gantry to XY

Service Clients:
  /servo/engage  (Trigger) — lower magnet to board
  /servo/release (Trigger) — raise magnet clear of board
"""

import time
from typing import Optional, Tuple, List

import chess
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from chess_interfaces.action import MoveGantry


# ─────────────────────────────────────────────────────────────────────────────
# Column letter to 0-indexed column
# ─────────────────────────────────────────────────────────────────────────────
COL_MAP = {'a': 0, 'b': 1, 'c': 2, 'd': 3, 'e': 4, 'f': 5, 'g': 6, 'h': 7}

# Board edge X coordinate (outside the board, safe for routing)
BOARD_EDGE_SAFE_X_MM = 5.0   # Far RIGHT, outside board (all squares are X >= 25mm)


# ─────────────────────────────────────────────────────────────────────────────
# Graveyard Manager
# ─────────────────────────────────────────────────────────────────────────────

class GraveyardManager:
    """
    Tracks available slots in the graveyard zone (behind black's pieces).

    Layout: all captured pieces go to the same graveyard zone, behind rank 8.
    Slots are arranged in rows spreading horizontally from the left side.

    Parameters are loaded from motion_planner_node ROS params (board_map.yaml).
    """

    def __init__(
        self,
        origin_x_mm: float = 210.0,    # X of first slot (near a-file side)
        origin_y_mm: float = 215.0,    # Y of graveyard row
        slot_spacing_mm: float = 22.0, # X distance between slots
        cols: int = 8,                 # Slots per row
        row_spacing_mm: float = 22.0,  # Y distance between rows
    ):
        self._origin_x = origin_x_mm
        self._origin_y = origin_y_mm
        self._spacing = slot_spacing_mm
        self._cols = cols
        self._row_spacing = row_spacing_mm
        self._next_slot = 0             # Linear index across all slots

    def reset(self):
        """Reset graveyard (call at game start)."""
        self._next_slot = 0

    def get_next_slot(self) -> Tuple[float, float]:
        """
        Get the (x_mm, y_mm) of the next available graveyard slot.

        Slots fill left-to-right (decreasing X) across each row, then
        move to the next row (increasing Y).
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

        # ── Parameters (loaded from board_map.yaml) ────────────────────────
        # Coordinate system: origin at bottom-right (homing position)
        # +X = LEFT (toward a-file), +Y = UP (toward rank 8 / black's side)
        self.declare_parameter('square_size_mm', 25.0)
        self.declare_parameter('board_origin_x_mm', 200.0)   # X of center of a1
        self.declare_parameter('board_origin_y_mm', 20.0)    # Y of center of a1
        self.declare_parameter('move_speed_mm_s', 50.0)
        self.declare_parameter('graveyard_origin_x_mm', 210.0)
        self.declare_parameter('graveyard_origin_y_mm', 215.0)
        self.declare_parameter('graveyard_slot_spacing_mm', 22.0)
        self.declare_parameter('graveyard_cols', 8)
        self.declare_parameter('board_edge_safe_x_mm', 5.0)

        self.sq_size    = self.get_parameter('square_size_mm').value
        self.origin_x   = self.get_parameter('board_origin_x_mm').value
        self.origin_y   = self.get_parameter('board_origin_y_mm').value
        self.move_speed = self.get_parameter('move_speed_mm_s').value
        self.edge_x     = self.get_parameter('board_edge_safe_x_mm').value

        self._graveyard = GraveyardManager(
            origin_x_mm    = self.get_parameter('graveyard_origin_x_mm').value,
            origin_y_mm    = self.get_parameter('graveyard_origin_y_mm').value,
            slot_spacing_mm= self.get_parameter('graveyard_slot_spacing_mm').value,
            cols           = self.get_parameter('graveyard_cols').value,
        )

        # ── Publishers ─────────────────────────────────────────────────────
        self._done_pub   = self.create_publisher(Bool, '/motion/done', 10)
        self._state_pub  = self.create_publisher(String, '/game_manager/state', 10)

        # ── Action client ──────────────────────────────────────────────────
        self._gantry_client = ActionClient(self, MoveGantry, '/gantry/move')

        # ── Service clients ────────────────────────────────────────────────
        self._servo_engage  = self.create_client(Trigger, '/servo/engage')
        self._servo_release = self.create_client(Trigger, '/servo/release')

        # ── Command subscription ───────────────────────────────────────────
        # Format: "UCI FEN"  e.g.  "e2e4 rnbqkbnr/.../w KQkq - 0 1"
        self.create_subscription(String, '/motion/command', self._command_cb, 10)

        self.get_logger().info(
            f'Motion Planner ready — '
            f'a1=({self.origin_x}, {self.origin_y})mm, '
            f'sq={self.sq_size}mm, speed={self.move_speed}mm/s'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Command Handling
    # ─────────────────────────────────────────────────────────────────────

    def _command_cb(self, msg: String):
        """
        Receive 'UCI FEN' command and execute the appropriate move sequence.

        Format:  "e2e4 rnbqkbnr/pppppppp/.../RNBQKBNR w KQkq - 0 1"
        """
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
            self._call_servo(self._servo_release)  # Safety: release magnet
            self._publish_done(False)

    # ─────────────────────────────────────────────────────────────────────
    # Move Classification and Execution
    # ─────────────────────────────────────────────────────────────────────

    def _execute_move(self, move: chess.Move, board: chess.Board) -> bool:
        """
        Determine the type of move and execute the full gantry sequence.

        Handles:
          - Normal moves (direct pick and place)
          - Captures (remove captured piece first via graveyard routing)
          - En passant (remove pawn from different square)
          - Castling (move king, then rook)
          - Pawn promotion (move pawn, publish PROMOTION_WAIT for computer black)
        """
        is_capture    = board.is_capture(move)
        is_castling   = board.is_castling(move)
        is_en_passant = board.is_en_passant(move)
        is_promotion  = move.promotion is not None

        self.get_logger().info(
            f'  capture={is_capture} castle={is_castling} '
            f'ep={is_en_passant} promo={is_promotion}'
        )

        if is_castling:
            return self._execute_castling(move, board)

        if is_en_passant:
            return self._execute_en_passant(move, board)

        if is_capture:
            # Phase 1: remove captured piece to graveyard
            ok = self._capture_piece_to_graveyard(move.to_square, board)
            if not ok:
                return False

        # Execute the main piece move
        ok = self._pick_and_place_square(move.from_square, move.to_square)
        if not ok:
            return False

        if is_promotion:
            self._handle_promotion(move, board)

        return True

    # ─────────────────────────────────────────────────────────────────────
    # Special Move Implementations
    # ─────────────────────────────────────────────────────────────────────

    def _execute_castling(self, move: chess.Move, board: chess.Board) -> bool:
        """
        Execute castling: move king, then rook.

        Castling UCI:
          White kingside:  e1g1  (rook h1→f1)
          White queenside: e1c1  (rook a1→d1)
          Black kingside:  e8g8  (rook h8→f8)
          Black queenside: e8c8  (rook a8→d8)
        """
        king_from = move.from_square
        king_to   = move.to_square

        # Determine rook positions from the move
        if board.is_kingside_castling(move):
            if board.turn == chess.WHITE:
                rook_from = chess.H1
                rook_to   = chess.F1
            else:
                rook_from = chess.H8
                rook_to   = chess.F8
        else:  # queenside
            if board.turn == chess.WHITE:
                rook_from = chess.A1
                rook_to   = chess.D1
            else:
                rook_from = chess.A8
                rook_to   = chess.D8

        self.get_logger().info(
            f'Castling: King {chess.square_name(king_from)}→{chess.square_name(king_to)}, '
            f'Rook {chess.square_name(rook_from)}→{chess.square_name(rook_to)}'
        )

        # Move king first, then rook
        ok = self._pick_and_place_square(king_from, king_to)
        if not ok:
            return False
        return self._pick_and_place_square(rook_from, rook_to)

    def _execute_en_passant(self, move: chess.Move, board: chess.Board) -> bool:
        """
        Execute en passant:
          1. Remove captured pawn (on the SAME rank as the capturing pawn,
             on the TARGET file)
          2. Move attacking pawn from source to target

        For white pawn d5→e6: captured pawn is at e5.
        For black pawn d4→e3: captured pawn is at e4.
        """
        src   = move.from_square
        dst   = move.to_square

        # Captured pawn is on the same rank as the attacker, on the dest file
        src_rank   = chess.square_rank(src)
        dst_file   = chess.square_file(dst)
        captured_sq = chess.square(dst_file, src_rank)

        self.get_logger().info(
            f'En passant: {chess.square_name(src)}→{chess.square_name(dst)}, '
            f'capturing pawn at {chess.square_name(captured_sq)}'
        )

        # Step 1: Remove captured pawn to graveyard
        ok = self._capture_piece_to_graveyard(captured_sq, board)
        if not ok:
            return False

        # Step 2: Move attacking pawn
        return self._pick_and_place_square(src, dst)

    def _handle_promotion(self, move: chess.Move, board: chess.Board):
        """
        Handle pawn promotion.

        - If COMPUTER (Black) promotes: publish PROMOTION_WAIT so game_manager
          pauses the clock and waits for the user to place the queen.
        - If HUMAN (White) promotes: publish PROMOTION_WAIT so user is prompted
          to physically swap the promoted piece. Clock keeps running.
        """
        color = 'Black' if board.turn == chess.BLACK else 'White'
        piece = chess.piece_name(move.promotion).capitalize()
        sq    = chess.square_name(move.to_square)

        self.get_logger().info(
            f'Pawn promotion: {color} pawn → {piece} at {sq}. '
            f'Prompting user to place piece on board.'
        )

        # Publish PROMOTION_WAIT so game_manager and clock_node react
        self._state_pub.publish(String(data='PROMOTION_WAIT'))

    # ─────────────────────────────────────────────────────────────────────
    # Graveyard Routing
    # ─────────────────────────────────────────────────────────────────────

    def _capture_piece_to_graveyard(
        self,
        square: chess.Square,
        board: chess.Board,
    ) -> bool:
        """
        Lift the piece from `square` and carry it to the next graveyard slot.

        Routing (safe, clears all board pieces):
          1. Engage magnet at source square
          2. Move to board_edge_safe_x (X=5mm, far right, outside board)
             at the same Y → clears all pieces horizontally
          3. Move to graveyard Y at same X=5mm → move vertically outside board
          4. Move to slot X → final horizontal move in graveyard area
          5. Release magnet
        """
        slot_x, slot_y = self._graveyard.get_next_slot()
        src_x,  src_y  = self._square_to_mm(chess.square_name(square))

        self.get_logger().info(
            f'  Removing piece from {chess.square_name(square)} to graveyard '
            f'slot ({slot_x:.0f}, {slot_y:.0f})mm'
        )

        # 1. Move to piece square
        ok = self._gantry_move(src_x, src_y)
        if not ok: return False

        # 2. Engage magnet (lower permanent magnet to board)
        self._call_servo(self._servo_engage)

        # 3. Move horizontally to safe board edge (X=5, same Y)
        ok = self._gantry_move(self.edge_x, src_y)
        if not ok:
            self._call_servo(self._servo_release)
            return False

        # 4. Move vertically to graveyard Y (same X=5, outside board)
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
    # Standard Pick-and-Place
    # ─────────────────────────────────────────────────────────────────────

    def _pick_and_place_square(
        self,
        src_sq: chess.Square,
        dst_sq: chess.Square,
    ) -> bool:
        """
        Simple pick-and-place between two board squares.
        No obstacle routing — direct path (valid for normal moves where
        the path between squares contains no pieces, or when piece jumping
        corner routing is not required).

        TODO Phase 4.6: Add corner-based obstacle routing via:
              PieceRoutingPlanner.route_via_corner(src_sq, dst_sq, board_state)
        """
        src_x, src_y = self._square_to_mm(chess.square_name(src_sq))
        dst_x, dst_y = self._square_to_mm(chess.square_name(dst_sq))

        self.get_logger().info(
            f'  Pick-and-place: {chess.square_name(src_sq)}({src_x:.0f},{src_y:.0f})'
            f' → {chess.square_name(dst_sq)}({dst_x:.0f},{dst_y:.0f})'
        )

        # 1. Move to source square
        ok = self._gantry_move(src_x, src_y)
        if not ok: return False

        # 2. Engage magnet (lower permanent magnet to board)
        self._call_servo(self._servo_engage)

        # 3. Move to destination
        ok = self._gantry_move(dst_x, dst_y)
        if not ok:
            self._call_servo(self._servo_release)
            return False

        # 4. Release magnet
        self._call_servo(self._servo_release)
        return True

    # ─────────────────────────────────────────────────────────────────────
    # Corner-Based Obstacle Routing (SCAFFOLDED — not yet implemented)
    # ─────────────────────────────────────────────────────────────────────

    def _route_via_corner(
        self,
        src_sq: chess.Square,
        dst_sq: chess.Square,
        board: chess.Board,
    ) -> List[Tuple[float, float]]:
        """
        Generate a collision-free path as a list of (x_mm, y_mm) waypoints.

        Design:
          Each chess piece fits within half a square. This means the gantry
          can navigate through the CORNERS where 4 squares meet without
          touching adjacent pieces. These corner points are:
            corner_x = origin_x - (col + 0.5) * sq_size  (between files)
            corner_y = origin_y + (rank + 0.5) * sq_size  (between ranks)

          A path can be planned by routing through these corners using a
          graph search (A* or BFS) that avoids corners adjacent to occupied squares.

        Current Status:
          NOT IMPLEMENTED. This is scaffolded for Phase 4.6.

        Fallback:
          Until implemented, _pick_and_place_square() uses direct routing,
          which works correctly for most standard moves but will attempt to
          cross occupied squares for knights and certain board configurations.
        """
        raise NotImplementedError(
            'Corner-based obstacle routing is not yet implemented. '
            'See Phase 4.6 in task_list.md.'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Coordinate Math
    # ─────────────────────────────────────────────────────────────────────

    def _square_to_mm(self, square: str) -> Tuple[float, float]:
        """
        Convert a chess square name (e.g. 'e4') to gantry XY in mm.

        Coordinate system:
          Origin (0,0) = bottom-right corner (homing position)
          +X = LEFT toward a-file  → a-file (col=0) has the highest X
          +Y = UP toward rank 8    → rank 8 (row=7) has the highest Y

        Formula:
          x = board_origin_x_mm - col_index * square_size_mm
          y = board_origin_y_mm + rank_index * square_size_mm
          where a=0, b=1, ..., h=7 and rank 1=index 0, rank 8=index 7
        """
        col = COL_MAP.get(square[0].lower())
        row = int(square[1]) - 1   # 0-indexed
        if col is None or not (0 <= row <= 7):
            raise ValueError(f'Invalid square: {square!r}')
        x = self.origin_x - col * self.sq_size
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
        """
        Send a MoveGantry goal and block until complete.
        Returns True on success, False on failure or timeout.
        """
        if not self._gantry_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('/gantry/move action server not available')
            return False

        goal = MoveGantry.Goal()
        goal.target_x_mm  = float(x_mm)
        goal.target_y_mm  = float(y_mm)
        goal.speed_mm_s   = float(speed) if speed else self.move_speed
        goal.engage_magnet = False  # We control magnet via servo service

        goal_future = self._gantry_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error(
                f'Gantry goal ({x_mm:.0f},{y_mm:.0f}) rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)

        if result_future.result() is None:
            self.get_logger().error(
                f'Gantry move ({x_mm:.0f},{y_mm:.0f}) timed out')
            return False

        result = result_future.result()
        success = result.status == GoalStatus.STATUS_SUCCEEDED
        if not success:
            self.get_logger().error(
                f'Gantry move failed: {result.result.message}')
        return success

    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().debug(
            f'  Gantry: {fb.percent_complete:.0f}%  '
            f'({fb.current_x_mm:.1f}, {fb.current_y_mm:.1f})mm'
        )

    # ─────────────────────────────────────────────────────────────────────
    # Servo Helpers
    # ─────────────────────────────────────────────────────────────────────

    def _call_servo(self, client, timeout: float = 3.0):
        """Call a servo trigger service (engage or release). Non-blocking failure."""
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn(
                'Servo service not available — skipping (magnet may not move)')
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
        """Publish move completion to /motion/done."""
        self._done_pub.publish(Bool(data=success))
        self.get_logger().info(
            f'Move {"complete" if success else "FAILED"}')


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
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()
