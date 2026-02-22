#!/usr/bin/env python3
"""
Game Manager Node — full chess game state machine.

Orchestrates the entire game flow:
  1. Home gantry on startup
  2. Wait for human to make first move (clock hit = game start)
  3. Capture/validate/process each move pair (human → computer)
  4. Handle game end conditions

Game Flow Per Turn:
  Human presses clock after moving
    → Capture board state via camera
    → Infer and validate human's move (vs pre-move board state)
    → Tell clock: computer's turn (BLACK clock starts)
    → Ask chess engine for computer's response
    → Tell motion planner to execute move
    → Wait for move completion
    → Hit clock servo (switches clock back to human)
    → Tell clock: human's turn (WHITE clock starts)
    → Repeat

State Machine States:
  STARTUP             → node initializing
  HOMING              → gantry returning to origin
  IDLE                → waiting for game to start (first clock hit)
  WAITING_PLAYER_MOVE → human's turn; waiting for clock press
  CAPTURING_BOARD     → camera capturing post-move image
  VALIDATING_MOVE     → comparing FEN to infer human's move
  CALCULATING_RESPONSE → chess engine computing reply
  EXECUTING_MOVE      → gantry executing computer's move
  HITTING_CLOCK       → clock servo pressing button
  GAME_OVER           → checkmate/stalemate/draw/flag fall

Published Topics:
  /game_manager/state (String) — current state name (read by clock_display_node, chess_clock_node)
  /game_manager/turn  (String) — "WHITE" or "BLACK" (read by chess_clock_node)
  /motion/command     (String) — "UCI FEN" e.g. "e2e4 rnbq..." (read by motion_planner_node)

Subscribed Topics:
  /limit_switch/clock_hit  (Bool) — human pressed chess clock
  /perception/board_state  (BoardState) — detected board FEN
  /game_manager/clock_event (String) — "FLAG_WHITE" or "FLAG_BLACK" from chess_clock_node
  /motion/done             (Bool) — motion planner completed move

Service Clients:
  /gantry/home      (Trigger)      — home gantry
  /camera/capture   (Trigger)      — trigger camera capture
  /clock/hit        (Trigger)      — servo presses clock button
  /chess_engine/request_move (RequestMove) — get best engine move
"""

import threading
import time

import chess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from chess_interfaces.msg import BoardState
from chess_interfaces.srv import RequestMove


# ─────────────────────────────────────────────────────────────────────────────
# Game State Enum (as string constants for easy publishing)
# ─────────────────────────────────────────────────────────────────────────────

class GS:
    STARTUP              = 'STARTUP'
    HOMING               = 'HOMING'
    IDLE                 = 'IDLE'
    WAITING_PLAYER_MOVE  = 'WAITING_PLAYER_MOVE'
    CAPTURING_BOARD      = 'CAPTURING_BOARD'
    VALIDATING_MOVE      = 'VALIDATING_MOVE'
    CALCULATING_RESPONSE = 'CALCULATING_RESPONSE'
    EXECUTING_MOVE       = 'EXECUTING_MOVE'
    HITTING_CLOCK        = 'HITTING_CLOCK'
    PROMOTION_WAIT       = 'PROMOTION_WAIT'
    GAME_OVER            = 'GAME_OVER'


class GameManagerNode(Node):

    def __init__(self):
        super().__init__('game_manager_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('engine_think_time_s', 2.0)
        self.declare_parameter('board_capture_timeout_s', 5.0)
        self.declare_parameter('motion_timeout_s', 120.0)
        self.declare_parameter('homing_timeout_s', 90.0)

        self._think_time   = self.get_parameter('engine_think_time_s').value
        self._cap_timeout  = self.get_parameter('board_capture_timeout_s').value
        self._move_timeout = self.get_parameter('motion_timeout_s').value
        self._home_timeout = self.get_parameter('homing_timeout_s').value

        # ── Python-chess board ────────────────────────────────────────────
        self._board = chess.Board()          # authoritative game state
        self._pre_move_fen = chess.STARTING_FEN  # FEN before human moves

        # ── State machine ─────────────────────────────────────────────────
        self._state = GS.STARTUP
        self._state_lock = threading.Lock()

        # ── Threading events (callbacks → game loop) ──────────────────────
        self._clock_hit_event  = threading.Event()
        self._board_state_event = threading.Event()
        self._motion_done_event = threading.Event()
        self._flag_event        = threading.Event()
        self._flag_loser        = ''        # 'WHITE' or 'BLACK'

        # Latest values from subscriptions
        self._latest_fen     = chess.STARTING_FEN
        self._motion_success = True

        # ── Publishers ────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/game_manager/state', 10)
        self._turn_pub  = self.create_publisher(String, '/game_manager/turn',  10)
        self._motion_pub = self.create_publisher(String, '/motion/command', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/limit_switch/clock_hit', self._on_clock_hit, 10)
        self.create_subscription(
            BoardState, '/perception/board_state', self._on_board_state, 10)
        self.create_subscription(
            String, '/game_manager/clock_event', self._on_clock_event, 10)
        self.create_subscription(
            Bool, '/motion/done', self._on_motion_done, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._home_client   = self.create_client(Trigger, '/gantry/home')
        self._capture_client = self.create_client(Trigger, '/camera/capture')
        self._clock_hit_client = self.create_client(Trigger, '/clock/hit')
        self._engine_client = self.create_client(
            RequestMove, '/chess_engine/request_move')

        # ── Start game loop thread ─────────────────────────────────────────
        self._game_thread = threading.Thread(
            target=self._game_loop, daemon=True, name='game_loop')
        self._game_thread.start()

        self.get_logger().info('Game Manager initialized — waiting for ROS to settle...')

    # ─────────────────────────────────────────────────────────────────────
    # ROS Callbacks (called from ROS spin thread)
    # ─────────────────────────────────────────────────────────────────────

    def _on_clock_hit(self, msg: Bool):
        """Human pressed the chess clock button."""
        if msg.data:
            state = self._state
            if state in (GS.IDLE, GS.WAITING_PLAYER_MOVE, GS.PROMOTION_WAIT):
                self.get_logger().info(f'Clock hit received (state={state})')
                self._clock_hit_event.set()

    def _on_board_state(self, msg: BoardState):
        """Received updated board state from perception pipeline."""
        self._latest_fen = msg.fen
        self._board_state_event.set()

    def _on_motion_done(self, msg: Bool):
        """Motion planner signalled move completion."""
        self._motion_success = msg.data
        self._motion_done_event.set()

    def _on_clock_event(self, msg: String):
        """Flag fall event from chess_clock_node."""
        event = msg.data.upper()
        if event.startswith('FLAG_'):
            loser = event[5:]  # 'WHITE' or 'BLACK'
            self._flag_loser = loser
            self._flag_event.set()
            self.get_logger().error(f'FLAG FALL — {loser} loses on time!')

    # ─────────────────────────────────────────────────────────────────────
    # State Machine Loop (runs in background thread)
    # ─────────────────────────────────────────────────────────────────────

    def _game_loop(self):
        """Main game state machine — runs in its own thread."""
        time.sleep(1.0)    # Let ROS spin settle

        self._transition(GS.HOMING)
        if not self._do_home():
            self.get_logger().error('Homing failed — staying in STARTUP. Retry manually.')
            self._transition(GS.STARTUP)
            return

        self._transition(GS.IDLE)
        self.get_logger().info('Ready. Waiting for human to make first move then press clock...')

        # ── Main game loop ────────────────────────────────────────────────
        while rclpy.ok():
            # Check for flag fall at any point
            if self._flag_event.is_set():
                self._flag_event.clear()
                self._end_game(f'Time expired — {self._flag_loser} loses')
                break

            if self._state in (GS.IDLE, GS.WAITING_PLAYER_MOVE):
                # Wait for human to press clock
                self._clock_hit_event.clear()
                self.get_logger().info(
                    f'[{self._state}] Waiting for clock press...')

                # Block until clock hit or flag fall (check flag every 0.5s)
                while rclpy.ok():
                    if self._clock_hit_event.wait(timeout=0.5):
                        break
                    if self._flag_event.is_set():
                        break
                if self._flag_event.is_set():
                    continue  # Handle flag at top of loop

                self._clock_hit_event.clear()

                if self._state == GS.IDLE:
                    # First clock press — game starts, capture initial board state
                    self.get_logger().info('First clock press — game starting!')
                    self._board = chess.Board()        # fresh game
                    self._pre_move_fen = self._board.fen()

                # ── Capture board state ───────────────────────────────────
                self._transition(GS.CAPTURING_BOARD)
                ok = self._do_capture_board()
                if not ok:
                    self.get_logger().warn('Board capture timed out — asking for retry')
                    self._transition(GS.WAITING_PLAYER_MOVE)
                    continue

                # ── Validate human's move ─────────────────────────────────
                self._transition(GS.VALIDATING_MOVE)
                human_move = self._do_validate_move()
                if human_move is None:
                    self.get_logger().warn(
                        'Could not determine valid move — please try again')
                    self._transition(GS.WAITING_PLAYER_MOVE)
                    continue

                # Apply move to internal board
                self._board.push(human_move)
                self.get_logger().info(
                    f'Human move accepted: {human_move.uci()}  '
                    f'Board: {self._board.fen().split(" ")[0]}')

                # Check game end after human move
                game_end = self._check_game_end()
                if game_end:
                    self.get_logger().info(f'Game over: {game_end}')
                    self._end_game(game_end)
                    break

                # ── Switch clock to computer ──────────────────────────────
                self._publish_turn('BLACK')

                # ── Calculate engine response ─────────────────────────────
                self._transition(GS.CALCULATING_RESPONSE)
                computer_move_uci = self._do_get_engine_move()
                if computer_move_uci is None:
                    self.get_logger().error(
                        'Engine failed — using first legal move as fallback')
                    moves = list(self._board.legal_moves)
                    if moves:
                        computer_move_uci = moves[0].uci()
                    else:
                        self.get_logger().error('No legal moves available — game over?')
                        self._end_game('No legal moves for computer')
                        break

                computer_move = chess.Move.from_uci(computer_move_uci)

                # ── Execute gantry move ────────────────────────────────────
                self._transition(GS.EXECUTING_MOVE)
                ok = self._do_execute_move(computer_move_uci, self._board.fen())
                if not ok:
                    self.get_logger().error(f'Motion failed for {computer_move_uci}')
                    # Don't break — gantry may have partially moved, try to continue
                    self.get_logger().warn('Attempting to continue despite motion error')

                # Apply computer move to internal board
                # Check for pawn promotion
                needs_promotion_wait = (
                    computer_move.promotion is not None
                    and self._board.turn == chess.BLACK  # computer is Black
                )

                self._board.push(computer_move)

                # If computer promoted (black pawn to rank 1), wait for user
                if needs_promotion_wait:
                    self.get_logger().info(
                        'Computer promoted! Waiting for user to place queen on board...')
                    self._transition(GS.PROMOTION_WAIT)
                    self._clock_hit_event.clear()
                    self._clock_hit_event.wait()  # User presses clock to confirm
                    self._transition(GS.EXECUTING_MOVE)

                # Check game end after computer move
                game_end = self._check_game_end()
                if game_end:
                    self.get_logger().info(f'Game over: {game_end}')
                    # Hit clock before ending so display is clean
                    self._transition(GS.HITTING_CLOCK)
                    self._do_hit_clock()
                    self._end_game(game_end)
                    break

                # ── Hit clock to pass turn to human ───────────────────────
                self._transition(GS.HITTING_CLOCK)
                self._do_hit_clock()

                # ── Switch clock to human ─────────────────────────────────
                self._publish_turn('WHITE')

                # Update pre-move FEN for next round
                self._pre_move_fen = self._board.fen()

                # ── Back to waiting player ────────────────────────────────
                self._transition(GS.WAITING_PLAYER_MOVE)

        self.get_logger().info('Game loop exited.')

    # ─────────────────────────────────────────────────────────────────────
    # State Machine Actions
    # ─────────────────────────────────────────────────────────────────────

    def _do_home(self) -> bool:
        """Call /gantry/home and wait for completion."""
        self.get_logger().info('Homing gantry...')
        if not self._home_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('/gantry/home service not available')
            return False
        future = self._home_client.call_async(Trigger.Request())
        # Spin with timeout to avoid blocking rclpy
        deadline = time.monotonic() + self._home_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        if not future.done() or future.result() is None:
            self.get_logger().error('Homing timed out')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info('Homing complete')
        else:
            self.get_logger().error(f'Homing failed: {result.message}')
        return result.success

    def _do_capture_board(self) -> bool:
        """Trigger camera capture and wait for board state update."""
        self.get_logger().info('Capturing board image...')

        # Clear previous board state event
        self._board_state_event.clear()

        # Trigger camera
        if self._capture_client.wait_for_service(timeout_sec=3.0):
            self._capture_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                '/camera/capture not available — waiting for board state anyway')

        # Wait for piece_detector to publish updated board state
        got_state = self._board_state_event.wait(timeout=self._cap_timeout)
        if not got_state:
            self.get_logger().warn('Board state not received within timeout')
            return False

        self._board_state_event.clear()
        self.get_logger().info(f'Board state received: {self._latest_fen[:40]}...')
        return True

    def _do_validate_move(self) -> 'chess.Move | None':
        """
        Infer the human's move by comparing pre-move and post-move FEN.

        Strategy: try every legal move from the pre-move position; see
        which one produces a board position matching the detected FEN.
        This handles all special moves (castling, en passant, promotion)
        automatically via python-chess.
        """
        pre_board = chess.Board(self._pre_move_fen)
        detected_piece_layout = self._latest_fen.split(' ')[0]

        for candidate_move in pre_board.legal_moves:
            test_board = pre_board.copy()
            test_board.push(candidate_move)
            candidate_layout = test_board.fen().split(' ')[0]
            if candidate_layout == detected_piece_layout:
                self.get_logger().info(
                    f'Inferred move: {candidate_move.uci()} '
                    f'({chess.piece_name(pre_board.piece_at(candidate_move.from_square).piece_type)} '
                    f'{chess.square_name(candidate_move.from_square)}'
                    f'→{chess.square_name(candidate_move.to_square)})'
                )
                return candidate_move

        # No legal move matched — perception might be a placeholder or move is ambiguous
        self.get_logger().warn(
            f'No legal move matches detected FEN.\n'
            f'  Pre-move:  {pre_board.fen()}\n'
            f'  Detected:  {self._latest_fen}\n'
            f'  Legal moves: {[m.uci() for m in pre_board.legal_moves]}'
        )
        return None

    def _do_get_engine_move(self) -> 'str | None':
        """Call chess engine for best response move. Returns UCI string or None."""
        self.get_logger().info(
            f'Requesting engine move for: {self._board.fen()}')

        if not self._engine_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/chess_engine/request_move not available')
            return None

        req = RequestMove.Request()
        req.fen = self._board.fen()

        future = self._engine_client.call_async(req)
        deadline = time.monotonic() + self._think_time + 10.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.1)

        if not future.done() or future.result() is None:
            self.get_logger().error('Engine request timed out')
            return None

        result = future.result()
        if result.success and result.best_move_uci:
            self.get_logger().info(f'Engine response: {result.best_move_uci}')
            return result.best_move_uci
        self.get_logger().error('Engine returned empty/failed response')
        return None

    def _do_execute_move(self, uci: str, fen: str) -> bool:
        """
        Publish move command to motion_planner_node and wait for /motion/done.

        Command format: "UCI FEN"  e.g. "e2e4 rnbqkbnr/pp.../w KQkq - 0 1"
        """
        command = f'{uci} {fen}'
        self.get_logger().info(f'Executing move: {uci}')

        self._motion_done_event.clear()
        self._motion_pub.publish(String(data=command))

        # Wait for motion planner to signal completion
        completed = self._motion_done_event.wait(timeout=self._move_timeout)
        if not completed:
            self.get_logger().error(
                f'Motion timeout after {self._move_timeout}s for move {uci}')
            return False

        self._motion_done_event.clear()
        return self._motion_success

    def _do_hit_clock(self) -> bool:
        """Call the clock servo to press the chess clock button."""
        self.get_logger().info('Hitting clock servo...')
        if not self._clock_hit_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('/clock/hit service not available — skipping')
            return False
        future = self._clock_hit_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if future.done() and future.result():
            return future.result().success
        return False

    # ─────────────────────────────────────────────────────────────────────
    # Game Logic Helpers
    # ─────────────────────────────────────────────────────────────────────

    def _check_game_end(self) -> 'str | None':
        """
        Check if the game has ended. Returns a human-readable result string
        if the game is over, or None if it should continue.
        """
        board = self._board

        if board.is_checkmate():
            winner = 'White' if board.turn == chess.BLACK else 'Black'
            return f'Checkmate — {winner} wins!'

        if board.is_stalemate():
            return 'Stalemate — Draw'

        if board.is_insufficient_material():
            return 'Draw — insufficient material'

        if board.is_seventyfive_moves():
            return 'Draw — 75-move rule'

        if board.is_fivefold_repetition():
            return 'Draw — fivefold repetition'

        if board.is_fifty_moves():
            return 'Draw — 50-move rule'

        return None

    def _end_game(self, reason: str):
        """Transition to GAME_OVER and log the result."""
        self._transition(GS.GAME_OVER)
        self._publish_turn('NONE')
        self.get_logger().info(f'═══ GAME OVER ═══  {reason}')
        self.get_logger().info(f'Final position: {self._board.fen()}')
        self.get_logger().info(
            f'Move history: {" ".join(m.uci() for m in self._board.move_stack)}')

    # ─────────────────────────────────────────────────────────────────────
    # Publishing Helpers
    # ─────────────────────────────────────────────────────────────────────

    def _transition(self, new_state: str):
        """Update state and publish to /game_manager/state."""
        with self._state_lock:
            old = self._state
            self._state = new_state
        self._state_pub.publish(String(data=new_state))
        self.get_logger().info(f'State: {old} → {new_state}')

    def _publish_turn(self, player: str):
        """Publish whose clock should be ticking ('WHITE', 'BLACK', 'NONE')."""
        self._turn_pub.publish(String(data=player))
        if player != 'NONE':
            self.get_logger().info(f'Clock turn: {player}')


def main(args=None):
    rclpy.init(args=args)
    node = GameManagerNode()
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
