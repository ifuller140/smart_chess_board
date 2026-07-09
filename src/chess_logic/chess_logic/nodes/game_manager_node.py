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
  HOMING_FAILED       → homing attempt failed; retrying automatically every 5s
  IDLE                → waiting for game to start (first clock hit)
  WAITING_PLAYER_MOVE → human's turn; waiting for clock press
  CAPTURING_BOARD     → camera capturing post-move image
  VALIDATING_MOVE     → comparing FEN to infer human's move
  CALCULATING_RESPONSE → chess engine computing reply
  EXECUTING_MOVE      → gantry executing computer's move
  HITTING_CLOCK       → clock servo pressing button
  PROMOTION_WAIT      → waiting for player to place/confirm promoted piece
  MOTION_ERROR        → gantry move failed; paused for operator, /game/new_game to recover
  GAME_OVER           → checkmate/stalemate/draw/flag fall/resignation

Note: the game loop thread never exits on any of the above conditions —
it always loops back to wait for /game/new_game (or, for HOMING_FAILED,
retries homing on its own). Only real node shutdown stops the thread.

Published Topics:
  /game_manager/state (String) — current state name (read by clock_display_node, chess_clock_node)
  /game_manager/turn  (String) — "WHITE" or "BLACK" (read by chess_clock_node)
  /motion/command     (String) — "UCI FEN" e.g. "e2e4 rnbq..." (read by motion_planner_node)

Subscribed Topics:
  /limit_switch/clock_hit      (Bool)   — human pressed chess clock
  /perception/changed_squares  (String) — comma-separated changed square names from piece_detector
  /game_manager/clock_event    (String) — "FLAG_WHITE" or "FLAG_BLACK" from chess_clock_node
  /motion/done                 (Bool)   — motion planner completed move

Service Clients:
  /gantry/home                  (Trigger)     — home gantry
  /camera/capture               (Trigger)     — trigger camera capture
  /perception/capture_premove   (Trigger)     — tell piece_detector to snapshot current board
  /clock/hit                    (Trigger)     — servo presses clock button
  /chess_engine/request_move    (RequestMove) — get best engine move
"""

import threading
import time

import chess
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from chess_interfaces.srv import RequestMove


# ─────────────────────────────────────────────────────────────────────────────
# Game State Enum (as string constants for easy publishing)
# ─────────────────────────────────────────────────────────────────────────────

class GS:
    STARTUP              = 'STARTUP'
    HOMING               = 'HOMING'
    HOMING_FAILED        = 'HOMING_FAILED'
    IDLE                 = 'IDLE'
    WAITING_PLAYER_MOVE  = 'WAITING_PLAYER_MOVE'
    CAPTURING_BOARD      = 'CAPTURING_BOARD'
    VALIDATING_MOVE      = 'VALIDATING_MOVE'
    CALCULATING_RESPONSE = 'CALCULATING_RESPONSE'
    EXECUTING_MOVE       = 'EXECUTING_MOVE'
    HITTING_CLOCK        = 'HITTING_CLOCK'
    PROMOTION_WAIT       = 'PROMOTION_WAIT'
    PROMOTION_DONE       = 'PROMOTION_DONE'
    MOTION_ERROR         = 'MOTION_ERROR'
    GAME_OVER            = 'GAME_OVER'

# States where the game loop is paused waiting for an operator action
# (new game or resign) rather than normal turn-taking.
_PAUSED_STATES = (GS.GAME_OVER, GS.MOTION_ERROR)


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

        # Live-reconfigure: without this, engine_think_time_s pushed via
        # SetParameters (Chess OS's game-settings UI) only updated ROS's
        # parameter-server bookkeeping, not self._think_time, so the push
        # had zero effect until the process was restarted.
        self.add_on_set_parameters_callback(self._on_params_changed)

        # ── Python-chess board ────────────────────────────────────────────
        self._board = chess.Board()          # authoritative game state
        self._pre_move_fen = chess.STARTING_FEN  # FEN before human moves

        # ── State machine ─────────────────────────────────────────────────
        self._state = GS.STARTUP
        self._state_lock = threading.Lock()

        # ── Threading events (callbacks → game loop) ──────────────────────
        self._clock_hit_event       = threading.Event()
        self._board_state_event     = threading.Event()
        self._motion_done_event     = threading.Event()
        self._flag_event            = threading.Event()
        self._flag_loser            = ''        # 'WHITE' or 'BLACK'
        self._new_game_event        = threading.Event()
        self._resign_event          = threading.Event()

        # Move history (UCI strings, one per half-move)
        self._move_history: list = []

        # Latest values from subscriptions
        self._latest_changed_squares: set = set()   # chess.Square indices from perception
        self._motion_success = True

        # Gates so a late/stale callback from a previous timed-out request
        # (e.g. a /motion/done that arrives after we already gave up waiting)
        # isn't misapplied to a later, unrelated request.
        self._awaiting_board_state = False
        self._awaiting_motion      = False

        # ── Publishers ────────────────────────────────────────────────────
        self._state_pub = self.create_publisher(String, '/game_manager/state', 10)
        self._turn_pub  = self.create_publisher(String, '/game_manager/turn',  10)
        self._motion_pub = self.create_publisher(String, '/motion/command', 10)
        self._abort_pub  = self.create_publisher(Bool,   '/motion/abort',   10)
        # Authoritative FEN for piece_detector_node (game-state-assisted piece typing)
        self._fen_pub   = self.create_publisher(String, '/game_manager/board_fen', 10)
        # Game control publishers (read by Chess OS)
        self._move_history_pub = self.create_publisher(String, '/game_manager/move_history', 10)
        self._result_pub       = self.create_publisher(String, '/game_manager/result', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/limit_switch/clock_hit', self._on_clock_hit, 10)
        self.create_subscription(
            String, '/perception/changed_squares', self._on_changed_squares, 10)
        self.create_subscription(
            String, '/game_manager/clock_event', self._on_clock_event, 10)
        self.create_subscription(
            Bool, '/motion/done', self._on_motion_done, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._home_client      = self.create_client(Trigger, '/gantry/home')
        self._capture_client   = self.create_client(Trigger, '/camera/capture')
        self._premove_client   = self.create_client(Trigger, '/perception/capture_premove')
        self._clock_hit_client = self.create_client(Trigger, '/clock/hit')
        self._engine_client    = self.create_client(
            RequestMove, '/chess_engine/request_move')

        # ── Game control services (called by Chess OS) ────────────────────
        self.create_service(Trigger, '/game/start',    self._svc_start_game)
        self.create_service(Trigger, '/game/new_game', self._svc_new_game)
        self.create_service(Trigger, '/game/resign',   self._svc_resign)

        # Publish initial FEN so piece_detector starts with correct state
        self._publish_board_fen()
        # Periodic re-publish so piece_detector gets FEN even after a restart
        self.create_timer(2.0, self._publish_board_fen)

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

    def _on_changed_squares(self, msg: String):
        """Receive changed squares list from piece_detector_node."""
        if not self._awaiting_board_state:
            # Stale message from a previous, already-timed-out capture request.
            self.get_logger().warn('Ignoring changed-squares — not currently awaiting a capture')
            return
        raw = msg.data.strip()
        sqs: set = set()
        if raw:
            for name in raw.split(','):
                name = name.strip()
                if name:
                    try:
                        sqs.add(chess.parse_square(name))
                    except ValueError:
                        self.get_logger().warn(f'Invalid square name from perception: {name!r}')
        self._latest_changed_squares = sqs
        self._board_state_event.set()

    def _on_motion_done(self, msg: Bool):
        """Motion planner signalled move completion."""
        if not self._awaiting_motion:
            # Stale message from a previous, already-timed-out motion request.
            self.get_logger().warn('Ignoring /motion/done — not currently awaiting a motion result')
            return
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
        while rclpy.ok() and not self._do_home():
            self.get_logger().error('Homing failed — retrying in 5s...')
            self._transition(GS.HOMING_FAILED)
            time.sleep(5.0)
            self._transition(GS.HOMING)
        if not rclpy.ok():
            return

        self._transition(GS.IDLE)
        self._verify_starting_position()
        # Capture the starting-position board as the initial pre-move reference
        self._do_capture_premove()
        self.get_logger().info('Ready. Waiting for human to make first move then press clock...')

        # ── Main game loop ────────────────────────────────────────────────
        while rclpy.ok():
            # New game reset (from Chess OS /game/new_game service)
            if self._new_game_event.is_set():
                self._new_game_event.clear()
                self._clock_hit_event.clear()
                self._board_state_event.clear()
                self._motion_done_event.clear()
                self._board = chess.Board()
                self._pre_move_fen = chess.STARTING_FEN
                self._move_history.clear()
                self._publish_board_fen()
                self._pub_move_history()
                self._transition(GS.IDLE)
                self._do_capture_premove()
                self.get_logger().info("New game reset complete — waiting for clock press")
                continue

            # Resign (from Chess OS /game/resign service) — handled here, in the
            # game-loop thread, rather than mutating state from the service
            # callback thread directly.
            if self._resign_event.is_set():
                self._resign_event.clear()
                if self._state not in (GS.GAME_OVER,):
                    self._abort_pub.publish(Bool(data=True))
                    self._end_game('Resignation')
                continue

            # Check for flag fall at any point
            if self._flag_event.is_set():
                self._flag_event.clear()
                self._end_game(f'Time expired — {self._flag_loser} loses')
                continue

            # Paused after a game end or an unrecoverable motion error — wait
            # here (instead of exiting the loop/thread) for /game/new_game.
            if self._state in _PAUSED_STATES:
                time.sleep(0.2)
                continue

            if self._state in (GS.IDLE, GS.WAITING_PLAYER_MOVE):
                # Wait for human to press clock
                self._clock_hit_event.clear()
                self.get_logger().info(
                    f'[{self._state}] Waiting for clock press...')

                # Block until clock hit, flag fall, or resign (check every 0.5s)
                while rclpy.ok():
                    if self._clock_hit_event.wait(timeout=0.5):
                        break
                    if self._flag_event.is_set() or self._resign_event.is_set():
                        break
                if self._flag_event.is_set() or self._resign_event.is_set():
                    continue  # Handle flag/resign at top of loop

                self._clock_hit_event.clear()

                if self._state == GS.IDLE:
                    # First clock press — game starts, capture initial board state
                    self.get_logger().info('First clock press — game starting!')
                    self._board = chess.Board()        # fresh game
                    self._pre_move_fen = self._board.fen()
                    self._publish_board_fen()

                # ── Capture board state ───────────────────────────────────
                self._transition(GS.CAPTURING_BOARD)
                ok = self._do_capture_board()
                if self._new_game_event.is_set() or self._resign_event.is_set():
                    continue  # new_game/resign woke this wait — handle at top, don't trust `ok`
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
                self._move_history.append(human_move.uci())
                self._pub_move_history()
                self._publish_board_fen()
                self.get_logger().info(
                    f'Human move accepted: {human_move.uci()}  '
                    f'Board: {self._board.fen().split(" ")[0]}')

                # If human promoted, wait for the physical piece swap
                if human_move.promotion is not None:
                    self.get_logger().info(
                        'Human pawn promotion — waiting for player to place promoted piece and press clock...')
                    self._transition(GS.PROMOTION_WAIT)
                    if not self._wait_promotion_confirm():
                        continue  # flag fell or resign happened during promotion wait
                    self._state_pub.publish(String(data=GS.PROMOTION_DONE))

                # Check game end after human move
                game_end = self._check_game_end()
                if game_end:
                    self.get_logger().info(f'Game over: {game_end}')
                    self._end_game(game_end)
                    continue

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
                        continue

                computer_move = chess.Move.from_uci(computer_move_uci)

                # ── Execute gantry move ────────────────────────────────────
                self._transition(GS.EXECUTING_MOVE)
                ok = self._do_execute_move(computer_move_uci, self._board.fen())
                if self._new_game_event.is_set() or self._resign_event.is_set():
                    continue  # new_game/resign woke this wait — handle at top, don't trust `ok`
                if not ok:
                    # Don't push the move to the board model — the physical
                    # board may not match it (partial move, dropped piece).
                    # Pause for operator intervention instead of silently
                    # desyncing model vs. reality.
                    self.get_logger().error(
                        f'Motion failed for {computer_move_uci} — pausing for operator. '
                        f'Verify the physical board, then call /game/new_game to reset.')
                    self._result_pub.publish(String(
                        data=f'Motion error executing {computer_move_uci} — needs operator attention'))
                    self._transition(GS.MOTION_ERROR)
                    continue

                # Apply computer move to internal board
                # Check for pawn promotion
                needs_promotion_wait = (
                    computer_move.promotion is not None
                    and self._board.turn == chess.BLACK  # computer is Black
                )

                self._board.push(computer_move)
                self._move_history.append(computer_move.uci())
                self._pub_move_history()
                self._publish_board_fen()

                # If computer promoted (black pawn to rank 1), wait for user
                if needs_promotion_wait:
                    self.get_logger().info(
                        'Computer promoted! Waiting for user to place queen on board...')
                    self._transition(GS.PROMOTION_WAIT)
                    if not self._wait_promotion_confirm():
                        continue  # flag fell or resign happened during promotion wait
                    # Resume the chess clock before continuing
                    self._state_pub.publish(String(data=GS.PROMOTION_DONE))
                    self._transition(GS.EXECUTING_MOVE)

                # Check game end after computer move
                game_end = self._check_game_end()
                if game_end:
                    self.get_logger().info(f'Game over: {game_end}')
                    # Hit clock before ending so display is clean
                    self._transition(GS.HITTING_CLOCK)
                    if not self._do_hit_clock():
                        self.get_logger().warn('Clock-hit servo call failed while ending game')
                    self._end_game(game_end)
                    continue

                # ── Hit clock to pass turn to human ───────────────────────
                self._transition(GS.HITTING_CLOCK)
                if not self._do_hit_clock():
                    self.get_logger().warn(
                        'Clock-hit servo call failed — turn is being published anyway; '
                        'verify the physical clock was actually pressed')

                # ── Switch clock to human ─────────────────────────────────
                self._publish_turn('WHITE')

                # Update pre-move FEN and capture fresh board reference for next round
                self._pre_move_fen = self._board.fen()
                self._do_capture_premove()

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
        """Trigger camera capture and wait for changed-squares detection."""
        self.get_logger().info('Capturing post-move board image...')

        self._board_state_event.clear()
        self._awaiting_board_state = True

        if self._capture_client.wait_for_service(timeout_sec=3.0):
            self._capture_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                '/camera/capture not available — waiting for changed-squares anyway')

        got = self._board_state_event.wait(timeout=self._cap_timeout)
        self._awaiting_board_state = False
        if not got:
            self.get_logger().warn('Changed-squares message not received within timeout')
            return False

        self._board_state_event.clear()
        sq_names = sorted(chess.square_name(s) for s in self._latest_changed_squares)
        self.get_logger().info(f'Changed squares detected: {sq_names}')
        return True

    def _do_capture_premove(self) -> bool:
        """
        Call /perception/capture_premove to store the current board image as
        the pre-move reference. Called when entering WAITING_PLAYER_MOVE so
        the reference reflects the board state before the player moves.
        """
        if not self._premove_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                '/perception/capture_premove not available — '
                'frame-diff detection will be unavailable this turn')
            return False
        future = self._premove_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            self.get_logger().warn('Pre-move capture timed out')
            return False
        result = future.result()
        if result.success:
            self.get_logger().info(f'Pre-move reference: {result.message}')
        return result.success

    def _do_validate_move(self) -> 'chess.Move | None':
        """
        Infer the human's move from the set of changed squares reported by
        piece_detector_node (frame diff of pre-move vs post-move board image).

        For each legal move, computes the exact set of squares that would change
        on the board and matches it against what perception detected:
          - Normal move:  2 squares (from, to)
          - Capture:      2 squares (from=empty, to=piece-changed)
          - En passant:   3 squares (from, to, captured-pawn)
          - Castling:     4 squares (king from/to + rook from/to)
          - Promotion:    2 squares (same footprint as normal; prefer queen)

        Noise tolerance: if no exact match, tries removing the lowest-diff square
        to account for minor false positives from lighting or camera noise.
        """
        pre_board = chess.Board(self._pre_move_fen)
        changed   = self._latest_changed_squares

        if len(changed) == 0:
            self.get_logger().warn('No squares changed — cannot infer move')
            return None

        sq_names = sorted(chess.square_name(s) for s in changed)
        self.get_logger().info(f'Validating against changed squares: {sq_names}')

        # Exact match first
        move = self._match_legal_move(pre_board, changed)
        if move:
            return move

        # Noise tolerance: try dropping one square at a time
        if len(changed) > 2:
            for sq in list(changed):
                subset = changed - {sq}
                if len(subset) >= 2:
                    move = self._match_legal_move(pre_board, subset)
                    if move:
                        self.get_logger().warn(
                            f'Noise tolerance applied: ignored square '
                            f'{chess.square_name(sq)} — matched {move.uci()}')
                        return move

        self.get_logger().warn(
            f'No legal move matches changed squares: {sq_names}  '
            f'Pre-FEN: {self._pre_move_fen}'
        )
        return None

    def _match_legal_move(
        self, board: chess.Board, changed: set
    ) -> 'chess.Move | None':
        """Find a legal move whose board footprint exactly equals changed."""
        candidates = []
        for move in board.legal_moves:
            if self._get_move_changed_squares(board, move) == changed:
                candidates.append(move)

        if not candidates:
            return None

        if len(candidates) == 1:
            move  = candidates[0]
            piece = board.piece_at(move.from_square)
            pname = chess.piece_name(piece.piece_type) if piece else '?'
            self.get_logger().info(
                f'Inferred move: {move.uci()} '
                f'({pname} {chess.square_name(move.from_square)}'
                f'→{chess.square_name(move.to_square)})'
            )
            return move

        # Multiple candidates occur only with pawn promotion (same squares, different piece)
        queen_promos = [m for m in candidates if m.promotion == chess.QUEEN]
        if queen_promos:
            self.get_logger().info(
                f'Promotion ambiguity resolved to queen: {queen_promos[0].uci()}')
            return queen_promos[0]

        chosen = candidates[0]
        self.get_logger().info(
            f'Multiple candidates, chose: {chosen.uci()} '
            f'(all: {[m.uci() for m in candidates]})'
        )
        return chosen

    def _get_move_changed_squares(self, board: chess.Board, move: chess.Move) -> set:
        """
        Return the set of chess.Square indices that change when move is applied.

        Standard move / capture:  {from_square, to_square}
        Castling adds rook from/to:  e.g. O-O → {e1, g1, h1, f1}
        En passant adds captured pawn's square:  {e5, d6, d5}
        """
        changed = {move.from_square, move.to_square}

        if board.is_castling(move):
            rank = chess.square_rank(move.from_square)   # 0 for white, 7 for black
            if board.is_kingside_castling(move):
                # Rook moves h→f
                changed.update({chess.square(7, rank), chess.square(5, rank)})
            else:
                # Rook moves a→d
                changed.update({chess.square(0, rank), chess.square(3, rank)})

        if board.is_en_passant(move):
            # Captured pawn: same file as destination, same rank as origin
            ep_file = chess.square_file(move.to_square)
            ep_rank = chess.square_rank(move.from_square)
            changed.add(chess.square(ep_file, ep_rank))

        return changed

    def _do_get_engine_move(self) -> 'str | None':
        """Call chess engine for best response move. Returns UCI string or None."""
        self.get_logger().info(
            f'Requesting engine move for: {self._board.fen()}')

        if not self._engine_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/chess_engine/request_move not available')
            return None

        req = RequestMove.Request()
        req.fen = self._board.fen()
        req.think_time_s = float(self._think_time)

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
        self._awaiting_motion = True
        self._motion_pub.publish(String(data=command))

        # Wait for motion planner to signal completion
        completed = self._motion_done_event.wait(timeout=self._move_timeout)
        self._awaiting_motion = False
        if not completed:
            self.get_logger().error(
                f'Motion timeout after {self._move_timeout}s for move {uci}')
            return False

        self._motion_done_event.clear()
        return self._motion_success

    def _wait_promotion_confirm(self) -> bool:
        """
        Block (with periodic flag/resign polling) until the player presses the
        clock to confirm the promoted piece has been physically placed.

        Returns True if confirmed normally. Returns False if a flag fall or
        resign happened during the wait — the caller should `continue` the
        main loop so it's handled at the top on the next iteration.
        """
        self._clock_hit_event.clear()
        while rclpy.ok():
            got_clock_hit = self._clock_hit_event.wait(timeout=0.5)
            # Resign also sets _clock_hit_event to unblock this wait promptly —
            # check flag/resign *after* waiting regardless of why we woke up,
            # so a resign isn't misread as a normal promotion confirmation.
            if self._flag_event.is_set() or self._resign_event.is_set():
                return False
            if got_clock_hit:
                return True
        return False

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
        self._result_pub.publish(String(data=reason))
        self.get_logger().info(f'═══ GAME OVER ═══  {reason}')
        self.get_logger().info(f'Final position: {self._board.fen()}')
        self.get_logger().info(
            f'Move history: {" ".join(m.uci() for m in self._board.move_stack)}')

    # ─────────────────────────────────────────────────────────────────────
    # Game Control Services (called by Chess OS)
    # ─────────────────────────────────────────────────────────────────────

    def _svc_start_game(self, request, response):
        """Trigger a clock-hit event from the UI — starts game or unblocks PROMOTION_WAIT."""
        if self._state in (GS.IDLE, GS.PROMOTION_WAIT):
            self._clock_hit_event.set()
            response.success = True
            response.message = f"Game start triggered from state {self._state}"
        else:
            response.success = False
            response.message = f"Cannot start game from state {self._state}"
        return response

    def _svc_new_game(self, request, response):
        """Reset to a fresh game — unblocks any waiting loop, resets board to starting position."""
        # Cancel any in-progress gantry motion before resetting
        self._abort_pub.publish(Bool(data=True))
        self._new_game_event.set()
        self._clock_hit_event.set()
        self._board_state_event.set()
        self._motion_done_event.set()
        response.success = True
        response.message = "New game reset triggered"
        return response

    def _svc_resign(self, request, response):
        """
        Request resignation. The actual state transition/board-ending happens
        in the game-loop thread (which observes _resign_event) rather than
        here in the service-callback thread, to avoid racing the loop's own
        reads/writes of _state and _board.
        """
        self._resign_event.set()
        self._clock_hit_event.set()  # unblock any wait currently in progress
        response.success = True
        response.message = "Resignation requested"
        return response

    def _pub_move_history(self):
        """Publish the full move history as a space-separated UCI string."""
        msg = String()
        msg.data = ' '.join(self._move_history)
        self._move_history_pub.publish(msg)

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

    def _publish_board_fen(self):
        """Publish the current authoritative board FEN to piece_detector and visualizer."""
        self._fen_pub.publish(String(data=self._board.fen()))

    def _on_params_changed(self, params):
        for p in params:
            if p.name == 'engine_think_time_s':
                self._think_time = float(p.value)
                self.get_logger().info(f'engine_think_time_s updated to {self._think_time}s')
        return SetParametersResult(successful=True)

    def _verify_starting_position(self):
        """
        Log readiness. With the frame-diff approach, vision verifies moves rather
        than the starting position — the board is assumed to be correctly set up.
        Called once when entering IDLE.
        """
        self.get_logger().info(
            'Board assumed to be in starting position. '
            'Ensure all 32 pieces are correctly placed before pressing clock.'
        )


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
