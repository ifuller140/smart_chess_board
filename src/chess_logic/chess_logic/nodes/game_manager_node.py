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
  /game_manager/capture_progress (String) — live status during _do_capture_board()'s
    stability wait, e.g. "Stabilizing: 2/3 consistent reads" — for chess_ui display
  /game_manager/move_candidates  (String) — JSON list of the top-3 scored legal moves
    from the last _do_validate_move() call, e.g. [{"uci":"e2e4","score":142.3}, ...]
  /game_manager/resume_pending_ack (Bool) — True after resuming a persisted game
    (see _load_persisted_state()) until /game/ack_resume is called

Game-state persistence:
  Board/history/clock are snapshotted to game_state_file (param, default
  ~/.chess/game_state.json) on every accepted move — see _persist_state()/
  _load_persisted_state(). A crash or Pi restart resumes into
  WAITING_PLAYER_MOVE (never mid-motion/mid-capture) and blocks the next
  clock press until /game/ack_resume confirms the physical board matches.

Subscribed Topics:
  /limit_switch/clock_hit      (Bool)   — human pressed chess clock
  /perception/changed_squares  (String) — comma-separated changed square names from piece_detector
  /perception/square_scores    (String) — JSON per-square diff scores from piece_detector,
    used for _do_validate_move()'s confidence-scored move matching
  /game_manager/clock_event    (String) — "FLAG_WHITE" or "FLAG_BLACK" from chess_clock_node
  /motion/done                 (Bool)   — motion planner completed move

Service Clients:
  /gantry/home                  (Trigger)     — home gantry
  /camera/capture               (Trigger)     — trigger camera capture
  /perception/capture_premove   (Trigger)     — tell piece_detector to snapshot current board
  /clock/hit                    (Trigger)     — servo presses clock button
  /chess_engine/request_move    (RequestMove) — get best engine move
"""

import json
import os
import threading
import time
from collections import Counter
from pathlib import Path

import chess
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from chess_interfaces.srv import ManualEdit, RequestMove, SetClockTimes, SetPromotion


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
        # Was 5.0 — now covers waiting for a *stable* read (capture_stability_count
        # consecutive identical changed-squares reports), not just the first
        # message to arrive, so it needs more headroom than a single detection
        # tick's worth of time.
        self.declare_parameter('board_capture_timeout_s', 10.0)
        # Consecutive identical /perception/changed_squares reads required
        # before trusting the result as the actual post-move board state —
        # see _do_capture_board()'s docstring for why a single reading isn't
        # reliable (hand still over the board, camera settling, etc).
        self.declare_parameter('capture_stability_count', 3)
        self.declare_parameter('motion_timeout_s', 120.0)
        self.declare_parameter('homing_timeout_s', 90.0)
        # Score-based move matching (see _do_validate_move()/_move_match_score()):
        # per-square scores at/below this are treated as background noise and
        # don't count against a candidate move that doesn't explain them —
        # without this, a move with a small footprint (2 squares) would be
        # penalized just for having a larger complement set than a 4-square
        # castling move, purely from summing near-zero background noise over
        # more squares, not from any real evidence against it.
        self.declare_parameter('match_noise_floor', 8.0)
        # Minimum match score to accept the best-scoring legal move at all —
        # guards against confidently picking an arbitrary "least-bad" legal
        # move when nothing actually explains the observed scores (e.g. a
        # spurious clock press with no real move made).
        self.declare_parameter('min_confident_match_score', 15.0)
        # Where to persist board/history/clock so a crash or Pi restart can
        # resume instead of losing the whole game (see _persist_state()/
        # _load_persisted_state()). Deliberately under the home dir, not the
        # repo tree — this is runtime state, not calibration config.
        self.declare_parameter('game_state_file', str(Path.home() / '.chess' / 'game_state.json'))
        # Mirrors chess_clock_node's own time_per_player_s default — used only
        # to judge whether a clock-time restore on resume is safe (see
        # _load_persisted_state(): only restored if the live clock still
        # looks like a fresh start, so a game_manager-only crash can't roll
        # back a clock that chess_clock_node itself never lost track of).
        self.declare_parameter('time_per_player_s', 600.0)

        self._think_time        = self.get_parameter('engine_think_time_s').value
        self._cap_timeout       = self.get_parameter('board_capture_timeout_s').value
        self._stability_count   = int(self.get_parameter('capture_stability_count').value)
        self._move_timeout      = self.get_parameter('motion_timeout_s').value
        self._home_timeout      = self.get_parameter('homing_timeout_s').value
        self._match_noise_floor = float(self.get_parameter('match_noise_floor').value)
        self._min_match_score   = float(self.get_parameter('min_confident_match_score').value)
        self._state_file        = Path(self.get_parameter('game_state_file').value)
        self._configured_time_per_player = float(self.get_parameter('time_per_player_s').value)

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
        # Manual board correction (see _svc_manual_edit()) — a backup for
        # when automatic vision-based move detection misreads the physical
        # board. Validated synchronously in the service callback; the actual
        # mutation happens in the game-loop thread, same pattern as resign/
        # new_game, to avoid racing the loop's own reads/writes of _board.
        self._manual_edit_event     = threading.Event()
        self._pending_manual_fen    = None

        # Human promotion-piece correction (see _svc_set_promotion()) —
        # _do_validate_move() always resolves promotion ambiguity to queen
        # (all 4 promotion move-types share an identical vision footprint),
        # so this lets the human correct it before confirming via the clock.
        self._set_promotion_event    = threading.Event()
        self._pending_promotion_piece = None   # chess.QUEEN/ROOK/BISHOP/KNIGHT
        self._pending_promotion_move  = None   # the move currently on the board
        self._pending_promotion_is_human = False

        # Set by _load_persisted_state() when a saved game is resumed; blocks
        # the next clock press until the operator confirms (via /game/ack_resume)
        # that the physical board actually matches the resumed position —
        # a resumed FEN is trusted for game logic but was never re-verified
        # against physical reality after whatever crash/restart happened.
        self._resumed_pending_ack = False

        # Move history (UCI strings, one per half-move)
        self._move_history: list = []

        # Latest known clock times (from chess_clock_node's own topics) —
        # cached only so _load_persisted_state() can judge whether a
        # clock-time restore is safe; game_manager_node does not own clock
        # timing itself.
        self._latest_white_time = None
        self._latest_black_time = None

        # Latest values from subscriptions
        self._latest_changed_squares: set = set()   # chess.Square indices from perception
        # Every /perception/changed_squares reading received during the current
        # capture window (piece_detector_node publishes on every ~0.5s detection
        # tick, not just once) — _do_capture_board() waits for capture_stability_count
        # consecutive identical readings here before trusting the result, instead
        # of accepting whatever the very first tick reports (which may catch a
        # hand still over the board, mid-motion blur, or a camera-settling frame).
        self._board_state_history: list = []
        # Latest /perception/square_scores reading (chess.Square -> float),
        # continuously updated. _do_capture_board() snapshots this into
        # _captured_square_scores once it accepts a stable (or mode-fallback)
        # result, for _do_validate_move()'s score-based move matching.
        self._latest_square_scores: dict = {}
        self._captured_square_scores: dict = {}
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
        # Live progress during _do_capture_board()'s stability wait, e.g.
        # "stabilizing: 2/3 consistent reads" — surfaced in chess_ui so an
        # operator can see it waiting out hand movement instead of looking
        # like it's stuck.
        self._capture_progress_pub = self.create_publisher(
            String, '/game_manager/capture_progress', 10)
        # Top-scoring candidate legal moves from the last _do_validate_move()
        # call, e.g. [{"uci":"e2e4","score":142.3}, ...] — transparency into
        # *why* a move was inferred, surfaced in chess_ui.
        self._move_candidates_pub = self.create_publisher(
            String, '/game_manager/move_candidates', 10)
        # Transient — published once whenever _do_validate_move() can't
        # confidently match a legal move, for a chess_ui banner. Not a
        # persistent status field (unlike move_candidates): the same
        # inconclusive board reading is retried on the next capture, so
        # there's no ongoing "rejected" state to reflect, just a one-off
        # notice for the operator to see and retry.
        self._move_rejected_pub = self.create_publisher(
            String, '/game_manager/move_rejected', 10)
        # Persistent (unlike move_rejected) — stays True/False until the next
        # engine call changes it, since "Stockfish is down" is an ongoing
        # integrity concern the operator should keep seeing, not a one-off event.
        self._engine_fallback_pub = self.create_publisher(
            Bool, '/game_manager/engine_used_fallback', 10)

        # ── Subscriptions ─────────────────────────────────────────────────
        self.create_subscription(
            Bool, '/limit_switch/clock_hit', self._on_clock_hit, 10)
        self.create_subscription(
            String, '/perception/changed_squares', self._on_changed_squares, 10)
        self.create_subscription(
            String, '/perception/square_scores', self._on_square_scores, 10)
        self.create_subscription(
            String, '/game_manager/clock_event', self._on_clock_event, 10)
        self.create_subscription(
            Bool, '/motion/done', self._on_motion_done, 10)
        self.create_subscription(
            Float32, '/clock/white_time', self._on_white_time, 10)
        self.create_subscription(
            Float32, '/clock/black_time', self._on_black_time, 10)

        # ── Service clients ───────────────────────────────────────────────
        self._home_client      = self.create_client(Trigger, '/gantry/home')
        self._capture_client   = self.create_client(Trigger, '/camera/capture')
        self._premove_client   = self.create_client(Trigger, '/perception/capture_premove')
        self._clock_hit_client = self.create_client(Trigger, '/clock/hit')
        self._set_clock_times_client = self.create_client(SetClockTimes, '/clock/set_times')
        self._engine_client    = self.create_client(
            RequestMove, '/chess_engine/request_move')

        # ── Game control services (called by Chess OS) ────────────────────
        self.create_service(Trigger, '/game/start',       self._svc_start_game)
        self.create_service(Trigger, '/game/new_game',    self._svc_new_game)
        self.create_service(Trigger, '/game/resign',      self._svc_resign)
        self.create_service(Trigger, '/game/ack_resume',  self._svc_ack_resume)
        self.create_service(ManualEdit, '/game/manual_edit', self._svc_manual_edit)
        self.create_service(SetPromotion, '/game/set_promotion', self._svc_set_promotion)

        # Tells chess_ui whether a resumed game is waiting on operator
        # confirmation (see _resumed_pending_ack above).
        self._resume_ack_pub = self.create_publisher(
            Bool, '/game_manager/resume_pending_ack', 10)
        # Lets chess_ui distinguish a human promotion-choice wait (show Q/R/
        # B/N picker) from a computer one (just informational — the engine's
        # UCI move already unambiguously specifies the piece).
        self._promo_is_human_pub = self.create_publisher(
            Bool, '/game_manager/promotion_is_human', 10)

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
            if self._resumed_pending_ack:
                self.get_logger().warn(
                    'Clock hit ignored — a resumed game is waiting for operator '
                    'confirmation (see chess_ui) that the physical board matches.')
                return
            state = self._state
            if state in (GS.IDLE, GS.WAITING_PLAYER_MOVE, GS.PROMOTION_WAIT):
                self.get_logger().info(f'Clock hit received (state={state})')
                self._clock_hit_event.set()

    def _on_white_time(self, msg: Float32):
        self._latest_white_time = float(msg.data)

    def _on_black_time(self, msg: Float32):
        self._latest_black_time = float(msg.data)

    def _on_changed_squares(self, msg: String):
        """Receive a changed-squares reading from piece_detector_node.

        Appends to _board_state_history rather than just overwriting a single
        "latest" value — piece_detector_node publishes on every detection tick
        (~0.5s), not once per capture, so _do_capture_board() can wait for
        several consecutive identical readings before trusting the result."""
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
        self._board_state_history.append(frozenset(sqs))
        self._board_state_event.set()

    def _on_square_scores(self, msg: String):
        """Continuously-updated cache of piece_detector_node's per-square diff
        scores — not gated on _awaiting_board_state like _on_changed_squares,
        since this is just a live cache; _do_capture_board() snapshots it into
        _captured_square_scores at the moment it accepts a result, for
        _do_validate_move()'s score-based move matching."""
        try:
            raw = json.loads(msg.data)
            self._latest_square_scores = {
                chess.parse_square(name): float(score)
                for name, score in raw.items()
            }
        except Exception as e:
            self.get_logger().debug(f'square_scores decode failed: {e}')

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

        if self._load_persisted_state():
            self._transition(GS.WAITING_PLAYER_MOVE)
            self.get_logger().warn(
                'Resumed saved game — waiting for operator ack, then clock press to continue...')
        else:
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
                self._resumed_pending_ack = False
                self._resume_ack_pub.publish(Bool(data=False))
                self._clear_persisted_state()
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

            # Manual board correction (from Chess OS's Advanced tab, via
            # /game/manual_edit) — validated already in _svc_manual_edit();
            # applied here so it can't race the loop's own board reads/writes.
            # Gives MOTION_ERROR/GAME_OVER a real recovery path (correct the
            # board, keep playing) instead of only a full /game/new_game reset.
            if self._manual_edit_event.is_set():
                self._manual_edit_event.clear()
                fen = self._pending_manual_fen
                self._pending_manual_fen = None
                if fen:
                    self._board = chess.Board(fen)
                    self._pre_move_fen = fen
                    self._move_history.append(f'EDIT:{fen.split(" ")[0]}')
                    self._pub_move_history()
                    self._publish_board_fen()
                    self._persist_state()
                    self.get_logger().warn(f'Manual board edit applied: {fen}')
                    self._transition(GS.WAITING_PLAYER_MOVE)
                    self._do_capture_premove()
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
                    self._move_rejected_pub.publish(String(
                        data='Move not recognized — verify the board and press the clock again'))
                    self._transition(GS.WAITING_PLAYER_MOVE)
                    continue

                # Apply move to internal board
                self._board.push(human_move)
                self._move_history.append(human_move.uci())
                self._pub_move_history()
                self._publish_board_fen()
                self._persist_state()
                self.get_logger().info(
                    f'Human move accepted: {human_move.uci()}  '
                    f'Board: {self._board.fen().split(" ")[0]}')

                # If human promoted, wait for the physical piece swap
                if human_move.promotion is not None:
                    self.get_logger().info(
                        'Human pawn promotion — waiting for player to place promoted piece and press clock...')
                    self._pending_promotion_move = human_move
                    self._pending_promotion_piece = None
                    self._pending_promotion_is_human = True
                    self._promo_is_human_pub.publish(Bool(data=True))
                    self._transition(GS.PROMOTION_WAIT)
                    if not self._wait_promotion_confirm():
                        continue  # flag fell or resign happened during promotion wait
                    self._pending_promotion_is_human = False
                    self._promo_is_human_pub.publish(Bool(data=False))
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
                    self._engine_fallback_pub.publish(Bool(data=True))
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
                self._persist_state()

                # If computer promoted (black pawn to rank 1), wait for user
                if needs_promotion_wait:
                    self._pending_promotion_is_human = False
                    self._promo_is_human_pub.publish(Bool(data=False))
                    self.get_logger().info(
                        f'Computer promoted! Waiting for user to place a '
                        f'{chess.piece_name(computer_move.promotion)} on board...')
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
        """Trigger camera capture and wait for a *stable* changed-squares reading.

        piece_detector_node publishes a changed-squares reading on every
        detection tick (~0.5s) regardless of game state, not just once per
        capture — so instead of trusting whatever the very first tick reports
        after the clock press (which may catch a hand still over the board,
        mid-motion blur, or a camera-settling frame), this waits for
        capture_stability_count consecutive identical readings before
        accepting the result. Falls back to the most-common (mode) reading
        seen if board_capture_timeout_s elapses without one fully stabilizing,
        rather than failing outright — same timeout-with-fallback pattern
        already used for homing retries elsewhere in this node.
        """
        self.get_logger().info(
            f'Capturing post-move board image (waiting for '
            f'{self._stability_count} consecutive stable reads)...')

        self._board_state_event.clear()
        self._board_state_history = []
        self._awaiting_board_state = True
        self._capture_progress_pub.publish(String(
            data=f'Waiting for {self._stability_count} consecutive stable reads...'))

        if self._capture_client.wait_for_service(timeout_sec=3.0):
            self._capture_client.call_async(Trigger.Request())
        else:
            self.get_logger().warn(
                '/camera/capture not available — waiting for changed-squares anyway')

        deadline = time.monotonic() + self._cap_timeout
        stable: 'frozenset | None' = None
        while time.monotonic() < deadline:
            got = self._board_state_event.wait(timeout=0.5)
            self._board_state_event.clear()
            if not got:
                continue
            recent = self._board_state_history[-self._stability_count:]
            streak = 1
            for i in range(len(self._board_state_history) - 1, 0, -1):
                if self._board_state_history[i] != self._board_state_history[i - 1]:
                    break
                streak += 1
            self._capture_progress_pub.publish(String(
                data=f'Stabilizing: {min(streak, self._stability_count)}/'
                     f'{self._stability_count} consistent reads '
                     f'(tick {len(self._board_state_history)})'))
            if len(recent) >= self._stability_count and len(set(recent)) == 1:
                stable = recent[-1]
                break

        self._awaiting_board_state = False
        # Snapshot whatever piece_detector_node's scores were at this moment —
        # used by _do_validate_move()'s score-based matching instead of the
        # thresholded changed_squares set alone.
        self._captured_square_scores = dict(self._latest_square_scores)

        if stable is not None:
            self._latest_changed_squares = set(stable)
            sq_names = sorted(chess.square_name(s) for s in stable)
            self.get_logger().info(
                f'Stable read after {len(self._board_state_history)} tick(s): {sq_names}')
            self._capture_progress_pub.publish(String(
                data=f'Stable after {len(self._board_state_history)} tick(s): '
                     f'{",".join(sq_names) or "(no change)"}'))
            return True

        if self._board_state_history:
            counts = Counter(self._board_state_history)
            mode_result, mode_count = counts.most_common(1)[0]
            self._latest_changed_squares = set(mode_result)
            sq_names = sorted(chess.square_name(s) for s in mode_result)
            self.get_logger().warn(
                f'No stable read within {self._cap_timeout}s — using most-common '
                f'reading ({mode_count}/{len(self._board_state_history)} ticks agreed): {sq_names}')
            self._capture_progress_pub.publish(String(
                data=f'No stable read — used most-common of '
                     f'{len(self._board_state_history)} ticks ({mode_count} agreed): '
                     f'{",".join(sq_names) or "(no change)"}'))
            return True

        self.get_logger().warn('Changed-squares message not received within timeout')
        self._capture_progress_pub.publish(String(data='No changed-squares reading received'))
        return False

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
        Infer the human's move from piece_detector_node's continuous per-
        square diff scores (_captured_square_scores — snapshotted by
        _do_capture_board() at the moment it accepted a stable result), not
        just the boolean thresholded changed-squares set.

        For every legal move, _move_match_score() scores how well its exact
        board footprint (see _get_move_changed_squares() — normal move:
        {from,to}; capture: same; en passant: +captured-pawn square;
        castling: +rook from/to) explains the observed scores, and the
        highest-scoring legal move overall wins. This replaces the previous
        exact-match-then-drop-exactly-one-square approach: that could handle
        either a missed detection OR a false positive, but not both at once,
        and didn't use the continuous confidence it already had access to.
        """
        pre_board = chess.Board(self._pre_move_fen)
        scores = self._captured_square_scores

        if not scores:
            self.get_logger().warn('No per-square scores available — cannot infer move')
            return None

        ranked = [
            (move, self._move_match_score(self._get_move_changed_squares(pre_board, move), scores))
            for move in pre_board.legal_moves
        ]
        if not ranked:
            self.get_logger().warn('No legal moves available to match against')
            return None

        ranked.sort(key=lambda item: item[1], reverse=True)
        top = ranked[:3]
        self.get_logger().info(
            'Top candidate moves: ' + ', '.join(f'{m.uci()}={s:.1f}' for m, s in top))
        self._move_candidates_pub.publish(String(data=json.dumps(
            [{"uci": m.uci(), "score": round(s, 1)} for m, s in top])))

        best_move, best_score = ranked[0]
        if best_score < self._min_match_score:
            self.get_logger().warn(
                f'No confident move match (best={best_move.uci()} '
                f'score={best_score:.1f} < min={self._min_match_score}) '
                f'Pre-FEN: {self._pre_move_fen}')
            return None

        # Promotion ambiguity: several legal moves share the exact same
        # footprint (only the promotion piece type differs) and therefore
        # the same score — prefer queen, matching the old exact-match logic.
        tied = [m for m, s in ranked if abs(s - best_score) < 1e-6]
        if len(tied) > 1:
            queen_promos = [m for m in tied if m.promotion == chess.QUEEN]
            if queen_promos:
                self.get_logger().info(
                    f'Promotion ambiguity resolved to queen: {queen_promos[0].uci()}')
                return queen_promos[0]

        piece = pre_board.piece_at(best_move.from_square)
        pname = chess.piece_name(piece.piece_type) if piece else '?'
        self.get_logger().info(
            f'Inferred move: {best_move.uci()} (score={best_score:.1f}) '
            f'({pname} {chess.square_name(best_move.from_square)}'
            f'→{chess.square_name(best_move.to_square)})'
        )
        return best_move

    def _move_match_score(self, footprint: set, scores: dict) -> float:
        """Score how well `footprint` (a legal move's changed-square set,
        from _get_move_changed_squares()) explains the observed per-square
        diff scores: sum of scores on the footprint squares, minus every
        OTHER square that's still elevated above match_noise_floor and left
        unexplained by this move. Squares near the background noise floor
        contribute ~0 either way, so a small-footprint move (2 squares) isn't
        penalized just for having a larger complement set than a 4-square
        castling move would — only genuinely elevated, unexplained squares
        count against a candidate."""
        explained = sum(scores.get(sq, 0.0) for sq in footprint)
        unexplained = sum(
            max(0.0, score - self._match_noise_floor)
            for sq, score in scores.items()
            if sq not in footprint
        )
        return explained - unexplained

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
        self._engine_fallback_pub.publish(Bool(data=bool(result.used_fallback)))
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
            if self._set_promotion_event.is_set():
                self._set_promotion_event.clear()
                self._apply_promotion_choice()
            if got_clock_hit:
                return True
        return False

    def _apply_promotion_choice(self):
        """Swap the pending human promotion's piece (see _svc_set_promotion())
        — _do_validate_move() always resolves promotion ambiguity to queen
        since all four promotion move-types share an identical vision
        footprint; this lets the human correct that default before
        confirming via the clock."""
        piece = self._pending_promotion_piece
        move = self._pending_promotion_move
        if piece is None or move is None or piece == move.promotion:
            return  # nothing to do — already the requested piece
        self._board.pop()
        if self._move_history:
            self._move_history.pop()
        new_move = chess.Move(move.from_square, move.to_square, promotion=piece)
        self._board.push(new_move)
        self._move_history.append(new_move.uci())
        self._pending_promotion_move = new_move
        self._pub_move_history()
        self._publish_board_fen()
        self._persist_state()
        self.get_logger().info(
            f'Promotion choice updated to {chess.piece_name(piece)}: {new_move.uci()}')

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
        # Nothing meaningful left to resume once a game has actually ended.
        self._clear_persisted_state()

    # ─────────────────────────────────────────────────────────────────────
    # Game-state persistence (crash/restart resume)
    # ─────────────────────────────────────────────────────────────────────

    def _persist_state(self):
        """Atomically snapshot board/history/clock so a crash or Pi restart
        can resume instead of losing the whole game. Called from every place
        that already mutates+republishes the board (human move, computer
        move accepted) — not a periodic timer, so there's no race with the
        loop's own writes."""
        try:
            self._state_file.parent.mkdir(parents=True, exist_ok=True)
            payload = {
                'state':          self._state,
                'fen':            self._board.fen(),
                'pre_move_fen':   self._pre_move_fen,
                'move_history':   self._move_history,
                'white_time_s':   self._latest_white_time,
                'black_time_s':   self._latest_black_time,
                'saved_at':       time.time(),
            }
            tmp = self._state_file.with_suffix('.tmp')
            tmp.write_text(json.dumps(payload, indent=2))
            os.replace(tmp, self._state_file)
        except Exception as e:
            self.get_logger().warn(f'Could not persist game state: {e}')

    def _clear_persisted_state(self):
        """Remove the persisted snapshot — nothing to resume after a fresh
        new-game reset or a real game-over."""
        try:
            self._state_file.unlink(missing_ok=True)
        except Exception as e:
            self.get_logger().warn(f'Could not clear persisted game state: {e}')

    def _load_persisted_state(self) -> bool:
        """Load a previously-persisted game if resumable. Called once, early
        in the game loop, before entering the normal wait-for-first-move
        path. Only ever resumes into WAITING_PLAYER_MOVE (the caller
        transitions there) — a saved mid-motion/mid-capture state can't be
        trusted against real hardware after a crash, so those (and IDLE/
        GAME_OVER, which have nothing worth resuming) are all treated the
        same as "nothing to resume". Returns True if a game was resumed."""
        if not self._state_file.exists():
            return False
        try:
            data = json.loads(self._state_file.read_text())
        except Exception as e:
            self.get_logger().warn(f'Could not read persisted game state: {e}')
            return False

        saved_state = data.get('state')
        fen = data.get('fen')
        if saved_state in (None, GS.IDLE, GS.GAME_OVER) or not fen:
            return False
        try:
            board = chess.Board(fen)
        except ValueError as e:
            self.get_logger().warn(f'Persisted FEN invalid, ignoring resume: {e}')
            return False

        self._board = board
        self._pre_move_fen = data.get('pre_move_fen', fen)
        self._move_history = list(data.get('move_history', []))
        self._pub_move_history()
        self._publish_board_fen()
        self._resumed_pending_ack = True
        self._resume_ack_pub.publish(Bool(data=True))
        self.get_logger().warn(
            f'Resumed a saved game (was {saved_state}) — {len(self._move_history)} '
            f'half-moves. Verify the physical board matches before pressing the clock '
            f'(/game/ack_resume in chess_ui).')

        # Conservative clock restore: only if the live clock still looks like
        # a fresh start (both times within 1s of the configured default) —
        # a game_manager-only crash shouldn't roll back an already-correct
        # clock that chess_clock_node itself never lost track of.
        w, b = self._latest_white_time, self._latest_black_time
        default = self._configured_time_per_player
        if (w is not None and b is not None
                and abs(w - default) < 1.0 and abs(b - default) < 1.0
                and data.get('white_time_s') is not None
                and data.get('black_time_s') is not None):
            if self._set_clock_times_client.wait_for_service(timeout_sec=2.0):
                req = SetClockTimes.Request()
                req.white_time_s = float(data['white_time_s'])
                req.black_time_s = float(data['black_time_s'])
                self._set_clock_times_client.call_async(req)
                self.get_logger().info('Restored persisted clock times.')
            else:
                self.get_logger().warn('/clock/set_times unavailable — clock not restored.')
        else:
            self.get_logger().warn(
                'Live clock does not look freshly-started — skipping clock-time '
                'restore to avoid rolling back an already-correct clock.')
        return True

    # ─────────────────────────────────────────────────────────────────────
    # Game Control Services (called by Chess OS)
    # ─────────────────────────────────────────────────────────────────────

    def _svc_start_game(self, request, response):
        """Trigger a clock-hit event from the UI — starts game or unblocks PROMOTION_WAIT."""
        if self._resumed_pending_ack:
            response.success = False
            response.message = "Resumed game needs operator acknowledgment first (/game/ack_resume)"
            return response
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

    def _svc_manual_edit(self, request, response):
        """Manually correct the board (see ManualEdit.srv) — validated here
        synchronously so the HTTP/service caller gets an immediate accurate
        result; the actual _board mutation happens in the game-loop thread
        (see the manual_edit_event handling at the top of _game_loop)."""
        allowed = (GS.IDLE, GS.WAITING_PLAYER_MOVE, GS.MOTION_ERROR, GS.GAME_OVER)
        if self._state not in allowed:
            response.success = False
            response.message = (f"Cannot manually edit board from state {self._state} "
                                 f"— only allowed from {', '.join(allowed)}")
            return response
        fen = request.fen.strip()
        try:
            board = chess.Board(fen)
        except ValueError as e:
            response.success = False
            response.message = f"Invalid FEN: {e}"
            return response
        if not board.is_valid():
            response.success = False
            response.message = f"Position not legal (chess.Board.status()={board.status()})"
            return response
        self._pending_manual_fen = board.fen()
        self._manual_edit_event.set()
        self._clock_hit_event.set()  # unblock any wait currently in progress
        response.success = True
        response.message = "Manual edit accepted"
        return response

    def _svc_set_promotion(self, request, response):
        """Correct a human promotion's piece choice (see SetPromotion.srv) —
        only valid while PROMOTION_WAIT is active for a human promotion.
        Applied in _wait_promotion_confirm()'s loop, not here directly."""
        if self._state != GS.PROMOTION_WAIT or not self._pending_promotion_is_human:
            response.success = False
            response.message = "Not currently waiting on a human promotion choice"
            return response
        piece_map = {'q': chess.QUEEN, 'r': chess.ROOK, 'b': chess.BISHOP, 'n': chess.KNIGHT}
        piece = piece_map.get(request.piece.strip().lower())
        if piece is None:
            response.success = False
            response.message = "piece must be one of q/r/b/n"
            return response
        self._pending_promotion_piece = piece
        self._set_promotion_event.set()
        response.success = True
        response.message = f"Promotion set to {request.piece.upper()}"
        return response

    def _svc_ack_resume(self, request, response):
        """Operator confirms the physical board matches a resumed saved game
        (see _load_persisted_state()) — required before the next clock press
        is accepted."""
        self._resumed_pending_ack = False
        self._resume_ack_pub.publish(Bool(data=False))
        response.success = True
        response.message = "Resume acknowledged"
        self.get_logger().info('Resumed game acknowledged by operator.')
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
            elif p.name == 'board_capture_timeout_s':
                self._cap_timeout = float(p.value)
                self.get_logger().info(f'board_capture_timeout_s updated to {self._cap_timeout}s')
            elif p.name == 'capture_stability_count':
                self._stability_count = int(p.value)
                self.get_logger().info(f'capture_stability_count updated to {self._stability_count}')
            elif p.name == 'match_noise_floor':
                self._match_noise_floor = float(p.value)
                self.get_logger().info(f'match_noise_floor updated to {self._match_noise_floor}')
            elif p.name == 'min_confident_match_score':
                self._min_match_score = float(p.value)
                self.get_logger().info(f'min_confident_match_score updated to {self._min_match_score}')
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
