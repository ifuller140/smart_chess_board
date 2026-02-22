#!/usr/bin/env python3
"""
Chess Clock Node — manages the game timer for both players.

Maintains two countdown clocks (White and Black), each starting at
time_per_player_s (default 600s = 10 minutes). Publishes remaining
time at 1Hz and detects flag fall (time reaches zero).

The clock runs against whoever's turn it is, INCLUDING the computer's
thinking and move execution time. The human's clock runs from when the
player's clock button is pressed until the servo hits the clock back.

State Machine:
  STOPPED  → clock not running (before game start)
  WHITE    → white player's clock is ticking
  BLACK    → black player's clock is ticking
  PAUSED   → clock temporarily suspended (e.g. pawn promotion by black)

Topics Published:
  /clock/white_time  (std_msgs/Float32) — white remaining seconds
  /clock/black_time  (std_msgs/Float32) — black remaining seconds
  /game_manager/clock_event (std_msgs/String) — "FLAG_WHITE" or "FLAG_BLACK"

Topics Subscribed:
  /game_manager/state (std_msgs/String) — game state transitions
  /game_manager/turn  (std_msgs/String) — "WHITE" or "BLACK" (whose turn)

Services:
  /clock/reset  (Trigger) — reset both clocks to full time
  /clock/pause  (Trigger) — pause the active clock
  /clock/resume (Trigger) — resume the paused clock

Parameters:
  time_per_player_s (float) — initial time per player in seconds (default 600)
"""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class ChessClockNode(Node):
    """
    Dual-player chess clock with flag fall detection.

    Clock states:
      'STOPPED'  — game not started
      'WHITE'    — white's clock ticking
      'BLACK'    — black's clock ticking
      'PAUSED'   — clock halted (e.g. waiting for promotion by black)
    """

    def __init__(self):
        super().__init__('chess_clock_node')

        # ---- Parameters ----
        self.declare_parameter('time_per_player_s', 600.0)
        self._init_time = self.get_parameter('time_per_player_s').value

        # ---- Clock state ----
        self._white_time: float = self._init_time
        self._black_time: float = self._init_time
        self._clock_state: str = 'STOPPED'   # 'STOPPED', 'WHITE', 'BLACK', 'PAUSED'
        self._paused_state: str = 'STOPPED'  # which state to restore after pause
        self._last_tick: float = time.monotonic()
        self._flag_fired: bool = False        # prevent repeated flag events

        # ---- Publishers ----
        self._white_pub = self.create_publisher(Float32, '/clock/white_time', 10)
        self._black_pub = self.create_publisher(Float32, '/clock/black_time', 10)
        self._event_pub = self.create_publisher(String, '/game_manager/clock_event', 10)

        # ---- Subscribers ----
        self.create_subscription(
            String, '/game_manager/state', self._on_game_state, 10)
        self.create_subscription(
            String, '/game_manager/turn', self._on_turn, 10)

        # ---- Services ----
        self.create_service(Trigger, '/clock/reset', self._srv_reset)
        self.create_service(Trigger, '/clock/pause', self._srv_pause)
        self.create_service(Trigger, '/clock/resume', self._srv_resume)

        # ---- 1Hz tick timer ----
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f'Chess clock ready: {self._init_time:.0f}s per player '
            f'({self._init_time/60:.1f} min)')

    # ------------------------------------------------------------------
    # Subscriptions
    # ------------------------------------------------------------------

    def _on_game_state(self, msg: String):
        """
        React to game manager state changes.

        Expected values:
          'WAITING_PLAYER_MOVE' → switch clock to active player
          'EXECUTING_MOVE'      → (clock keeps running on computer's time)
          'HITTING_CLOCK'       → after computer hits clock → switch to human
          'GAME_OVER'           → stop clock
          'HOMING', 'IDLE'      → stop clock
          'PROMOTION_WAIT'      → pause clock (black promotion requires wait)
        """
        state = msg.data.upper()
        self.get_logger().debug(f'Game state: {state}')

        if state in ('IDLE', 'HOMING', 'STARTUP', 'GAME_OVER'):
            self._stop()
        elif state == 'PROMOTION_WAIT':
            # Black promoted — pause clock until user swaps piece and confirms
            self._pause()
        elif state == 'PROMOTION_DONE':
            # User confirmed promotion — resume the clock
            self._resume()

    def _on_turn(self, msg: String):
        """
        Switch the active clock to the given player.

        Expected values: 'WHITE' or 'BLACK'
        """
        turn = msg.data.upper()
        if turn == 'WHITE':
            self._set_active('WHITE')
        elif turn == 'BLACK':
            self._set_active('BLACK')

    # ------------------------------------------------------------------
    # Services
    # ------------------------------------------------------------------

    def _srv_reset(self, request, response):
        """Reset both clocks to full time and stop counting."""
        self._white_time = self._init_time
        self._black_time = self._init_time
        self._clock_state = 'STOPPED'
        self._flag_fired = False
        self._publish()
        response.success = True
        response.message = f'Clocks reset to {self._init_time:.0f}s'
        self.get_logger().info(response.message)
        return response

    def _srv_pause(self, request, response):
        """Pause the active clock (stores state to restore on resume)."""
        self._pause()
        response.success = True
        response.message = f'Clock paused (was {self._paused_state})'
        return response

    def _srv_resume(self, request, response):
        """Resume the paused clock."""
        self._resume()
        response.success = True
        response.message = f'Clock resumed ({self._clock_state})'
        return response

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _set_active(self, player: str):
        """Switch the ticking clock to 'WHITE' or 'BLACK'."""
        self._clock_state = player
        self._last_tick = time.monotonic()
        self._flag_fired = False
        self.get_logger().info(f'Clock now ticking for: {player}')

    def _stop(self):
        """Stop the clock (no ticking)."""
        self._clock_state = 'STOPPED'
        self.get_logger().info('Clock stopped')

    def _pause(self):
        """Pause the clock, saving state for resume."""
        if self._clock_state != 'PAUSED':
            self._paused_state = self._clock_state
            self._clock_state = 'PAUSED'
            self.get_logger().info(f'Clock paused (saving state: {self._paused_state})')

    def _resume(self):
        """Resume from pause."""
        if self._clock_state == 'PAUSED' and self._paused_state in ('WHITE', 'BLACK'):
            self._clock_state = self._paused_state
            self._last_tick = time.monotonic()  # Reset tick reference after pause
            self.get_logger().info(f'Clock resumed ({self._clock_state})')

    def _tick(self):
        """1Hz timer: decrement active player's clock and check for flag fall."""
        now = time.monotonic()

        if self._clock_state == 'WHITE':
            elapsed = now - self._last_tick
            self._last_tick = now
            self._white_time = max(0.0, self._white_time - elapsed)
            if self._white_time <= 0.0 and not self._flag_fired:
                self._fire_flag('WHITE')

        elif self._clock_state == 'BLACK':
            elapsed = now - self._last_tick
            self._last_tick = now
            self._black_time = max(0.0, self._black_time - elapsed)
            if self._black_time <= 0.0 and not self._flag_fired:
                self._fire_flag('BLACK')

        else:
            # STOPPED or PAUSED — update tick reference so we don't
            # count the stopped period as elapsed when we resume
            self._last_tick = now

        self._publish()

    def _fire_flag(self, loser: str):
        """Publish a flag fall event and stop the clock."""
        self._flag_fired = True
        self._clock_state = 'STOPPED'
        event_msg = f'FLAG_{loser}'
        self._event_pub.publish(String(data=event_msg))
        self.get_logger().error(
            f'FLAG FALL: {loser} ran out of time! Publishing {event_msg}')

    def _publish(self):
        """Publish current times to ROS topics."""
        self._white_pub.publish(Float32(data=float(self._white_time)))
        self._black_pub.publish(Float32(data=float(self._black_time)))

    # ------------------------------------------------------------------
    # Public utility (for debugging / status queries)
    # ------------------------------------------------------------------

    def get_state(self) -> str:
        return self._clock_state

    def get_times(self) -> tuple:
        """Returns (white_seconds, black_seconds)."""
        return self._white_time, self._black_time


def main(args=None):
    rclpy.init(args=args)
    node = ChessClockNode()
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
