#!/usr/bin/env python3
"""
Manual Gantry Control Test — Continuous Velocity Mode + Clock Controls.

Publishes velocity commands to /stepper/velocity (Twist) so the real
stepper_driver_node handles all acceleration, wave generation, and
motor control.

Key-hold model:
  - Key pressed → motor accelerates in that direction (via driver ramp)
  - Key held → motor continues at max speed continuously
  - Key released → motor decelerates smoothly over ~1-2 seconds
  - Multiple keys held → velocity vectors add (normalized for diagonals)

Clock controls:
  - c : hit clock servo (up then down) + switch turn back to player
  - r : reset both chess clocks to full time
  - t : toggle clock start/stop
  - p : switch turn to other player (simulate player hitting clock)

Requires:
  ROS nodes running: stepper_driver_node, limit_switch_node
  Optional: chess_clock_node, clock_servo_node (for clock features)
"""

import curses
import math
import sys
import threading
import time
from typing import Dict, Optional, Set

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from ..base_test import HardwareTest, TestStep

# How long (seconds) a key stays "held" after the last keypress event.
# OS key-repeat fires at ~30Hz (33ms), so 80ms = 2.5× repeat period.
# If we don't see a key for 80ms, the user released it.
KEY_EXPIRY_S = 0.080

# Key identifiers (strings, not ints) for the held-key set
KEY_RIGHT = 'right'
KEY_LEFT  = 'left'
KEY_UP    = 'up'
KEY_DOWN  = 'down'
KEY_NE    = 'ne'    # numpad 9
KEY_NW    = 'nw'    # numpad 7
KEY_SE    = 'se'    # numpad 3
KEY_SW    = 'sw'    # numpad 1


class ManualGantryTestNode(Node):
    """Lightweight ROS node for publishing velocity commands, reading state,
       and controlling the chess clock and clock servo."""

    def __init__(self):
        super().__init__('manual_gantry_test')

        # Publishers — gantry
        self.vel_pub = self.create_publisher(Twist, '/stepper/velocity', 10)
        self.max_vel_pub = self.create_publisher(Float32, '/stepper/set_max_velocity', 10)
        self.accel_pub = self.create_publisher(Float32, '/stepper/set_acceleration', 10)

        # Publishers — clock control
        self._turn_pub = self.create_publisher(String, '/game_manager/turn', 10)
        self._state_pub = self.create_publisher(String, '/game_manager/state', 10)

        # Gantry state
        self.status = 'UNKNOWN'
        self.x_limit = False
        self.y_limit = False
        self.pos_x_mm = 0.0
        self.pos_y_mm = 0.0

        # Clock state
        self.white_time: float = 0.0
        self.black_time: float = 0.0
        self.clock_running: bool = False
        self.current_turn: str = 'NONE'   # 'WHITE', 'BLACK', or 'NONE'

        # Gantry subscriptions
        self.create_subscription(String, '/stepper/status', self._status_cb, 10)
        self.create_subscription(Bool, '/limit_switch/x_min', self._x_limit_cb, 10)
        self.create_subscription(Bool, '/limit_switch/y_min', self._y_limit_cb, 10)
        self.create_subscription(Point, '/gantry/pose', self._pose_cb, 10)

        # Clock subscriptions
        self.create_subscription(Float32, '/clock/white_time', self._white_time_cb, 10)
        self.create_subscription(Float32, '/clock/black_time', self._black_time_cb, 10)

        # Service clients — clock
        self._clock_hit_client = self.create_client(Trigger, '/clock/hit')
        self._clock_reset_client = self.create_client(Trigger, '/clock/reset')
        self._clock_pause_client = self.create_client(Trigger, '/clock/pause')
        self._clock_resume_client = self.create_client(Trigger, '/clock/resume')

    # -- Gantry callbacks --
    def _status_cb(self, msg): self.status = msg.data
    def _x_limit_cb(self, msg): self.x_limit = msg.data
    def _y_limit_cb(self, msg): self.y_limit = msg.data
    def _pose_cb(self, msg):
        self.pos_x_mm = msg.x
        self.pos_y_mm = msg.y

    # -- Clock callbacks --
    def _white_time_cb(self, msg): self.white_time = max(0.0, msg.data)
    def _black_time_cb(self, msg): self.black_time = max(0.0, msg.data)

    # -- Gantry publishing --
    def publish_velocity(self, vx: float, vy: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy)
        self.vel_pub.publish(msg)

    def publish_max_velocity(self, val: float):
        self.max_vel_pub.publish(Float32(data=val))

    def publish_acceleration(self, val: float):
        self.accel_pub.publish(Float32(data=val))

    def stop(self):
        self.publish_velocity(0.0, 0.0)

    # -- Clock operations --
    def hit_clock_servo(self) -> bool:
        """Call /clock/hit service (non-blocking fire-and-forget from bg thread)."""
        if not self._clock_hit_client.service_is_ready():
            self.get_logger().warn('/clock/hit not available')
            return False
        self._clock_hit_client.call_async(Trigger.Request())
        return True

    def reset_clock(self) -> bool:
        """Call /clock/reset service."""
        if not self._clock_reset_client.service_is_ready():
            self.get_logger().warn('/clock/reset not available')
            return False
        self._clock_reset_client.call_async(Trigger.Request())
        return True

    def pause_clock(self) -> bool:
        """Call /clock/pause service."""
        if not self._clock_pause_client.service_is_ready():
            self.get_logger().warn('/clock/pause not available')
            return False
        self._clock_pause_client.call_async(Trigger.Request())
        return True

    def resume_clock(self) -> bool:
        """Call /clock/resume service."""
        if not self._clock_resume_client.service_is_ready():
            self.get_logger().warn('/clock/resume not available')
            return False
        self._clock_resume_client.call_async(Trigger.Request())
        return True

    def set_turn(self, player: str):
        """Publish /game_manager/turn to switch the ticking clock."""
        self._turn_pub.publish(String(data=player))
        self.current_turn = player

    def set_game_state(self, state: str):
        """Publish /game_manager/state for clock node transitions."""
        self._state_pub.publish(String(data=state))


class ManualGantryTest(HardwareTest):
    """
    Interactive continuous-velocity gantry control via ROS.

    Hold a key → continuous acceleration and movement.
    Release key → smooth deceleration and stop.
    Multiple keys → blended velocity vector.

    Also provides chess clock and clock servo controls.
    """

    @property
    def name(self) -> str:
        return 'Manual Gantry (Continuous)'

    @property
    def description(self) -> str:
        return 'Hold-key continuous velocity control with smooth accel/decel + clock controls'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._ros_node: Optional[ManualGantryTestNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._max_speed = 400.0       # steps/sec target velocity
        self._acceleration = 1200.0   # steps/sec² ramp rate
        self._clock_msg = ''          # feedback message for clock actions

    def setup(self) -> bool:
        try:
            if not rclpy.ok():
                rclpy.init()
            self._ros_node = ManualGantryTestNode()
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(0.5)
            self._ros_node.publish_max_velocity(self._max_speed)
            self._ros_node.publish_acceleration(self._acceleration)
            return True
        except Exception as exc:
            print(f'[ERROR] Setup failed: {exc}')
            print('Make sure ROS nodes are running:')
            print('  ros2 launch chess_hw_interface hw_interface_launch.py')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._ros_node)
        except Exception:
            pass

    def teardown(self):
        if self._ros_node:
            try:
                self._ros_node.stop()
            except Exception:
                pass
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def get_steps(self):
        return [
            TestStep(
                name='Continuous Joystick Control',
                display_text='JOYSTK',
                action=self._run_joystick,
                wait_for_input=False,
                success_message='DONE',
                failure_message='FAIL',
            )
        ]

    def _run_joystick(self) -> bool:
        print('\nStarting continuous gantry control...')
        print('Controls: hold arrows/WASD to move, release to coast-stop')
        print('Clock: c=hit servo, r=reset, t=start/stop, p=switch turn')
        print('Requires ROS nodes: stepper_driver_node, limit_switch_node')
        time.sleep(1.0)
        try:
            curses.wrapper(self._control_loop)
            return True
        except Exception as exc:
            print(f'\n[ERROR] Control loop error: {exc}')
            return False

    # ------------------------------------------------------------------
    # Key-expiry held-key model
    # ------------------------------------------------------------------

    def _update_held_keys(
        self,
        held: Dict[str, float],
        key: int,
        now: float,
    ) -> None:
        """
        Update the held-key dict based on a new keypress event.

        held: {key_name: last_seen_timestamp}
        key:  curses key code (-1 means no event this frame)
        now:  current time.time()
        """
        key_map = {
            curses.KEY_RIGHT: KEY_RIGHT,
            ord('d'): KEY_RIGHT, ord('D'): KEY_RIGHT,
            curses.KEY_LEFT:  KEY_LEFT,
            ord('a'): KEY_LEFT,  ord('A'): KEY_LEFT,
            curses.KEY_UP:    KEY_UP,
            ord('w'): KEY_UP,   ord('W'): KEY_UP,
            curses.KEY_DOWN:  KEY_DOWN,
            ord('s'): KEY_DOWN, ord('S'): KEY_DOWN,
            ord('9'): KEY_NE,
            ord('7'): KEY_NW,
            ord('3'): KEY_SE,
            ord('1'): KEY_SW,
        }
        if key in key_map:
            held[key_map[key]] = now

    def _expire_held_keys(self, held: Dict[str, float], now: float) -> None:
        """Remove keys that haven't been seen within KEY_EXPIRY_S."""
        expired = [k for k, t in held.items() if now - t > KEY_EXPIRY_S]
        for k in expired:
            del held[k]

    def _compute_velocity(self, held: Dict[str, float]) -> tuple:
        """
        Compute (vx, vy) from held keys.

        Cardinal keys: +/- max_speed on one axis.
        Diagonal keys: normalized to max_speed magnitude.
        Multiple cardinals: vectors add, normalized if over max_speed.
        """
        vx, vy = 0.0, 0.0

        # Diagonal shortcuts (pre-normalized)
        if KEY_NE in held: vx -= 1.0; vy += 1.0
        if KEY_NW in held: vx += 1.0; vy += 1.0
        if KEY_SE in held: vx -= 1.0; vy -= 1.0
        if KEY_SW in held: vx += 1.0; vy -= 1.0

        # Cardinals
        if KEY_RIGHT in held: vx -= 1.0
        if KEY_LEFT  in held: vx += 1.0
        if KEY_UP    in held: vy += 1.0
        if KEY_DOWN  in held: vy -= 1.0

        # Normalize to max_speed
        mag = math.sqrt(vx * vx + vy * vy)
        if mag > 1e-6:
            scale = self._max_speed / mag if mag > 1.0 else self._max_speed
            vx *= scale
            vy *= scale

        return vx, vy

    # ------------------------------------------------------------------
    # Clock control handlers
    # ------------------------------------------------------------------

    def _handle_clock_hit(self):
        """c key: hit clock servo (up+down) and switch turn back to WHITE (player)."""
        ok = self._ros_node.hit_clock_servo()
        if ok:
            self._ros_node.set_turn('WHITE')
            self._ros_node.clock_running = True
            self._clock_msg = 'Servo hit! → WHITE turn'
        else:
            self._clock_msg = 'Servo N/A — clock_servo_node not running'

    def _handle_clock_reset(self):
        """r key: reset both clocks to full time and stop."""
        ok = self._ros_node.reset_clock()
        if ok:
            self._ros_node.clock_running = False
            self._ros_node.current_turn = 'NONE'
            self._clock_msg = 'Clocks reset to full time'
        else:
            self._clock_msg = 'Reset N/A — chess_clock_node not running'

    def _handle_clock_toggle(self):
        """t key: start clock (WHITE) or pause if already running."""
        if self._ros_node.clock_running:
            ok = self._ros_node.pause_clock()
            if ok:
                self._ros_node.clock_running = False
                self._clock_msg = 'Clock PAUSED'
            else:
                self._clock_msg = 'Pause N/A'
        else:
            # If not running, start with WHITE's turn
            if self._ros_node.current_turn in ('WHITE', 'BLACK'):
                # Resume from where we paused
                ok = self._ros_node.resume_clock()
                if ok:
                    self._ros_node.clock_running = True
                    self._clock_msg = f'Clock RESUMED ({self._ros_node.current_turn})'
                else:
                    self._clock_msg = 'Resume N/A'
            else:
                # Fresh start
                self._ros_node.set_turn('WHITE')
                self._ros_node.clock_running = True
                self._clock_msg = 'Clock STARTED → WHITE turn'

    def _handle_turn_switch(self):
        """p key: switch turn to other player (simulate physical clock press).
        Does NOT fire the clock servo."""
        current = self._ros_node.current_turn
        if current == 'WHITE':
            self._ros_node.set_turn('BLACK')
            self._ros_node.clock_running = True
            self._clock_msg = 'Turn → BLACK (player hit clock)'
        elif current == 'BLACK':
            self._ros_node.set_turn('WHITE')
            self._ros_node.clock_running = True
            self._clock_msg = 'Turn → WHITE'
        else:
            # No turn set yet — start with WHITE
            self._ros_node.set_turn('WHITE')
            self._ros_node.clock_running = True
            self._clock_msg = 'Turn → WHITE (started)'

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def _control_loop(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(16)   # ~60Hz poll — faster than OS key-repeat (30Hz)

        # held_keys: {key_name: last_seen_time}
        held: Dict[str, float] = {}
        running = True
        vx, vy = 0.0, 0.0
        self._clock_msg = ''

        while running:
            now = time.time()
            key = stdscr.getch()

            # --- Handle control/quit keys (not movement) ---
            if key in (ord('q'), ord('Q')):
                self._ros_node.stop()
                running = False
                continue
            elif key == ord(']'):
                self._max_speed = min(1500.0, self._max_speed + 50.0)
                self._ros_node.publish_max_velocity(self._max_speed)
            elif key == ord('['):
                self._max_speed = max(50.0, self._max_speed - 50.0)
                self._ros_node.publish_max_velocity(self._max_speed)
            elif key == ord('.'):
                self._acceleration = min(8000.0, self._acceleration + 200.0)
                self._ros_node.publish_acceleration(self._acceleration)
            elif key == ord(','):
                self._acceleration = max(200.0, self._acceleration - 200.0)
                self._ros_node.publish_acceleration(self._acceleration)

            # --- Clock control keys ---
            elif key in (ord('c'), ord('C')):
                self._handle_clock_hit()
            elif key in (ord('r'), ord('R')):
                self._handle_clock_reset()
            elif key in (ord('t'), ord('T')):
                self._handle_clock_toggle()
            elif key in (ord('p'), ord('P')):
                self._handle_turn_switch()

            # --- Update held-key state ---
            self._update_held_keys(held, key, now)
            self._expire_held_keys(held, now)

            # --- Compute velocity vector from currently held keys ---
            vx, vy = self._compute_velocity(held)

            # --- Always publish — driver handles ramping ---
            self._ros_node.publish_velocity(vx, vy)

            # --- Limit switch safety stop ---
            if self._ros_node.x_limit and vx < 0:
                # Heading into X limit — kill X component
                vx = 0.0
                self._ros_node.publish_velocity(vx, vy)
            if self._ros_node.y_limit and vy > 0:
                vy = 0.0
                self._ros_node.publish_velocity(vx, vy)

            # --- Draw UI ---
            self._draw_ui(stdscr, vx, vy, held)

        stdscr.addstr(35, 0, 'Exiting... motors decelerating.')
        stdscr.refresh()
        time.sleep(0.5)

    def _format_clock_time(self, seconds: float) -> str:
        """Format seconds as MM:SS."""
        total_s = int(seconds)
        minutes = total_s // 60
        secs = total_s % 60
        return f'{minutes:02d}:{secs:02d}'

    def _draw_ui(self, stdscr, vx: float, vy: float, held: Dict[str, float]):
        """Render the curses TUI."""
        try:
            h, w = stdscr.getmaxyx()
            stdscr.clear()

            def safe_addstr(row, col, text, *args):
                if row < h - 1:
                    try:
                        stdscr.addstr(row, col, text[:w - col - 1], *args)
                    except curses.error:
                        pass

            safe_addstr(0, 0, '═' * min(68, w - 1))
            safe_addstr(1, 0, '  CONTINUOUS GANTRY CONTROL + CLOCK  (ROS /stepper/velocity)')
            safe_addstr(2, 0, '═' * min(68, w - 1))

            safe_addstr(4, 0, f'Max Speed:    {self._max_speed:7.0f} steps/sec   [ / ] adjust')
            safe_addstr(5, 0, f'Acceleration: {self._acceleration:7.0f} steps/sec²  , / . adjust')
            safe_addstr(6, 0, f'Driver:       {self._ros_node.status}')
            safe_addstr(7, 0, f'Position:     X={self._ros_node.pos_x_mm:7.1f} mm  '
                               f'Y={self._ros_node.pos_y_mm:7.1f} mm')

            # Limit switch indicators
            lim = ''
            if self._ros_node.x_limit: lim += '  ⚠ X-LIMIT'
            if self._ros_node.y_limit: lim += '  ⚠ Y-LIMIT'
            safe_addstr(8, 0, f'Limits:  {lim if lim else "  clear"}')

            # Velocity vector
            mag = math.sqrt(vx * vx + vy * vy)
            if mag < 1.0:
                dir_str = '● STOPPED'
                arrow = '·'
            else:
                angle = math.degrees(math.atan2(vy, vx))
                directions = [
                    (22.5,  '→ EAST'),   (67.5,  '↗ NE'),
                    (112.5, '↑ NORTH'),  (157.5, '↖ NW'),
                    (180.1, '← WEST'),
                ]
                neg_dirs = [
                    (-22.5,  '→ EAST'),  (-67.5,  '↘ SE'),
                    (-112.5, '↓ SOUTH'), (-157.5, '↙ SW'),
                    (-180.0, '← WEST'),
                ]
                dir_str = '← WEST' if angle <= -157.5 else '→ EAST'
                for threshold, label in directions:
                    if angle <= threshold:
                        dir_str = label
                        break
                for threshold, label in neg_dirs:
                    if angle >= threshold:
                        dir_str = label
                        break

                # Speed bar
                pct = min(1.0, mag / max(self._max_speed, 1.0))
                bar_len = 20
                filled = int(pct * bar_len)
                arrow = f'[{"█" * filled}{"░" * (bar_len - filled)}] {mag:5.0f}/{self._max_speed:.0f}'

            safe_addstr(10, 0, f'Direction:  {dir_str}')
            safe_addstr(11, 0, f'Speed:      {arrow}')

            # Held keys display
            held_names = '  '.join(k for k in [
                KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT,
                KEY_NE, KEY_NW, KEY_SE, KEY_SW
            ] if k in held) or 'none'
            safe_addstr(12, 0, f'Keys held:  {held_names}')

            # Visual direction pad
            up_lit    = '▲' if KEY_UP    in held or KEY_NE in held or KEY_NW in held else '△'
            down_lit  = '▼' if KEY_DOWN  in held or KEY_SE in held or KEY_SW in held else '▽'
            left_lit  = '◄' if KEY_LEFT  in held or KEY_NW in held or KEY_SW in held else '◁'
            right_lit = '►' if KEY_RIGHT in held or KEY_NE in held or KEY_SE in held else '▷'

            safe_addstr(14, 0, '  Direction Pad:')
            safe_addstr(15, 4, f'   {up_lit}')
            safe_addstr(16, 4, f' {left_lit}  ·  {right_lit}')
            safe_addstr(17, 4, f'   {down_lit}')

            # ── Chess Clock Status ──
            safe_addstr(19, 0, '─── Chess Clock ───')
            w_time = self._format_clock_time(self._ros_node.white_time)
            b_time = self._format_clock_time(self._ros_node.black_time)
            turn = self._ros_node.current_turn
            clock_state = 'RUNNING' if self._ros_node.clock_running else 'STOPPED'

            # Highlight the active player's time
            w_indicator = '▶ ' if turn == 'WHITE' and self._ros_node.clock_running else '  '
            b_indicator = '▶ ' if turn == 'BLACK' and self._ros_node.clock_running else '  '

            safe_addstr(20, 0, f'  {w_indicator}White: {w_time}   │   {b_indicator}Black: {b_time}')
            safe_addstr(21, 0, f'  Clock: {clock_state}    Turn: {turn}')
            if self._clock_msg:
                safe_addstr(22, 0, f'  ↳ {self._clock_msg}')

            # ── Controls Help ──
            safe_addstr(24, 0, 'Controls:')
            safe_addstr(25, 2, '↑ ↓ ← →  / WASD : cardinal movement (hold for continuous)')
            safe_addstr(26, 2, '7  9  1  3       : diagonal (NW NE SW SE)')
            safe_addstr(27, 2, '[ / ]            : decrease/increase max speed')
            safe_addstr(28, 2, ', / .            : decrease/increase acceleration')
            safe_addstr(29, 2, 'c                : hit clock servo (up+down) → switch to WHITE')
            safe_addstr(30, 2, 'p                : switch turn (simulate player clock press)')
            safe_addstr(31, 2, 'r                : reset both clocks to full time')
            safe_addstr(32, 2, 't                : toggle clock start/stop')
            safe_addstr(33, 2, 'q                : quit (motors decelerate to stop)')

            safe_addstr(35, 0, 'Release key → smooth deceleration. All motion via ROS.')
            safe_addstr(36, 0, '═' * min(68, w - 1))

            stdscr.refresh()
        except curses.error:
            pass
