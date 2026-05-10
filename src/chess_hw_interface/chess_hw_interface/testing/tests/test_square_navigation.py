#!/usr/bin/env python3
"""
Gantry Square Navigation Test.

Homes the gantry, then lets the user type a chess square (e.g. "e4")
and moves the gantry to the center of that square. The user can then
interactively engage and release the servo (permanent magnet) to test
piece pickup at each square.

Coordinate System:
  Origin (0,0): Bottom-LEFT corner (a-file side, player's side)
  +X direction: RIGHTWARD toward h-file / X limit switch (right side)
  +Y direction: BACKWARD toward rank 8 / camera tower

Square Mapping (from board_map.yaml):
  board_origin_x_mm = 20   (center of a1, X from left margin)
  board_origin_y_mm = 20   (center of a1, Y from front margin)
  square_size_mm    = 25

  square_x = board_origin_x_mm + (col_index * square_size_mm)
             where a=0, b=1, ..., h=7
  square_y = board_origin_y_mm + (rank_index * square_size_mm)
             where rank 1=0, rank 2=1, ..., rank 8=7

If board_calibration.json exists, those coordinates override the defaults.

Requires:
  ROS nodes running: stepper_driver_node, homing_node, servo_node,
                     gantry_kinematics_node, limit_switch_node
  Start with: ros2 launch chess_hw_interface hw_interface_launch.py
"""

import json
import threading
import time
from pathlib import Path
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

try:
    from chess_interfaces.action import MoveGantry
    CHESS_INTERFACES_AVAILABLE = True
except ImportError:
    CHESS_INTERFACES_AVAILABLE = False

from ..base_test import HardwareTest, TestStep


# -------------------------------------------------------------------------
# Board geometry constants — defaults; overridden by board_calibration.json
# -------------------------------------------------------------------------
_DEFAULT_ORIGIN_X = 20.0   # X of a1 center (mm from left/origin)
_DEFAULT_ORIGIN_Y = 20.0   # Y of a1 center (mm from front/origin)
_DEFAULT_SQ_SIZE  = 25.0   # Square size (mm)

# Project root: src/chess_hw_interface/chess_hw_interface/testing/tests/ → 5 levels up
_PROJECT_ROOT = Path(__file__).parents[5]
_CALIB_FILE   = _PROJECT_ROOT / "board_calibration.json"


def _load_geometry() -> tuple:
    """Return (origin_x, origin_y, sq_x, sq_y) from calibration file or defaults."""
    if _CALIB_FILE.exists():
        try:
            c = json.loads(_CALIB_FILE.read_text())
            print(f'  [NAV] Using calibration: a1=({c["a1_x"]:.1f},{c["a1_y"]:.1f})'
                  f' sq=({c["sq_x"]:.1f}x{c["sq_y"]:.1f})mm')
            return float(c['a1_x']), float(c['a1_y']), float(c['sq_x']), float(c['sq_y'])
        except Exception as e:
            print(f'  [NAV] Calibration load failed ({e}), using defaults.')
    return _DEFAULT_ORIGIN_X, _DEFAULT_ORIGIN_Y, _DEFAULT_SQ_SIZE, _DEFAULT_SQ_SIZE


FILE_MAP = {'a': 0, 'b': 1, 'c': 2, 'd': 3,
            'e': 4, 'f': 5, 'g': 6, 'h': 7}


def square_to_mm(square: str,
                 origin_x: float = _DEFAULT_ORIGIN_X,
                 origin_y: float = _DEFAULT_ORIGIN_Y,
                 sq_x: float = _DEFAULT_SQ_SIZE,
                 sq_y: float = _DEFAULT_SQ_SIZE) -> tuple:
    """Convert chess square notation (e.g. 'e4') to gantry XY in mm."""
    if len(square) != 2:
        raise ValueError(f'Invalid square: {square!r}')
    file_char = square[0].lower()
    rank_char = square[1]
    if file_char not in FILE_MAP:
        raise ValueError(f'Invalid file: {file_char!r}')
    rank = int(rank_char)
    if rank < 1 or rank > 8:
        raise ValueError(f'Invalid rank: {rank!r}')

    col = FILE_MAP[file_char]   # a=0 … h=7
    row = rank - 1              # rank 1=0 … rank 8=7

    # +X = rightward (a-file at low X, h-file at high X)
    x_mm = origin_x + col * sq_x
    # +Y = backward (rank 1 at low Y, rank 8 at high Y)
    y_mm = origin_y + row * sq_y

    return x_mm, y_mm


# -------------------------------------------------------------------------
# ROS node for navigation
# -------------------------------------------------------------------------

class SquareNavNode(Node):
    """ROS node for square navigation test."""

    def __init__(self):
        super().__init__('square_nav_test_node')

        # Service clients
        self.home_client = self.create_client(Trigger, '/gantry/home')
        self.engage_client = self.create_client(Trigger, '/servo/engage')
        self.release_client = self.create_client(Trigger, '/servo/release')

        # Action client for gantry move
        if CHESS_INTERFACES_AVAILABLE:
            self._move_client = ActionClient(self, MoveGantry, '/gantry/move')
        else:
            self._move_client = None

        # State
        self.homed = False
        self.x_limit = False
        self.y_limit = False
        self.current_x_mm = 0.0
        self.current_y_mm = 0.0

        self.create_subscription(Bool, '/limit_switch/x_min', self._x_cb, 10)
        self.create_subscription(Bool, '/limit_switch/y_min', self._y_cb, 10)

    def _x_cb(self, msg): self.x_limit = msg.data
    def _y_cb(self, msg): self.y_limit = msg.data

    def call_home(self, timeout_s: float = 60.0) -> bool:
        """Call the /gantry/home service and wait."""
        if not self.home_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('  /gantry/home service not available')
            return False
        req = Trigger.Request()
        future = self.home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.result() is not None:
            return future.result().success
        return False

    def call_engage(self) -> bool:
        if not self.engage_client.wait_for_service(timeout_sec=3.0):
            return False
        future = self.engage_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False

    def call_release(self) -> bool:
        if not self.release_client.wait_for_service(timeout_sec=3.0):
            return False
        future = self.release_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result().success if future.result() else False

    def move_to_mm(self, x_mm: float, y_mm: float,
                   timeout_s: float = 30.0) -> bool:
        """Send a MoveGantry goal and wait for completion."""
        if not CHESS_INTERFACES_AVAILABLE or self._move_client is None:
            print('  [ERROR] chess_interfaces not available. Cannot send gantry goal.')
            return False

        if not self._move_client.wait_for_server(timeout_sec=5.0):
            print('  [ERROR] /gantry/move action server not available')
            return False

        goal = MoveGantry.Goal()
        goal.target_x_mm = float(x_mm)
        goal.target_y_mm = float(y_mm)
        goal.speed_mm_s = 50.0
        goal.engage_magnet = False


        goal_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            print('  [ERROR] Move goal rejected')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout_s)

        if result_future.result() is None:
            print('  [ERROR] Move timed out')
            return False

        status = result_future.result().status
        return status == GoalStatus.STATUS_SUCCEEDED


# -------------------------------------------------------------------------
# Test class
# -------------------------------------------------------------------------

class GantrySquareNavigationTest(HardwareTest):
    """
    Interactive test: home gantry, then navigate to chess squares by name.

    The user types a square (e.g. "e4") and the gantry moves there.
    The user can then test the servo magnet at each position.
    Press Ctrl+C or type 'q' to exit.
    """

    @property
    def name(self) -> str:
        return 'Square Navigation'

    @property
    def description(self) -> str:
        return 'Home gantry, navigate to chess squares, test servo at each'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[SquareNavNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._origin_x, self._origin_y, self._sq_x, self._sq_y = _load_geometry()

    def setup(self) -> bool:
        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = SquareNavNode()
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(0.5)
            return True
        except Exception as e:
            print(f'[ERROR] Setup failed: {e}')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def teardown(self):
        if self._node:
            try:
                self._node.call_release()
                self._node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name='Home Gantry',
                display_text='HOMING',
                action=self._do_home,
                wait_for_input=False,
                success_message='HOMED',
                failure_message='HOME ER',
            ),
            TestStep(
                name='Square Navigation',
                display_text='SQ NAV',
                action=self._run_navigation,
                wait_for_input=False,
                success_message='DONE',
                failure_message='FAIL',
            ),
        ]

    # ------------------------------------------------------------------

    def _do_home(self) -> bool:
        """Home the gantry to the limit switches."""
        print('  Homing gantry via /gantry/home service...')
        print('  (Gantry will move to bottom-right corner)')
        ok = self._node.call_home(timeout_s=90.0)
        if ok:
            print('  Homing complete. Gantry is at (0,0).')
        else:
            print('  [ERROR] Homing failed.')
        return ok

    def _run_navigation(self) -> bool:
        """Interactive square navigation loop."""
        print()
        print('  ╔══════════════════════════════════════════════════╗')
        print('  ║         GANTRY SQUARE NAVIGATION TEST            ║')
        print('  ║                                                  ║')
        print('  ║  Type a square (e.g. "e4") and press Enter       ║')
        print('  ║  Then use:                                       ║')
        print('  ║    m = engage magnet (servo down)               ║')
        print('  ║    u = release magnet (servo up)                ║')
        print('  ║    q = quit                                      ║')
        print('  ╚══════════════════════════════════════════════════╝')
        print()

        # Print the square → mm mapping for reference
        print(f'  Geometry: origin=({self._origin_x:.1f},{self._origin_y:.1f})mm '
              f' sq=({self._sq_x:.1f}x{self._sq_y:.1f})mm')
        print('  Reference:')
        for rank in [8, 1]:
            row = []
            for file in 'abcdefgh':
                x, y = square_to_mm(f'{file}{rank}',
                                    self._origin_x, self._origin_y,
                                    self._sq_x, self._sq_y)
                row.append(f'{file}{rank}=({x:.0f},{y:.0f})')
            print(f'    Rank {rank}: ' + '  '.join(row))
        print()

        while True:
            try:
                raw = input('  Enter square (or q): ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if raw == 'q' or raw == 'quit':
                self._node.call_release()
                break

            if raw == 'm':
                print('  → Engaging magnet (servo down)...')
                ok = self._node.call_engage()
                print(f'  {"OK" if ok else "FAILED"}')
                continue

            if raw == 'u':
                print('  → Releasing magnet (servo up)...')
                ok = self._node.call_release()
                print(f'  {"OK" if ok else "FAILED"}')
                continue

            # Try to parse as square
            try:
                x_mm, y_mm = square_to_mm(raw, self._origin_x, self._origin_y,
                                           self._sq_x, self._sq_y)
            except ValueError as e:
                print(f'  [ERROR] {e}')
                print('  Valid squares: a1-h8  (e.g. e4, d7, a1, h8)')
                continue

            print(f'  → Moving to {raw.upper()}: X={x_mm:.1f}mm, Y={y_mm:.1f}mm ...')
            self.show_display(raw.upper())

            ok = self._node.move_to_mm(x_mm, y_mm)
            if ok:
                print(f'  Arrived at {raw.upper()}. (m=engage, u=release, or next square)')
            else:
                print(f'  [ERROR] Move failed. Check gantry_kinematics_node.')

        return True
