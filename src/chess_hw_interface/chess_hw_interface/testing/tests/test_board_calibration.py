#!/usr/bin/env python3
"""
Board Calibration Verification Test.

Loads board_calibration.json (created via the Chess OS Gantry → Calibration
workflow), prints all 64 computed square positions, and optionally runs a
physical verification pass by homing the gantry and moving to corner squares
for the user to confirm visually.

Workflow to create calibration (Chess OS):
  1. Gantry tab → Engage magnet
  2. Place a piece on the magnet tip
  3. Jog gantry to center of a1 square
  4. Click "Save a1"
  5. Jog gantry to center of h8 square
  6. Click "Save h8"
  7. Click "Apply" — saves board_calibration.json

Then run this test to verify the result.

Requires (for physical verification):
  ROS nodes: stepper_driver_node, homing_node, servo_node,
             gantry_kinematics_node, limit_switch_node
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
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

try:
    from chess_interfaces.action import MoveGantry
    CHESS_INTERFACES_AVAILABLE = True
except ImportError:
    CHESS_INTERFACES_AVAILABLE = False

from ..base_test import HardwareTest, TestStep

# Project root: parents[5] from this file's location
_PROJECT_ROOT = Path(__file__).parents[5]
_CALIB_FILE   = _PROJECT_ROOT / "board_calibration.json"

FILE_LETTERS = 'abcdefgh'


def _compute_all_squares(a1_x: float, a1_y: float,
                         sq_x: float, sq_y: float) -> dict:
    """Return {square: (x_mm, y_mm)} for all 64 squares."""
    squares = {}
    for fi, f in enumerate(FILE_LETTERS):
        for rank in range(1, 9):
            ri = rank - 1
            sq = f'{f}{rank}'
            squares[sq] = (round(a1_x + fi * sq_x, 2),
                           round(a1_y + ri * sq_y, 2))
    return squares


# ─────────────────────────────────────────────────────────────────────────────
# ROS node (reuses pattern from test_square_navigation)
# ─────────────────────────────────────────────────────────────────────────────

class CalibNavNode(Node):
    def __init__(self):
        super().__init__('board_calib_test_node')

        self.home_client    = self.create_client(Trigger, '/gantry/home')
        self.engage_client  = self.create_client(Trigger, '/servo/engage')
        self.release_client = self.create_client(Trigger, '/servo/release')

        if CHESS_INTERFACES_AVAILABLE:
            self._move_client = ActionClient(self, MoveGantry, '/gantry/move')
        else:
            self._move_client = None

        self.x_limit = False
        self.y_limit = False
        self.create_subscription(Bool, '/limit_switch/x_min', lambda m: setattr(self, 'x_limit', m.data), 10)
        self.create_subscription(Bool, '/limit_switch/y_min', lambda m: setattr(self, 'y_limit', m.data), 10)

    def call_home(self, timeout_s: float = 90.0) -> bool:
        if not self.home_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self.home_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        return bool(future.result() and future.result().success)

    def call_engage(self) -> bool:
        if not self.engage_client.wait_for_service(timeout_sec=3.0):
            return False
        future = self.engage_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def call_release(self) -> bool:
        if not self.release_client.wait_for_service(timeout_sec=3.0):
            return False
        future = self.release_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def move_to(self, x_mm: float, y_mm: float, timeout_s: float = 45.0) -> bool:
        if not CHESS_INTERFACES_AVAILABLE or self._move_client is None:
            print('  [ERROR] chess_interfaces not available')
            return False
        if not self._move_client.wait_for_server(timeout_sec=5.0):
            print('  [ERROR] /gantry/move action server not available')
            return False
        goal = MoveGantry.Goal()
        goal.target_x_mm  = float(x_mm)
        goal.target_y_mm  = float(y_mm)
        goal.speed_mm_s   = 40.0
        goal.engage_magnet = False
        gf = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gf, timeout_sec=10.0)
        gh = gf.result()
        if not gh or not gh.accepted:
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout_s)
        return rf.result() is not None and rf.result().status == GoalStatus.STATUS_SUCCEEDED


# ─────────────────────────────────────────────────────────────────────────────
# Test class
# ─────────────────────────────────────────────────────────────────────────────

class BoardCalibrationTest(HardwareTest):
    """
    Verify board calibration from board_calibration.json.

    Prints all 64 computed square positions. Optionally homes the gantry
    and moves to corner/center squares so the user can confirm placement.
    """

    @property
    def name(self) -> str:
        return 'Board Calibration Verification'

    @property
    def description(self) -> str:
        return 'Load board_calibration.json, print all squares, optional physical verify'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[CalibNavNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._calib: Optional[dict] = None

    def setup(self) -> bool:
        if not _CALIB_FILE.exists():
            print(f'\n  [FAIL] Calibration file not found: {_CALIB_FILE}')
            print('  Run the Board Calibration workflow in Chess OS first:')
            print('  Gantry tab → engage magnet → jog to a1 → Save a1 → jog to h8 → Save h8 → Apply')
            return False
        try:
            self._calib = json.loads(_CALIB_FILE.read_text())
            required = ('a1_x', 'a1_y', 'h8_x', 'h8_y', 'sq_x', 'sq_y')
            if not all(k in self._calib for k in required):
                print(f'  [FAIL] Calibration file missing keys. Expected: {required}')
                return False
        except Exception as e:
            print(f'  [FAIL] Cannot read calibration file: {e}')
            return False

        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = CalibNavNode()
            self._spin_thread = threading.Thread(target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(0.5)
        except Exception as e:
            print(f'  [WARN] ROS init failed ({e}) — table-only mode, no physical movement')

        return True

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
                name='Print Calibration',
                display_text='CALIB',
                action=self._print_calibration,
                wait_for_input=False,
                success_message='LISTED',
                failure_message='NO FILE',
            ),
            TestStep(
                name='Physical Verification',
                display_text='VERIFY',
                action=self._physical_verify,
                wait_for_input=False,
                success_message='DONE',
                failure_message='SKIP',
            ),
        ]

    # ──────────────────────────────────────────────────────────────────────────

    def _print_calibration(self) -> bool:
        c = self._calib
        print()
        print('  ╔══════════════════════════════════════════════════════╗')
        print('  ║            BOARD CALIBRATION LOADED                  ║')
        print('  ╠══════════════════════════════════════════════════════╣')
        print(f'  ║  a1 center : ({c["a1_x"]:6.1f}, {c["a1_y"]:6.1f}) mm                  ║')
        print(f'  ║  h8 center : ({c["h8_x"]:6.1f}, {c["h8_y"]:6.1f}) mm                  ║')
        print(f'  ║  Square X  : {c["sq_x"]:6.2f} mm                              ║')
        print(f'  ║  Square Y  : {c["sq_y"]:6.2f} mm                              ║')
        if 'calibrated_at' in c:
            print(f'  ║  Date      : {c["calibrated_at"]:<38} ║')
        print('  ╚══════════════════════════════════════════════════════╝')
        print()

        squares = _compute_all_squares(c['a1_x'], c['a1_y'], c['sq_x'], c['sq_y'])

        print('  All 64 squares (X, Y in mm):')
        print('  ' + '─' * 64)
        header = '        ' + ''.join(f'  {f}   ' for f in FILE_LETTERS)
        print(header)
        for rank in range(8, 0, -1):
            row_parts = [f'  Rank {rank} |']
            for f in FILE_LETTERS:
                x, y = squares[f'{f}{rank}']
                row_parts.append(f'({x:3.0f},{y:3.0f})')
            print(' '.join(row_parts))
        print()

        # Sanity check
        sq_x, sq_y = c['sq_x'], c['sq_y']
        if sq_x < 10 or sq_y < 10:
            print(f'  [WARN] Square size {sq_x:.1f}x{sq_y:.1f}mm is suspiciously small.')
        elif abs(sq_x - 25) > 5 or abs(sq_y - 25) > 5:
            print(f'  [WARN] Square size {sq_x:.1f}x{sq_y:.1f}mm deviates >5mm from standard 25mm.')
        else:
            print(f'  [OK] Square sizes ({sq_x:.1f}x{sq_y:.1f}mm) look reasonable.')

        return True

    def _physical_verify(self) -> bool:
        if self._node is None:
            print('  ROS not available — skipping physical verification.')
            return True  # not a failure

        try:
            ans = input('\n  Run physical verification? (y/N): ').strip().lower()
        except (EOFError, KeyboardInterrupt):
            return True

        if ans != 'y':
            print('  Skipped.')
            return True

        c = self._calib
        squares = _compute_all_squares(c['a1_x'], c['a1_y'], c['sq_x'], c['sq_y'])

        # Verification squares: a1, h8, d4, e5 (corners + center pair)
        verify_list = ['a1', 'h8', 'd4', 'e5', 'a8', 'h1']

        print('\n  Homing gantry…')
        if not self._node.call_home(timeout_s=90.0):
            print('  [ERROR] Homing failed. Check ROS nodes.')
            return False
        print('  Homed. Engaging magnet — place a piece on it.')
        self._node.call_engage()

        all_ok = True
        for sq in verify_list:
            x, y = squares[sq]
            print(f'\n  → Moving to {sq.upper()} ({x:.1f}, {y:.1f}) mm…')
            self.show_display(sq.upper())
            ok = self._node.move_to(x, y, timeout_s=30.0)
            if not ok:
                print(f'  [ERROR] Move to {sq} failed.')
                all_ok = False
                continue
            try:
                ans = input(f'  Does gantry appear over {sq.upper()} center? (y/N): ').strip().lower()
                if ans != 'y':
                    print(f'  [WARN] User reported {sq} position incorrect.')
                    all_ok = False
            except (EOFError, KeyboardInterrupt):
                break

        # Release and return to origin
        self._node.call_release()
        print('\n  Returning to origin…')
        self._node.move_to(0.0, 0.0, timeout_s=30.0)

        if all_ok:
            print('  ✓ All verified positions accepted by user.')
        else:
            print('  ✗ Some positions were rejected — recalibrate in Chess OS.')

        return all_ok
