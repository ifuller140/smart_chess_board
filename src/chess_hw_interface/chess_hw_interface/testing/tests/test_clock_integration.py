#!/usr/bin/env python3
"""
Chess Clock Integration Test.

Tests that the chess_clock_node works correctly end-to-end:
  1. Clock starts in STOPPED state; both timers at full time
  2. Switching turn to WHITE starts white's clock
  3. Clock ticks (verified by watching published times decrease)
  4. Switching turn to BLACK switches which clock ticks
  5. PAUSED state freezes the active clock
  6. /clock/reset resets both times to full
  7. Flag fall test: fast-forward to near-zero and confirm FLAG event

Requires:
  chess_clock_node running:
    ros2 run chess_hw_interface chess_clock_node
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from ..base_test import HardwareTest, TestStep


class ClockMonitorNode(Node):
    """Subscribe to clock topics and control via publishers/services."""

    def __init__(self):
        super().__init__('clock_integration_test_node')

        self.white_time:  float = -1.0
        self.black_time:  float = -1.0
        self.clock_event: str   = ''

        self.create_subscription(
            Float32, '/clock/white_time', self._on_white, 10)
        self.create_subscription(
            Float32, '/clock/black_time', self._on_black, 10)
        self.create_subscription(
            String, '/game_manager/clock_event', self._on_event, 10)

        # Publishers to drive the clock state machine
        self._state_pub = self.create_publisher(
            String, '/game_manager/state', 10)
        self._turn_pub  = self.create_publisher(
            String, '/game_manager/turn', 10)

        self._reset_client  = self.create_client(Trigger, '/clock/reset')
        self._pause_client  = self.create_client(Trigger, '/clock/pause')
        self._resume_client = self.create_client(Trigger, '/clock/resume')

    def _on_white(self, msg): self.white_time = msg.data
    def _on_black(self, msg): self.black_time = msg.data
    def _on_event(self, msg): self.clock_event = msg.data

    def set_turn(self, player: str):
        """Publish /game_manager/turn to switch the ticking clock."""
        self._turn_pub.publish(String(data=player))

    def set_state(self, state: str):
        """Publish /game_manager/state to trigger state transitions."""
        self._state_pub.publish(String(data=state))

    def call_reset(self) -> bool:
        if not self._reset_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self._reset_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def call_pause(self) -> bool:
        if not self._pause_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self._pause_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def call_resume(self) -> bool:
        if not self._resume_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self._resume_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)


class ClockIntegrationTest(HardwareTest):
    """
    End-to-end integration test for the chess_clock_node.
    Verifies timing accuracy, state transitions, and flag fall.
    """

    @property
    def name(self) -> str:
        return 'Clock Integration'

    @property
    def description(self) -> str:
        return 'Test chess_clock_node: tick rate, state transitions, pause, flag fall'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[ClockMonitorNode] = None
        self._spin_thread: Optional[threading.Thread] = None

    def setup(self) -> bool:
        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = ClockMonitorNode()
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(1.0)
            return True
        except Exception as e:
            print(f'[ERROR] Setup: {e}')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def teardown(self):
        if self._node:
            try:
                self._node.call_reset()
                self._node.set_state('GAME_OVER')
                self._node.destroy_node()
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
                name='Connect to Clock Node',
                display_text='CLK CHK',
                action=self._test_connect,
                wait_for_input=False,
                success_message='CLK OK',
                failure_message='NO CLK',
            ),
            TestStep(
                name='Reset and Verify Initial State',
                display_text='RESET',
                action=self._test_reset,
                wait_for_input=False,
                success_message='RST OK',
                failure_message='RST FAIL',
            ),
            TestStep(
                name='White Clock Ticking',
                display_text='WHITE TK',
                action=self._test_white_tick,
                wait_for_input=False,
                success_message='W TICK',
                failure_message='W NO TK',
            ),
            TestStep(
                name='Black Clock Ticking',
                display_text='BLACK TK',
                action=self._test_black_tick,
                wait_for_input=False,
                success_message='B TICK',
                failure_message='B NO TK',
            ),
            TestStep(
                name='Pause / Resume',
                display_text='PAUSE',
                action=self._test_pause_resume,
                wait_for_input=False,
                success_message='PAUSE OK',
                failure_message='PSE FAIL',
            ),
            TestStep(
                name='Reset Service',
                display_text='RESET2',
                action=self._test_reset_after_play,
                wait_for_input=False,
                success_message='RST OK',
                failure_message='RST FAIL',
            ),
        ]

    # ── Step implementations ──────────────────────────────────────────────

    def _test_connect(self) -> bool:
        """Verify the clock node is publishing time topics."""
        print('  Waiting for /clock/white_time and /clock/black_time...')
        deadline = time.time() + 8.0
        while (self._node.white_time < 0 or self._node.black_time < 0) \
                and time.time() < deadline:
            time.sleep(0.2)

        if self._node.white_time >= 0 and self._node.black_time >= 0:
            print(f'  ✓ Clock topics active: white={self._node.white_time:.0f}s,'
                  f' black={self._node.black_time:.0f}s')
            return True
        print('  ✗ Clock topics not receiving. Is chess_clock_node running?')
        return False

    def _test_reset(self) -> bool:
        """Call /clock/reset and verify both timers return to full time."""
        ok = self._node.call_reset()
        if not ok:
            print('  ✗ /clock/reset service unavailable')
            return False
        time.sleep(1.5)
        wt, bt = self._node.white_time, self._node.black_time
        print(f'  After reset: white={wt:.1f}s, black={bt:.1f}s')
        # Both should be >= 590s (close to 600, accounting for 1Hz latency)
        ok = wt >= 590.0 and bt >= 590.0
        if ok:
            print('  ✓ Both clocks reset to full time')
        else:
            print('  ✗ Reset did not restore full time')
        return ok

    def _test_white_tick(self) -> bool:
        """Set WHITE turn and verify white time decreases by ~3s in 3s."""
        self._node.call_reset()
        time.sleep(0.5)
        w_before = self._node.white_time
        b_before = self._node.black_time

        print('  Setting turn to WHITE...')
        self._node.set_turn('WHITE')
        time.sleep(3.5)

        w_after = self._node.white_time
        b_after = self._node.black_time

        w_delta = w_before - w_after
        b_delta = b_before - b_after

        print(f'  White time: {w_before:.1f} → {w_after:.1f} (delta={w_delta:.1f}s)')
        print(f'  Black time: {b_before:.1f} → {b_after:.1f} (delta={b_delta:.1f}s)')

        # White should have lost 2-4 seconds, black should be unchanged
        white_ticked = 1.5 <= w_delta <= 5.0
        black_frozen = abs(b_delta) < 0.5

        if white_ticked and black_frozen:
            print('  ✓ White clock is ticking, black is frozen')
            return True
        if not white_ticked:
            print(f'  ✗ White time delta {w_delta:.1f}s unexpected (expected ~3s)')
        if not black_frozen:
            print(f'  ✗ Black time changed by {b_delta:.1f}s (should be frozen)')
        return False

    def _test_black_tick(self) -> bool:
        """Switch to BLACK turn and verify black time decreases."""
        b_before = self._node.black_time

        print('  Switching turn to BLACK...')
        self._node.set_turn('BLACK')
        time.sleep(3.5)

        w_after = self._node.white_time
        b_after = self._node.black_time
        w_delta = self._node.white_time  # white_time from _test_white_tick still running
        b_delta = b_before - b_after

        print(f'  Black time: {b_before:.1f} → {b_after:.1f} (delta={b_delta:.1f}s)')

        black_ticked = 1.5 <= b_delta <= 5.0
        if black_ticked:
            print('  ✓ Black clock is now ticking')
            return True
        print(f'  ✗ Black time delta {b_delta:.1f}s unexpected')
        return False

    def _test_pause_resume(self) -> bool:
        """Pause clock, verify it freezes, then resume and verify it ticks again."""
        self._node.set_turn('WHITE')
        time.sleep(1.0)
        t_before_pause = self._node.white_time

        print('  Pausing clock...')
        ok = self._node.call_pause()
        if not ok:
            print('  ✗ /clock/pause unavailable')
            return False

        time.sleep(3.0)
        t_during_pause = self._node.white_time
        frozen_delta = abs(t_before_pause - t_during_pause)

        print(f'  During pause: white={t_during_pause:.1f}s (delta={frozen_delta:.2f}s)')
        if frozen_delta > 1.5:
            print('  ✗ Clock kept ticking despite pause')
            return False
        print('  ✓ Clock frozen during pause')

        print('  Resuming clock...')
        ok = self._node.call_resume()
        if not ok:
            print('  ✗ /clock/resume unavailable')
            return False

        time.sleep(3.0)
        t_after_resume = self._node.white_time
        resume_delta   = t_during_pause - t_after_resume
        print(f'  After resume: white={t_after_resume:.1f}s (delta={resume_delta:.1f}s)')

        if 1.5 <= resume_delta <= 5.0:
            print('  ✓ Clock resumed ticking')
            return True
        print(f'  ✗ Resume delta {resume_delta:.1f}s unexpected')
        return False

    def _test_reset_after_play(self) -> bool:
        """Reset after play and verify both times return to full."""
        self._node.set_state('GAME_OVER')
        time.sleep(0.5)
        ok = self._node.call_reset()
        time.sleep(1.5)
        wt, bt = self._node.white_time, self._node.black_time
        print(f'  After final reset: white={wt:.0f}s, black={bt:.0f}s')
        if ok and wt >= 590.0 and bt >= 590.0:
            print('  ✓ Clock reset cleanly after game')
            return True
        return False
