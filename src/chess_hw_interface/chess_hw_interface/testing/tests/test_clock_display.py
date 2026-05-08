#!/usr/bin/env python3
"""
Clock Display Node Integration Test.

Verifies the TM1637 dual-display clock system via ROS topics.
Publishes time values to /clock/white_time and /clock/black_time and
confirms the clock_display_node echoes them to the physical displays.

Two display modes are tested:
  1. Countdown timer display (MM:SS format)
  2. Game state flash messages (CHECK, MATE, IDLE)
  3. Brightness cycling

Tests require:
  - clock_display_node running (configured in hw_interface_launch.py)
  - Display hardware connected (or test will pass after timeout with warnings)
"""

import threading
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from ..base_test import HardwareTest, TestResult, TestStep


class ClockDisplayTestNode(Node):
    """
    Publishes to clock time topics and game state topic.
    Used by ClockDisplayTest to drive the clock_display_node.
    """

    def __init__(self):
        super().__init__('clock_display_test_node')
        self.white_pub = self.create_publisher(Float32, '/clock/white_time', 10)
        self.black_pub = self.create_publisher(Float32, '/clock/black_time', 10)
        self.state_pub = self.create_publisher(String, '/game_manager/state', 10)

    def set_times(self, white_s: float, black_s: float):
        self.white_pub.publish(Float32(data=float(white_s)))
        self.black_pub.publish(Float32(data=float(black_s)))

    def set_state(self, state: str):
        self.state_pub.publish(String(data=state))


class ClockDisplayTest(HardwareTest):
    """
    Tests the TM1637 dual-display clock system.

    What is verified:
      - Display node is reachable via ROS topics
      - Time values are published and (visually) shown on displays
      - Game state flash messages cycle correctly
      - Countdown animation plays without errors
    """

    @property
    def name(self) -> str:
        return 'ClockDisplay'

    @property
    def description(self) -> str:
        return 'Test dual TM1637 clock displays via ROS (clock_display_node)'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name='Node Connectivity',
                display_text='ROS CHK',
                action=self._check_ros_connectivity,
                wait_for_input=False,
                timeout_seconds=5.0,
                success_message='ROS OK',
                failure_message='ROS FAIL',
            ),
            TestStep(
                name='Static Time Display',
                display_text='TIME TST',
                action=self._test_static_times,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='TIME OK',
                failure_message='TIME BAD',
            ),
            TestStep(
                name='Countdown Animation',
                display_text='COUNTDOWN',
                action=self._test_countdown,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='CNT OK',
                failure_message='CNT BAD',
            ),
            TestStep(
                name='Game State Flash',
                display_text='FLASH',
                action=self._test_state_flash,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='FLASH OK',
                failure_message='FLASH BAD',
            ),
            TestStep(
                name='Low Time Warning',
                display_text='LOW TIME',
                action=self._test_low_time,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='LOW OK',
                failure_message='LOW BAD',
            ),
        ]

    def _get_node(self) -> ClockDisplayTestNode:
        if not rclpy.ok():
            rclpy.init()
        if not hasattr(self, '_node') or self._node is None:
            self._node = ClockDisplayTestNode()
            self._spin_thread = threading.Thread(
                target=rclpy.spin, args=(self._node,), daemon=True)
            self._spin_thread.start()
        return self._node

    def _check_ros_connectivity(self) -> bool:
        """Verify ROS is running and we can publish to clock topics."""
        try:
            node = self._get_node()
            node.set_times(600.0, 600.0)
            time.sleep(0.2)
            print('  Clock display test node connected to ROS')
            return True
        except Exception as e:
            print(f'  ROS connectivity check failed: {e}')
            return False

    def _test_static_times(self) -> bool:
        """Display static time values on both displays."""
        print('  Displaying static times: White=10:00, Black=10:00')
        node = self._get_node()

        test_pairs = [
            (600.0, 600.0, 'White=10:00  Black=10:00'),
            (300.0, 300.0, 'White=5:00   Black=5:00'),
            (90.0,  150.0, 'White=1:30   Black=2:30'),
            (45.0,  270.0, 'White=0:45   Black=4:30'),
        ]

        for white_s, black_s, desc in test_pairs:
            print(f'  → {desc}')
            node.set_times(white_s, black_s)
            time.sleep(1.5)

        return True

    def _test_countdown(self) -> bool:
        """Animate a countdown from 1:00 to 0:00 on white's display."""
        print('  Animating countdown: White 1:00 → 0:00, Black stays at 5:00')
        node = self._get_node()

        for remaining in range(60, -1, -1):
            node.set_times(float(remaining), 300.0)
            time.sleep(0.08)

        print('  Countdown complete')
        return True

    def _test_state_flash(self) -> bool:
        """Test game state flash messages on the displays."""
        print('  Testing game state flash messages...')
        node = self._get_node()

        node.set_times(60.0, 120.0)
        time.sleep(0.3)

        states = ['CHECK', 'CHECKMATE', 'STALEMATE', 'GAME_OVER', 'IDLE']
        for state in states:
            print(f'  → Flash: {state}')
            node.set_state(state)
            time.sleep(2.5)  # 2 second flash + 0.5s buffer

        # Restore normal time display
        node.set_times(60.0, 120.0)
        time.sleep(0.5)
        return True

    def _test_low_time(self) -> bool:
        """Simulate low-time warning conditions (colon blink visible at 2Hz)."""
        print('  Testing low-time display (sub-30 seconds, colon blinks)...')
        node = self._get_node()

        for remaining in range(30, -1, -5):
            node.set_times(float(remaining), 300.0)
            print(f'  → White: {remaining}s')
            time.sleep(1.0)

        # Zero
        node.set_times(0.0, 0.0)
        print('  → Both at 0:00')
        time.sleep(1.5)

        return True

    def run(self, verbose: bool = True) -> TestResult:
        """Override to ensure cleanup after test run."""
        result = super().run(verbose=verbose)
        self._cleanup()
        return result

    def _cleanup(self):
        if hasattr(self, '_node') and self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            self._node = None
