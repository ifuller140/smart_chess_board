#!/usr/bin/env python3
"""
Hardware Test Runner CLI.

Examples:
    python3 -m chess_hw_interface.testing.test_runner --list
    python3 -m chess_hw_interface.testing.test_runner --all
    python3 -m chess_hw_interface.testing.test_runner --category gantry --subtest full
    python3 -m chess_hw_interface.testing.test_runner --category gantry --subtest speed_sweep
    python3 -m chess_hw_interface.testing.test_runner --test gantry  # legacy alias
"""

import argparse
import sys
from typing import Dict, Optional, Type
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from .base_test import HardwareTest, TestResult
from .test_display import DisplayInterface, create_display
from .tests.test_camera import CameraTest
from .tests.test_clock import ClockTest
from .tests.test_clock_display import ClockDisplayTest
from .tests.test_gantry import (
    GantryCoreXYDirectionTest,
    GantryDiagonalSyncTest,
    GantryEnableHoldTest,
    GantryFullTest,
    GantryHomingTest,
    GantryLimitSwitchTest,
    GantryLockstepTest,
    GantryMotorATest,
    GantryMotorBTest,
    GantryPulseIntegrityTest,
    GantryRepeatabilityTest,
    GantrySpeedSweepTest,
    GantrySquareReturnTest,
)
from .tests.test_magnet import MagnetTest
from .tests.test_manual_gantry import ManualGantryTest
from .tests.test_raw_motor import RawMotorTest
from .tests.test_servo import ServoTest
from .tests.test_board_calibration import BoardCalibrationTest
from .tests.test_square_navigation import GantrySquareNavigationTest
from .tests.test_timing_sweep import TimingSweepTest
from .tests.test_vision import VisionPipelineTest
from .tests.test_clock_integration import ClockIntegrationTest
from .tests.test_vision_detailed import (
    CameraCornerDetectionTest,
    BoardDetectionTest,
    ColorPieceDetectionTest,
    SquareMappingTest,
    FENBoardDisplayTest,
)


class MockGPIOInterface:
    """No-op GPIO implementation for CLI navigation and dry-runs."""
    is_mock = True

    def setup_output(self, pin: int):
        return None

    def setup_input(self, pin: int, pull_up: bool = True):
        return None

    def write(self, pin: int, value: bool):
        return None

    def read(self, pin: int) -> bool:
        return False

    def read_x_limit(self) -> bool:
        return False

    def read_y_limit(self) -> bool:
        return False

    def read_clock_button(self) -> bool:
        return False

    def cleanup(self):
        return None


class RosMonitorNode(Node):
    """Monitor hardware state via ROS topics."""
    def __init__(self):
        super().__init__('test_runner_monitor')
        self.x_limit = False
        self.y_limit = False
        self.clock_button = False

        # Subscriptions
        self.create_subscription(Bool, '/limit_switch/x_min', self._x_cb, 10)
        self.create_subscription(Bool, '/limit_switch/y_min', self._y_cb, 10)
        self.create_subscription(Bool, '/limit_switch/clock_hit', self._clock_cb, 10)

    def _x_cb(self, msg): self.x_limit = msg.data
    def _y_cb(self, msg): self.y_limit = msg.data
    def _clock_cb(self, msg): self.clock_button = msg.data


class RosInterface:
    """Hardware interface using ROS topics instead of direct GPIO."""
    def __init__(self):
        # Ensure ROS is running (monitor node spun in background thread in main)
        self.monitor_node = None  # Set by main()
        self.is_mock = False

    def setup_output(self, pin: int):
        print(f"[WARN] setup_output({pin}) ignored in ROS mode (requires direct GPIO)")

    def setup_input(self, pin: int, pull_up: bool = True):
        pass  # Inputs handled by limit_switch_node

    def write(self, pin: int, value: bool):
        print(f"[WARN] write({pin}, {value}) ignored in ROS mode")

    def read(self, pin: int) -> bool:
        print(f"[WARN] direct read({pin}) not supported in ROS mode")
        return False

    def read_x_limit(self) -> bool:
        return self.monitor_node.x_limit if self.monitor_node else False

    def read_y_limit(self) -> bool:
        return self.monitor_node.y_limit if self.monitor_node else False

    def read_clock_button(self) -> bool:
        return self.monitor_node.clock_button if self.monitor_node else False

    def cleanup(self):
        pass


class ConsoleDisplay(DisplayInterface):
    """Log display text to console since hardware runs in background node."""
    def show_text(self, text: str):
        # Only log if it's a significant status change to avoid spam
        if text in ['PASS', 'FAIL', 'SETUP', 'DONE']:
            print(f"  [DISPLAY]: {text}")

    def show_time(self, minutes: int, seconds: int, colon: bool = True):
        # We generally don't want to spam console with time updates 2Hz
        pass

    def clear(self):
        pass

    def set_brightness(self, level: int):
        pass
    
    def cleanup(self):
        pass



class NullDisplay(DisplayInterface):
    def show_text(self, text: str):
        return None

    def clear(self):
        return None

    def set_brightness(self, level: int):
        return None


CATEGORY_REGISTRY: Dict[str, Dict[str, Type[HardwareTest]]] = {
    'gantry': {
        'full': GantryFullTest,
        'limits': GantryLimitSwitchTest,
        'pulse': GantryPulseIntegrityTest,
        'motor_a': GantryMotorATest,
        'motor_b': GantryMotorBTest,
        'corexy': GantryCoreXYDirectionTest,
        'speed_sweep': GantrySpeedSweepTest,
        'repeatability': GantryRepeatabilityTest,
        'enable_hold': GantryEnableHoldTest,
        'homing': GantryHomingTest,
        'manual': ManualGantryTest,
        'lockstep': GantryLockstepTest,
        'diagonal_sync': GantryDiagonalSyncTest,
        'square_return': GantrySquareReturnTest,
        'raw_motor': RawMotorTest,
        'timing_sweep': TimingSweepTest,
        'square_nav': GantrySquareNavigationTest,
        'calibrate': BoardCalibrationTest,
    },
    'servo': {
        'full': ServoTest,
    },
    'camera': {
        'full': CameraTest,
    },
    'magnet': {
        'full': MagnetTest,
    },
    'clock': {
        'full': ClockTest,
        'integration': ClockIntegrationTest,
        'display': ClockDisplayTest,
    },
    'vision': {
        'full':    VisionPipelineTest,
        'corners': CameraCornerDetectionTest,
        'board':   BoardDetectionTest,
        'pieces':  ColorPieceDetectionTest,
        'squares': SquareMappingTest,
        'fen':     FENBoardDisplayTest,
    },
}

# Backward-compatible aliases
LEGACY_TEST_ALIASES = {
    'gantry': ('gantry', 'full'),
    'manual_gantry': ('gantry', 'manual'),
    'square_nav': ('gantry', 'square_nav'),
    'calibrate':  ('gantry', 'calibrate'),
    'servo': ('servo', 'full'),
    'camera': ('camera', 'full'),
    'magnet': ('magnet', 'full'),
    'clock': ('clock', 'full'),
    'clock_integration': ('clock', 'integration'),
    'clock_display': ('clock', 'display'),
    'vision': ('vision', 'full'),
    'vision_corners': ('vision', 'corners'),
    'vision_board':   ('vision', 'board'),
    'vision_pieces':  ('vision', 'pieces'),
    'vision_squares': ('vision', 'squares'),
    'vision_fen':     ('vision', 'fen'),
}


def _build_display(display_type: str, mock: bool) -> DisplayInterface:
    if mock:
        return NullDisplay()

    try:
        return create_display(display_type)
    except Exception as exc:
        print(f'[WARN] Display init failed, falling back to null display: {exc}')
        return NullDisplay()


def run_test(
    category: str,
    subtest: str,
    gpio,
    display: DisplayInterface,
    verbose: bool = True,
) -> TestResult:
    test_cls = CATEGORY_REGISTRY[category][subtest]
    test = test_cls(gpio_interface=gpio, display_interface=display)
    return test.run(verbose=verbose)


def list_tests():
    print('\nHardware Test Categories and Subtests:')
    print('-' * 60)
    for category, subtests in CATEGORY_REGISTRY.items():
        print(f'  {category}')
        for subtest, test_cls in subtests.items():
            test = test_cls(gpio_interface=None, display_interface=None)
            print(f'    - {subtest:12} {test.description}')

    print('\nRecommended gantry debugging flow:')
    print('-' * 60)
    print('  1) gantry/raw_motor       Bare-metal per-motor test (start here!)')
    print('  2) gantry/timing_sweep    Find working speed range')
    print('  3) gantry/limits          Confirm switch polarity and wiring')
    print('  4) gantry/pulse           Check pulse timing jitter')
    print('  5) gantry/motor_a + motor_b')
    print('  6) gantry/lockstep        Both motors sync — hardware validation')
    print('  7) gantry/corexy          Verify axis direction mapping')
    print('  8) gantry/diagonal_sync   Diagonal motion — no X/Y wobble')
    print('  9) gantry/speed_sweep     Identify stall zones')
    print(' 10) gantry/square_return   Return-to-origin accuracy')
    print(' 11) gantry/repeatability   Run loop stress test')
    print(' 12) gantry/enable_hold     Validate holding torque behavior')
    print(' 13) gantry/manual          Fine-tune by keyboard control')
    print('  14) gantry/square_nav     Navigate to chess squares by name (e.g. e4)')


    print('\nLegacy aliases (still supported):')
    print('-' * 60)
    for alias, (category, subtest) in LEGACY_TEST_ALIASES.items():
        print(f'  {alias:15} -> {category}/{subtest}')
    print()


def run_all(gpio, display: DisplayInterface, verbose: bool = True) -> Dict[str, TestResult]:
    results: Dict[str, TestResult] = {}
    print('\n' + '=' * 70)
    print('  SMART CHESS BOARD - HARDWARE TEST SUITE')
    print('=' * 70)

    for category, subtests in CATEGORY_REGISTRY.items():
        for subtest in subtests:
            key = f'{category}/{subtest}'
            print(f'\n>>> Running {key} ...')
            results[key] = run_test(category, subtest, gpio, display, verbose)

    print('\n' + '=' * 70)
    print('  TEST SUMMARY')
    print('=' * 70)
    passed = sum(1 for r in results.values() if r == TestResult.PASSED)
    failed = sum(1 for r in results.values() if r == TestResult.FAILED)

    for key, result in results.items():
        status = 'PASS' if result == TestResult.PASSED else 'FAIL'
        print(f'  {key:25} {status}')

    print('-' * 70)
    print(f'  Total: {passed} passed, {failed} failed')
    print('=' * 70 + '\n')
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Smart Chess Board Hardware Test Suite',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  %(prog)s --list
  %(prog)s --all
  %(prog)s --category gantry --subtest full
  %(prog)s --category gantry --subtest limits pulse speed_sweep
  %(prog)s --test gantry manual_gantry
  %(prog)s --category gantry --subtest full --mock
''',
    )

    parser.add_argument('--list', '-l', action='store_true', help='List categories and subtests')
    parser.add_argument('--all', '-a', action='store_true', help='Run all categories/subtests')

    parser.add_argument(
        '--category',
        '-c',
        choices=sorted(CATEGORY_REGISTRY.keys()),
        help='Hardware category to run',
    )
    parser.add_argument(
        '--subtest',
        '-s',
        nargs='+',
        metavar='NAME',
        help='Subtest(s) within selected category (default: full when available)',
    )

    parser.add_argument(
        '--test',
        '-t',
        nargs='+',
        metavar='NAME',
        help='Legacy test alias(es), e.g. gantry, manual_gantry, servo',
    )

    parser.add_argument('--quiet', '-q', action='store_true', help='Minimal output')
    parser.add_argument('--mock', action='store_true', help='Run without real GPIO/display hardware')
    parser.add_argument(
        '--display',
        choices=['tm1637', 'dual_tm1637'],
        default='dual_tm1637',
        help='Display backend when not using --mock',
    )

    return parser.parse_args()


def main() -> int:
    args = parse_args()

    if args.list:
        list_tests()
        return 0

    if not args.all and not args.test and not args.category:
        print('No action specified. Use --list, --all, --category, or --test.')
        return 1

    # Initialize ROS globally for the test runner/monitor
    if not rclpy.ok():
        # Use SignalHandlerOptions.NO to avoid conflict with test logic signals if any
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)

    monitor_node = RosMonitorNode()
    
    # Spin monitor in background
    import threading
    monitor_thread = threading.Thread(target=lambda: rclpy.spin(monitor_node), daemon=True)
    monitor_thread.start()

    if args.mock:
        gpio = MockGPIOInterface()
        display = NullDisplay()
    else:
        # Use RosInterface (replacing GPIOInterface)
        gpio = RosInterface()
        gpio.monitor_node = monitor_node
        # Use ConsoleDisplay (replacing TM1637Display which requires direct GPIO)
        display = ConsoleDisplay()

    # display = _build_display(args.display, args.mock) # Old logic removed
    
    verbose = not args.quiet

    try:
        if args.all:
            results = run_all(gpio, display, verbose)
            return 0 if all(r == TestResult.PASSED for r in results.values()) else 1

        results: Dict[str, TestResult] = {}

        # Legacy aliases
        if args.test:
            for alias in args.test:
                if alias not in LEGACY_TEST_ALIASES:
                    print(f'[ERROR] Unknown legacy test alias: {alias}')
                    print(f"Available aliases: {', '.join(sorted(LEGACY_TEST_ALIASES.keys()))}")
                    return 1
                category, subtest = LEGACY_TEST_ALIASES[alias]
                key = f'{category}/{subtest}'
                print(f'\n>>> Running {key} (alias: {alias}) ...')
                results[key] = run_test(category, subtest, gpio, display, verbose)

        # Category/subtests
        if args.category:
            category = args.category
            available = CATEGORY_REGISTRY[category]
            if args.subtest:
                selected = args.subtest
            else:
                selected = ['full'] if 'full' in available else list(available.keys())

            for subtest in selected:
                if subtest not in available:
                    print(f'[ERROR] Unknown subtest for {category}: {subtest}')
                    print(f"Available: {', '.join(sorted(available.keys()))}")
                    return 1
                key = f'{category}/{subtest}'
                print(f'\n>>> Running {key} ...')
                results[key] = run_test(category, subtest, gpio, display, verbose)

        return 0 if all(r == TestResult.PASSED for r in results.values()) else 1
    finally:
        gpio.cleanup()
        monitor_node.destroy_node()
        if rclpy.ok():
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
