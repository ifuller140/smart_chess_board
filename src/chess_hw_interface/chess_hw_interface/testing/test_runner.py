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

from .base_test import HardwareTest, TestResult
from .test_display import DisplayInterface, create_display
from .tests.test_camera import CameraTest
from .tests.test_clock import ClockTest
from .tests.test_gantry import (
    GantryCoreXYDirectionTest,
    GantryEnableHoldTest,
    GantryFullTest,
    GantryHomingTest,
    GantryLimitSwitchTest,
    GantryMotorATest,
    GantryMotorBTest,
    GantryPulseIntegrityTest,
    GantryRepeatabilityTest,
    GantrySpeedSweepTest,
)
from .tests.test_magnet import MagnetTest
from .tests.test_manual_gantry import ManualGantryTest
from .tests.test_servo import ServoTest


class GPIOInterface:
    """GPIO wrapper around RPi.GPIO with active-low limit switch helpers."""

    PIN_X_LIMIT = 10
    PIN_Y_LIMIT = 9
    PIN_CLOCK_BUTTON = 15

    def __init__(self):
        import RPi.GPIO as GPIO

        self.GPIO = GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        # Active HIGH switches with pull-downs (pressed = 3.3V = HIGH).
        GPIO.setup(self.PIN_X_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.PIN_Y_LIMIT, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.setup(self.PIN_CLOCK_BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    def setup_output(self, pin: int):
        self.GPIO.setup(pin, self.GPIO.OUT)

    def setup_input(self, pin: int, pull_up: bool = True):
        pud = self.GPIO.PUD_UP if pull_up else self.GPIO.PUD_DOWN
        self.GPIO.setup(pin, self.GPIO.IN, pull_up_down=pud)

    def write(self, pin: int, value: bool):
        self.GPIO.output(pin, self.GPIO.HIGH if value else self.GPIO.LOW)

    def read(self, pin: int) -> bool:
        return self.GPIO.input(pin) == self.GPIO.HIGH

    def read_x_limit(self) -> bool:
        return self.read(self.PIN_X_LIMIT)  # Active HIGH: 1 = pressed

    def read_y_limit(self) -> bool:
        return self.read(self.PIN_Y_LIMIT)  # Active HIGH: 1 = pressed

    def read_clock_button(self) -> bool:
        return self.read(self.PIN_CLOCK_BUTTON)  # Active HIGH: 1 = pressed

    def cleanup(self):
        self.GPIO.cleanup()


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
    },
}

# Backward-compatible aliases
LEGACY_TEST_ALIASES = {
    'gantry': ('gantry', 'full'),
    'manual_gantry': ('gantry', 'manual'),
    'servo': ('servo', 'full'),
    'camera': ('camera', 'full'),
    'magnet': ('magnet', 'full'),
    'clock': ('clock', 'full'),
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

    print('\nRecommended gantry flow:')
    print('-' * 60)
    print('  1) gantry/limits        Confirm switch polarity and wiring')
    print('  2) gantry/pulse         Check pulse timing jitter')
    print('  3) gantry/motor_a + motor_b')
    print('  4) gantry/corexy        Verify axis direction mapping')
    print('  5) gantry/speed_sweep   Identify stall zones')
    print('  6) gantry/repeatability Run loop stress test')
    print('  7) gantry/enable_hold   Validate holding torque behavior')
    print('  8) gantry/manual        Fine-tune by keyboard control')

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

    gpio = MockGPIOInterface() if args.mock else GPIOInterface()
    display = _build_display(args.display, args.mock)
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


if __name__ == '__main__':
    sys.exit(main())
