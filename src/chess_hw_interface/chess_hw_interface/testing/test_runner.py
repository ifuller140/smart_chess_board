#!/usr/bin/env python3
"""
Hardware Test Runner CLI.

Run hardware tests from the command line:
    python3 -m chess_hw_interface.testing.test_runner --all
    python3 -m chess_hw_interface.testing.test_runner --test gantry
    python3 -m chess_hw_interface.testing.test_runner --list
"""

import argparse
import sys
from typing import Dict, Type, Optional

from .base_test import HardwareTest, TestResult
from .test_display import create_display, DisplayInterface

# Import test modules
from .tests.test_gantry import GantryTest
from .tests.test_servo import ServoTest
from .tests.test_camera import CameraTest
from .tests.test_magnet import MagnetTest
from .tests.test_clock import ClockTest


# Registry of available tests
TEST_REGISTRY: Dict[str, Type[HardwareTest]] = {
    "gantry": GantryTest,
    "servo": ServoTest,
    "camera": CameraTest,
    "magnet": MagnetTest,
    "clock": ClockTest,
}


class GPIOInterface:
    """
    GPIO interface for hardware tests.
    
    Wraps RPi.GPIO for consistent access.
    """
    
    def __init__(self, mock: bool = False):
        """
        Initialize GPIO interface.
        
        Args:
            mock: If True, don't use real GPIO (for testing on non-Pi)
        """
        self.mock = mock
        self._pin_states = {}
        
        if not mock:
            try:
                import RPi.GPIO as GPIO
                self.GPIO = GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setwarnings(False)
            except ImportError:
                print("[WARNING] RPi.GPIO not available, using mock mode")
                self.mock = True
                self.GPIO = None
        else:
            self.GPIO = None
    
    def setup_output(self, pin: int):
        """Configure pin as output."""
        if self.mock:
            self._pin_states[pin] = False
        else:
            self.GPIO.setup(pin, self.GPIO.OUT)
    
    def setup_input(self, pin: int, pull_up: bool = True):
        """Configure pin as input with optional pull-up."""
        if self.mock:
            self._pin_states[pin] = True if pull_up else False
        else:
            pud = self.GPIO.PUD_UP if pull_up else self.GPIO.PUD_DOWN
            self.GPIO.setup(pin, self.GPIO.IN, pull_up_down=pud)
    
    def write(self, pin: int, value: bool):
        """Write to output pin."""
        if self.mock:
            self._pin_states[pin] = value
        else:
            self.GPIO.output(pin, self.GPIO.HIGH if value else self.GPIO.LOW)
    
    def read(self, pin: int) -> bool:
        """Read from input pin."""
        if self.mock:
            return self._pin_states.get(pin, False)
        else:
            return self.GPIO.input(pin) == self.GPIO.HIGH
    
    def read_x_limit(self) -> bool:
        """Read X-MIN limit switch (active LOW)."""
        # BCM pin 6 - returns True when pressed (inverted because pull-up)
        return not self.read(6) if not self.mock else False
    
    def read_y_limit(self) -> bool:
        """Read Y-MIN limit switch (active LOW)."""
        # BCM pin 13
        return not self.read(13) if not self.mock else False
    
    def read_clock_button(self) -> bool:
        """Read clock hit button (active LOW)."""
        # BCM pin 19
        return not self.read(19) if not self.mock else False
    
    def cleanup(self):
        """Cleanup GPIO on exit."""
        if not self.mock and self.GPIO:
            self.GPIO.cleanup()


def run_test(
    test_name: str,
    gpio: Optional[GPIOInterface] = None,
    display: Optional[DisplayInterface] = None,
    verbose: bool = True
) -> TestResult:
    """
    Run a single test by name.
    
    Args:
        test_name: Name of test to run (from registry)
        gpio: GPIO interface (creates one if not provided)
        display: Display interface (creates mock if not provided)
        verbose: Print detailed output
        
    Returns:
        Test result
    """
    if test_name not in TEST_REGISTRY:
        print(f"[ERROR] Unknown test: {test_name}")
        print(f"Available tests: {', '.join(TEST_REGISTRY.keys())}")
        return TestResult.FAILED
    
    test_class = TEST_REGISTRY[test_name]
    test = test_class(gpio_interface=gpio, display_interface=display)
    
    return test.run(verbose=verbose)


def run_all_tests(
    gpio: Optional[GPIOInterface] = None,
    display: Optional[DisplayInterface] = None,
    verbose: bool = True
) -> Dict[str, TestResult]:
    """
    Run all registered tests.
    
    Args:
        gpio: GPIO interface
        display: Display interface
        verbose: Print detailed output
        
    Returns:
        Dictionary of test names to results
    """
    results = {}
    
    print("\n" + "=" * 60)
    print("  SMART CHESS BOARD - HARDWARE TEST SUITE")
    print("=" * 60 + "\n")
    
    for test_name in TEST_REGISTRY:
        print(f"\n>>> Running {test_name} test...")
        results[test_name] = run_test(test_name, gpio, display, verbose)
    
    # Summary
    print("\n" + "=" * 60)
    print("  TEST SUMMARY")
    print("=" * 60)
    
    passed = sum(1 for r in results.values() if r == TestResult.PASSED)
    failed = sum(1 for r in results.values() if r == TestResult.FAILED)
    
    for test_name, result in results.items():
        status = "✓ PASSED" if result == TestResult.PASSED else "✗ FAILED"
        print(f"  {test_name:15} {status}")
    
    print("-" * 60)
    print(f"  Total: {passed} passed, {failed} failed")
    print("=" * 60 + "\n")
    
    return results


def list_tests():
    """Print list of available tests."""
    print("\nAvailable hardware tests:")
    print("-" * 40)
    
    for test_name, test_class in TEST_REGISTRY.items():
        # Create instance to get description
        test = test_class(gpio_interface=None, display_interface=None)
        print(f"  {test_name:15} - {test.description}")
    
    print()


def main():
    """Main entry point for CLI."""
    parser = argparse.ArgumentParser(
        description="Smart Chess Board Hardware Test Suite",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s --list              List available tests
  %(prog)s --all               Run all tests
  %(prog)s --test gantry       Run gantry test only
  %(prog)s --test servo camera Run specific tests
  %(prog)s --mock              Run in mock mode (no hardware)
        """
    )
    
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="List available tests"
    )
    
    parser.add_argument(
        "--all", "-a",
        action="store_true",
        help="Run all tests"
    )
    
    parser.add_argument(
        "--test", "-t",
        nargs="+",
        metavar="NAME",
        help="Run specific test(s)"
    )
    
    parser.add_argument(
        "--mock", "-m",
        action="store_true",
        help="Run in mock mode (no real hardware)"
    )
    
    parser.add_argument(
        "--quiet", "-q",
        action="store_true",
        help="Minimal output"
    )
    
    parser.add_argument(
        "--display",
        choices=["mock", "seven_segment", "i2c"],
        default="mock",
        help="Display type to use (default: mock)"
    )
    
    args = parser.parse_args()
    
    # Handle --list
    if args.list:
        list_tests()
        return 0
    
    # Need either --all or --test
    if not args.all and not args.test:
        parser.print_help()
        return 1
    
    # Create interfaces
    gpio = GPIOInterface(mock=args.mock)
    display = create_display(args.display)
    verbose = not args.quiet
    
    try:
        if args.all:
            results = run_all_tests(gpio, display, verbose)
            # Return 0 if all passed, 1 otherwise
            return 0 if all(r == TestResult.PASSED for r in results.values()) else 1
        else:
            # Run specific tests
            all_passed = True
            for test_name in args.test:
                result = run_test(test_name, gpio, display, verbose)
                if result != TestResult.PASSED:
                    all_passed = False
            return 0 if all_passed else 1
    finally:
        gpio.cleanup()


if __name__ == "__main__":
    sys.exit(main())
