#!/usr/bin/env python3
"""
Electromagnet Test Suite.

Tests:
1. Magnet engage (on)
2. Magnet release (off)
3. Timed hold test
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep


class MagnetTest(HardwareTest):
    """
    Test electromagnet functionality.
    """
    
    # Magnet control pin (BCM)
    # USER_ATTENTION: Set this to your actual electromagnet control pin
    MAGNET_PIN = 26  # Placeholder - update in pinout.md and here
    
    @property
    def name(self) -> str:
        return "Magnet"
    
    @property
    def description(self) -> str:
        return "Test electromagnet engage and release"
    
    def setup(self) -> bool:
        """Setup magnet control pin."""
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        try:
            self.gpio.setup_output(self.MAGNET_PIN)
            self.gpio.write(self.MAGNET_PIN, False)  # Start with magnet off
            return True
        except Exception as e:
            print(f"[ERROR] Magnet setup failed: {e}")
            return False
    
    def teardown(self):
        """Ensure magnet is off."""
        if self.gpio:
            self.gpio.write(self.MAGNET_PIN, False)
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Warning
            TestStep(
                name="Safety Check",
                display_text="MAG SAFE",
                action=self._safety_warning,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="READY",
                failure_message="ABORT"
            ),
            
            # Engage test
            TestStep(
                name="Magnet Engage",
                display_text="MAG ON",
                action=self._test_engage,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="ON OK?",
                failure_message="ON FAIL"
            ),
            
            # Confirm engage
            TestStep(
                name="Confirm Engage",
                display_text="ON OK?",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="ON CONF",
                failure_message="ON BAD"
            ),
            
            # Release test
            TestStep(
                name="Magnet Release",
                display_text="MAG OFF",
                action=self._test_release,
                wait_for_input=False,
                success_message="OFF OK",
                failure_message="OFF FAIL"
            ),
            
            # Hold strength test
            TestStep(
                name="Hold Strength Test",
                display_text="HOLD TST",
                action=self._test_hold_strength,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=60.0,
                success_message="HOLD OK",
                failure_message="WEAK"
            ),
        ]
    
    def _safety_warning(self) -> bool:
        """Display safety warning."""
        print("\n  ⚠️  ELECTROMAGNET TEST")
        print("  ----------------------")
        print("  - Keep hands clear of magnet")
        print("  - Remove sensitive electronics from area")
        print("  - Have a steel object ready to test attraction")
        print("\n  Press clock button when ready...\n")
        return True
    
    def _set_magnet(self, state: bool):
        """Set magnet state."""
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        self.gpio.write(self.MAGNET_PIN, state)
    
    def _test_engage(self) -> bool:
        """Test magnet engagement."""
        print("  Engaging electromagnet...")
        self._set_magnet(True)
        time.sleep(0.5)
        print("  Magnet is ON - try placing steel object near it")
        return True
    
    def _test_release(self) -> bool:
        """Test magnet release."""
        print("  Releasing electromagnet...")
        self._set_magnet(False)
        time.sleep(0.3)
        print("  Magnet is OFF")
        return True
    
    def _test_hold_strength(self) -> bool:
        """Test magnet holding strength."""
        print("\n  HOLD STRENGTH TEST")
        print("  ------------------")
        print("  1. Place a chess piece with steel base on the magnet")
        print("  2. Magnet will engage for 5 seconds")
        print("  3. Verify piece stays attached when lifted")
        print("\n  Press clock button to start test...\n")
        
        # Wait handled by test framework
        
        print("  Engaging magnet for 5 seconds...")
        self._set_magnet(True)
        
        for i in range(5, 0, -1):
            self.show_display(f"HOLD  {i}")
            time.sleep(1)
        
        print("  Releasing magnet...")
        self._set_magnet(False)
        
        return True
