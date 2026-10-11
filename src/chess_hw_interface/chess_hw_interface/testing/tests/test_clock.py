#!/usr/bin/env python3
"""
Clock Display Test Suite.

Tests:
1. Display connection
2. Character display
3. Scrolling text
4. Brightness control
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep


class ClockTest(HardwareTest):
    """
    Test chess clock display functionality.
    """
    
    @property
    def name(self) -> str:
        return "Clock"
    
    @property
    def description(self) -> str:
        return "Test clock display output"
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Display all characters
            TestStep(
                name="Character Test",
                display_text="CHAR TST",
                action=self._test_characters,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="CHAR OK",
                failure_message="CHAR BAD"
            ),
            
            # Confirm characters
            TestStep(
                name="Confirm Characters",
                display_text="CHARS OK",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="CONFIRMED",
                failure_message="FAILED"
            ),
            
            # Numeric display
            TestStep(
                name="Numeric Test",
                display_text="NUM TEST",
                action=self._test_numbers,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="NUM OK",
                failure_message="NUM BAD"
            ),
            
            # Time display format
            TestStep(
                name="Time Format",
                display_text="TIME FMT",
                action=self._test_time_format,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="TIME OK",
                failure_message="TIME BAD"
            ),
            
            # Scrolling text
            TestStep(
                name="Scroll Test",
                display_text="SCROLL",
                action=self._test_scroll,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="SCRL OK",
                failure_message="SCRL BAD"
            ),
        ]
    
    def _test_characters(self) -> bool:
        """Test all displayable characters."""
        print("  Testing character display...")
        
        # Test uppercase letters
        for char in "ABCDEFGH":
            self.show_display(f"   {char}    ")
            time.sleep(0.2)
        
        for char in "IJKLMNOP":
            self.show_display(f"   {char}    ")
            time.sleep(0.2)
        
        for char in "QRSTUVWX":
            self.show_display(f"   {char}    ")
            time.sleep(0.2)
        
        self.show_display("   Y Z  ")
        time.sleep(0.3)
        
        return True
    
    def _test_numbers(self) -> bool:
        """Test numeric display."""
        print("  Testing numeric display...")
        
        # Count up
        for i in range(10):
            self.show_display(f"   {i}    ")
            time.sleep(0.2)
        
        # Multi-digit
        for num in [12, 345, 6789, 1234]:
            self.show_display(f"{num:>8}")
            time.sleep(0.3)
        
        return True
    
    def _test_time_format(self) -> bool:
        """Test time display format like a chess clock."""
        print("  Testing time format (MM:SS)...")
        
        # Simulate countdown
        times = ["10:00", " 9:59", " 5:30", " 1:00", " 0:30", " 0:10", " 0:05"]
        
        for t in times:
            # Format as two segments: left and right
            self.show_display(t.center(8))
            time.sleep(0.4)
        
        # Blink effect for low time
        for _ in range(4):
            self.show_display(" 0:05   ")
            time.sleep(0.2)
            self.show_display("        ")
            time.sleep(0.2)
        
        return True
    
    def _test_scroll(self) -> bool:
        """Test scrolling text."""
        print("  Testing scrolling text...")
        
        message = "  SMART CHESS BOARD HARDWARE TEST  "
        
        for i in range(len(message) - 7):
            self.show_display(message[i:i+8])
            time.sleep(0.15)
        
        return True
