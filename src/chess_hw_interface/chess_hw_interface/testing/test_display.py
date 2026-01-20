#!/usr/bin/env python3
"""
Clock display interface for hardware testing.

Uses both clock segments as a single display to show test status messages.
Supports various display types (7-segment, I2C, etc.) through abstraction.
"""

import time
from abc import ABC, abstractmethod
from typing import Optional


class DisplayInterface(ABC):
    """Abstract interface for clock display hardware."""
    
    @abstractmethod
    def show_text(self, text: str):
        """Display text on the clock display."""
        pass
    
    @abstractmethod
    def clear(self):
        """Clear the display."""
        pass
    
    @abstractmethod
    def set_brightness(self, level: int):
        """Set display brightness (0-100)."""
        pass


class MockDisplay(DisplayInterface):
    """Mock display for testing without hardware."""
    
    def __init__(self):
        self._current_text = ""
        self._brightness = 100
    
    def show_text(self, text: str):
        """Print text to console as mock display."""
        self._current_text = text[:8]
        # Format like a 7-segment display
        display = f"┌{'─'*10}┐"
        content = f"│ {self._current_text:^8} │"
        bottom = f"└{'─'*10}┘"
        print(f"\n{display}\n{content}\n{bottom}")
    
    def clear(self):
        """Clear mock display."""
        self._current_text = ""
        print("\n┌──────────┐\n│          │\n└──────────┘")
    
    def set_brightness(self, level: int):
        """Set mock brightness."""
        self._brightness = max(0, min(100, level))


class SevenSegmentDisplay(DisplayInterface):
    """
    7-segment display interface using GPIO.
    
    Assumes two 4-digit 7-segment displays combined as one 8-character display.
    """
    
    # 7-segment encoding for characters (active LOW)
    # Segments: a, b, c, d, e, f, g
    CHAR_MAP = {
        '0': 0b0111111, '1': 0b0000110, '2': 0b1011011, '3': 0b1001111,
        '4': 0b1100110, '5': 0b1101101, '6': 0b1111101, '7': 0b0000111,
        '8': 0b1111111, '9': 0b1101111, 'A': 0b1110111, 'B': 0b1111100,
        'C': 0b0111001, 'D': 0b1011110, 'E': 0b1111001, 'F': 0b1110001,
        'G': 0b0111101, 'H': 0b1110110, 'I': 0b0000110, 'J': 0b0001110,
        'K': 0b1110101, 'L': 0b0111000, 'M': 0b0010101, 'N': 0b1010100,
        'O': 0b0111111, 'P': 0b1110011, 'Q': 0b1100111, 'R': 0b1010000,
        'S': 0b1101101, 'T': 0b1111000, 'U': 0b0111110, 'V': 0b0011100,
        'W': 0b0101010, 'X': 0b1110110, 'Y': 0b1101110, 'Z': 0b1011011,
        ' ': 0b0000000, '-': 0b1000000, '_': 0b0001000,
    }
    
    def __init__(self, gpio_interface, segment_pins: list, digit_pins: list):
        """
        Initialize 7-segment display.
        
        Args:
            gpio_interface: GPIO control interface
            segment_pins: List of 7 GPIO pins for segments [a,b,c,d,e,f,g]
            digit_pins: List of 8 GPIO pins for digit selection
        """
        self.gpio = gpio_interface
        self.segment_pins = segment_pins
        self.digit_pins = digit_pins
        self._current_text = "        "
        self._brightness = 100
        self._running = False
        
        # Setup pins
        if self.gpio:
            for pin in segment_pins + digit_pins:
                self.gpio.setup_output(pin)
    
    def show_text(self, text: str):
        """Display text by setting internal buffer."""
        # Pad or truncate to 8 characters
        self._current_text = text.upper().ljust(8)[:8]
    
    def clear(self):
        """Clear display."""
        self._current_text = "        "
    
    def set_brightness(self, level: int):
        """Set brightness (affects duty cycle in multiplexing)."""
        self._brightness = max(0, min(100, level))
    
    def refresh(self):
        """
        Refresh display (call in a loop for multiplexing).
        Should be called frequently (~1000Hz) for stable display.
        """
        if not self.gpio:
            return
            
        for digit_idx, char in enumerate(self._current_text):
            # Turn off all digits
            for pin in self.digit_pins:
                self.gpio.write(pin, False)
            
            # Set segments for current character
            pattern = self.CHAR_MAP.get(char, 0)
            for seg_idx, pin in enumerate(self.segment_pins):
                self.gpio.write(pin, bool(pattern & (1 << seg_idx)))
            
            # Turn on current digit
            self.gpio.write(self.digit_pins[digit_idx], True)
            
            # Hold for brightness-adjusted time
            hold_time = 0.001 * (self._brightness / 100)
            time.sleep(hold_time)
            
            # Turn off digit
            self.gpio.write(self.digit_pins[digit_idx], False)


class I2CDisplay(DisplayInterface):
    """
    I2C-based display interface (e.g., HT16K33-based displays).
    
    <!-- USER_ATTENTION: Implement based on your specific I2C display chip -->
    """
    
    def __init__(self, i2c_address: int = 0x70):
        """
        Initialize I2C display.
        
        Args:
            i2c_address: I2C address of the display controller
        """
        self._address = i2c_address
        self._current_text = ""
        # TODO: Initialize I2C bus
        # import smbus2
        # self.bus = smbus2.SMBus(1)
    
    def show_text(self, text: str):
        """Display text via I2C."""
        self._current_text = text[:8]
        # TODO: Write to I2C display
        print(f"[I2C Display] {self._current_text}")
    
    def clear(self):
        """Clear I2C display."""
        self._current_text = ""
        # TODO: Send clear command
    
    def set_brightness(self, level: int):
        """Set I2C display brightness."""
        # TODO: Send brightness command
        pass


def create_display(display_type: str = "mock", **kwargs) -> DisplayInterface:
    """
    Factory function to create appropriate display interface.
    
    Args:
        display_type: "mock", "seven_segment", or "i2c"
        **kwargs: Additional arguments for specific display types
        
    Returns:
        DisplayInterface instance
    """
    if display_type == "mock":
        return MockDisplay()
    elif display_type == "seven_segment":
        return SevenSegmentDisplay(
            gpio_interface=kwargs.get("gpio"),
            segment_pins=kwargs.get("segment_pins", []),
            digit_pins=kwargs.get("digit_pins", [])
        )
    elif display_type == "i2c":
        return I2CDisplay(i2c_address=kwargs.get("i2c_address", 0x70))
    else:
        raise ValueError(f"Unknown display type: {display_type}")
