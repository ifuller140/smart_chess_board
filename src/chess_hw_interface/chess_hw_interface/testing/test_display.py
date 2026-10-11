#!/usr/bin/env python3
"""
TM1637 Display Driver for Chess Clock Displays.

The TM1637 is a 4-digit 7-segment display controller that uses a proprietary
two-wire protocol (CLK + DIO). This is NOT I2C despite the similar pin names.

Per pinout.md ground truth diagram:
- Clock 1: CLK=GPIO25, DIO=GPIO8
- Clock 2: CLK=GPIO7, DIO=GPIO1
"""

import time
from abc import ABC, abstractmethod
from typing import Optional

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False


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
        """Set display brightness (0-7 for TM1637)."""
        pass


class TM1637Display(DisplayInterface):
    """
    TM1637 4-Digit 7-Segment Display Driver.
    
    Uses bit-banging protocol to communicate with TM1637 chip.
    Each display has CLK and DIO pins.
    """
    
    # Commands
    CMD_DATA = 0x40      # Data command: write data to display register
    CMD_ADDR = 0xC0      # Address command: set display address
    CMD_CTRL = 0x80      # Control command: display on/off and brightness
    
    # 7-segment encoding (active HIGH, common cathode)
    # Bit order: DP-G-F-E-D-C-B-A (bit 7 to bit 0)
    CHAR_MAP = {
        '0': 0x3F, '1': 0x06, '2': 0x5B, '3': 0x4F,
        '4': 0x66, '5': 0x6D, '6': 0x7D, '7': 0x07,
        '8': 0x7F, '9': 0x6F, 'A': 0x77, 'B': 0x7C,
        'C': 0x39, 'D': 0x5E, 'E': 0x79, 'F': 0x71,
        'G': 0x3D, 'H': 0x76, 'I': 0x06, 'J': 0x0E,
        'K': 0x75, 'L': 0x38, 'M': 0x15, 'N': 0x54,
        'O': 0x3F, 'P': 0x73, 'Q': 0x67, 'R': 0x50,
        'S': 0x6D, 'T': 0x78, 'U': 0x3E, 'V': 0x1C,
        'W': 0x2A, 'X': 0x76, 'Y': 0x6E, 'Z': 0x5B,
        ' ': 0x00, '-': 0x40, '_': 0x08, ':': 0x80,
    }
    
    # Bit timing (microseconds)
    BIT_DELAY_US = 10  # Delay between bit transitions
    
    def __init__(self, clk_pin: int, dio_pin: int, brightness: int = 7):
        """
        Initialize TM1637 display.
        
        Args:
            clk_pin: GPIO pin (BCM) for CLK
            dio_pin: GPIO pin (BCM) for DIO
            brightness: Initial brightness 0-7 (default: 7 = max)
        """
        self.clk_pin = clk_pin
        self.dio_pin = dio_pin
        self._brightness = min(7, max(0, brightness))
        self._display_on = True
        self._colon = False
        
        if not GPIO_AVAILABLE:
            raise ImportError("RPi.GPIO is required for TM1637 display")
        
        # Setup GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        GPIO.setup(self.clk_pin, GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(self.dio_pin, GPIO.OUT, initial=GPIO.HIGH)
        
        # Initialize display
        self.clear()
        self._update_brightness()
    
    def _delay(self):
        """Bit delay for protocol timing."""
        time.sleep(self.BIT_DELAY_US / 1_000_000)
    
    def _start(self):
        """Send start signal: DIO goes LOW while CLK is HIGH."""
        GPIO.output(self.clk_pin, GPIO.HIGH)
        GPIO.output(self.dio_pin, GPIO.HIGH)
        self._delay()
        GPIO.output(self.dio_pin, GPIO.LOW)
        self._delay()
        GPIO.output(self.clk_pin, GPIO.LOW)
        self._delay()
    
    def _stop(self):
        """Send stop signal: DIO goes HIGH while CLK is HIGH."""
        GPIO.output(self.clk_pin, GPIO.LOW)
        GPIO.output(self.dio_pin, GPIO.LOW)
        self._delay()
        GPIO.output(self.clk_pin, GPIO.HIGH)
        self._delay()
        GPIO.output(self.dio_pin, GPIO.HIGH)
        self._delay()
    
    def _write_byte(self, data: int) -> bool:
        """
        Write a byte to the TM1637.
        
        Args:
            data: Byte to write
            
        Returns:
            True if ACK received, False otherwise
        """
        # Send 8 bits, LSB first
        for i in range(8):
            GPIO.output(self.clk_pin, GPIO.LOW)
            self._delay()
            
            # Set data bit
            if data & (1 << i):
                GPIO.output(self.dio_pin, GPIO.HIGH)
            else:
                GPIO.output(self.dio_pin, GPIO.LOW)
            self._delay()
            
            GPIO.output(self.clk_pin, GPIO.HIGH)
            self._delay()
        
        # Wait for ACK
        GPIO.output(self.clk_pin, GPIO.LOW)
        GPIO.output(self.dio_pin, GPIO.HIGH)
        self._delay()
        
        # Read ACK (DIO should go LOW)
        GPIO.setup(self.dio_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        self._delay()
        GPIO.output(self.clk_pin, GPIO.HIGH)
        self._delay()
        
        ack = GPIO.input(self.dio_pin) == GPIO.LOW
        
        # Return DIO to output mode
        GPIO.output(self.clk_pin, GPIO.LOW)
        GPIO.setup(self.dio_pin, GPIO.OUT)
        GPIO.output(self.dio_pin, GPIO.LOW)
        self._delay()
        
        return ack
    
    def _update_brightness(self):
        """Send brightness/display control command."""
        self._start()
        if self._display_on:
            self._write_byte(self.CMD_CTRL | 0x08 | self._brightness)
        else:
            self._write_byte(self.CMD_CTRL)  # Display off
        self._stop()
    
    def show_digits(self, digits: list, colon: bool = False):
        """
        Display raw segment data.
        
        Args:
            digits: List of 4 segment bytes
            colon: Whether to show colon (between digits 1 and 2)
        """
        # Write data command (auto-increment address)
        self._start()
        self._write_byte(self.CMD_DATA)
        self._stop()
        
        # Write address and data
        self._start()
        self._write_byte(self.CMD_ADDR)  # Start at address 0
        
        for i, digit in enumerate(digits[:4]):
            # Add colon to second digit if requested
            if i == 1 and colon:
                digit |= 0x80
            self._write_byte(digit)
        
        self._stop()
        
        # Update display control
        self._update_brightness()
    
    def show_text(self, text: str):
        """
        Display text on the 4-digit display.
        
        Args:
            text: Text to display (max 4 characters)
        """
        text = text.upper()[:4].ljust(4)
        
        digits = []
        for char in text:
            digits.append(self.CHAR_MAP.get(char, 0x00))
        
        self.show_digits(digits, colon=self._colon)
    
    def show_number(self, number: int, leading_zeros: bool = False):
        """
        Display a number (0-9999).
        
        Args:
            number: Number to display
            leading_zeros: Whether to show leading zeros
        """
        number = max(0, min(9999, number))
        
        if leading_zeros:
            text = f"{number:04d}"
        else:
            text = f"{number:4d}"
        
        self.show_text(text)
    
    def show_time(self, minutes: int, seconds: int, colon: bool = True):
        """
        Display time in MM:SS format.
        
        Args:
            minutes: Minutes (0-99)
            seconds: Seconds (0-59)
            colon: Whether to show colon
        """
        minutes = max(0, min(99, minutes))
        seconds = max(0, min(59, seconds))
        
        self._colon = colon
        text = f"{minutes:02d}{seconds:02d}"
        self.show_text(text)
        self._colon = False
    
    def clear(self):
        """Clear the display."""
        self.show_digits([0x00, 0x00, 0x00, 0x00])
    
    def set_brightness(self, level: int):
        """
        Set display brightness.
        
        Args:
            level: Brightness level 0-7 (0=dimmest, 7=brightest)
        """
        self._brightness = min(7, max(0, level))
        self._update_brightness()
    
    def display_on(self):
        """Turn display on."""
        self._display_on = True
        self._update_brightness()
    
    def display_off(self):
        """Turn display off (preserves digit data)."""
        self._display_on = False
        self._update_brightness()
    
    def cleanup(self):
        """Cleanup GPIO on exit."""
        self.clear()
        # Don't call GPIO.cleanup() here - let the main program handle it


class DualTM1637Display(DisplayInterface):
    """
    Controls two TM1637 displays as a single 8-character display.
    
    This combines two 4-digit displays for chess clock testing,
    treating them as left (display 1) and right (display 2) halves.
    """
    
    def __init__(self, display1: TM1637Display, display2: TM1637Display):
        """
        Initialize dual display.
        
        Args:
            display1: Left display (first 4 characters)
            display2: Right display (last 4 characters)
        """
        self.display1 = display1
        self.display2 = display2
    
    def show_text(self, text: str):
        """
        Display text across both displays (8 characters total).
        
        Args:
            text: Text to display (max 8 characters)
        """
        text = text.ljust(8)[:8]
        self.display1.show_text(text[:4])
        self.display2.show_text(text[4:])
    
    def clear(self):
        """Clear both displays."""
        self.display1.clear()
        self.display2.clear()
    
    def set_brightness(self, level: int):
        """Set brightness on both displays."""
        self.display1.set_brightness(level)
        self.display2.set_brightness(level)
    
    def cleanup(self):
        """Cleanup both displays."""
        self.display1.cleanup()
        self.display2.cleanup()


def create_display(display_type: str = "tm1637", **kwargs) -> DisplayInterface:
    """
    Factory function to create appropriate display interface.
    
    Args:
        display_type: "tm1637" or "dual_tm1637"
        **kwargs: Additional arguments:
            For tm1637: clk_pin, dio_pin, brightness
            For dual_tm1637: clk1_pin, dio1_pin, clk2_pin, dio2_pin, brightness
        
    Returns:
        DisplayInterface instance
        
    Raises:
        ValueError: If display_type is not recognized
    """
    if display_type == "tm1637":
        if PIGPIO_AVAILABLE:
            try:
                return PigpioTM1637Display(
                    clk_pin=kwargs.get("clk_pin", 25),
                    dio_pin=kwargs.get("dio_pin", 8),
                    brightness=kwargs.get("brightness", 7)
                )
            except (ImportError, RuntimeError):
                pass # Fallback to RPi.GPIO

        return TM1637Display(
            clk_pin=kwargs.get("clk_pin", 25),  # Default from pinout.md
            dio_pin=kwargs.get("dio_pin", 8),
            brightness=kwargs.get("brightness", 7)
        )
    elif display_type == "dual_tm1637":
        clk1 = kwargs.get("clk1_pin", 25)
        dio1 = kwargs.get("dio1_pin", 8)
        clk2 = kwargs.get("clk2_pin", 7)
        dio2 = kwargs.get("dio2_pin", 1)
        br = kwargs.get("brightness", 7)

        if PIGPIO_AVAILABLE:
            try:
                d1 = PigpioTM1637Display(clk_pin=clk1, dio_pin=dio1, brightness=br)
                d2 = PigpioTM1637Display(clk_pin=clk2, dio_pin=dio2, brightness=br)
                return DualTM1637Display(d1, d2)
            except (ImportError, RuntimeError):
                pass
        
        display1 = TM1637Display(clk_pin=clk1, dio_pin=dio1, brightness=br)
        display2 = TM1637Display(clk_pin=clk2, dio_pin=dio2, brightness=br)
        return DualTM1637Display(display1, display2)
    else:
        raise ValueError(f"Unknown display type: {display_type}. Use 'tm1637' or 'dual_tm1637'.")


# Quick test
if __name__ == "__main__":
    print("TM1637 Display Test")
    print("=" * 40)
    
    # Use default pins from pinout.md
    try:
        display = create_display("tm1637", clk_pin=25, dio_pin=8)
        
        print("Testing text display...")
        display.show_text("TEST")
        time.sleep(1)
        
        print("Testing numbers...")
        for i in range(10):
            display.show_number(i * 111)
            time.sleep(0.3)
        
        print("Testing time display...")
        for i in range(5):
            display.show_time(5, 59 - i, colon=True)
            time.sleep(0.5)
            display.show_time(5, 59 - i, colon=False)
            time.sleep(0.5)
        
        print("Testing brightness...")
        for b in range(8):
            display.set_brightness(b)
            display.show_text(f"BR {b}")
            time.sleep(0.3)
        
        display.set_brightness(7)
        display.show_text("DONE")
        time.sleep(1)
        
        display.clear()
        print("Test complete!")
        
    except ImportError as e:
        print(f"Error: {e}")
        print("This must be run on a Raspberry Pi with GPIO.")
    except Exception as e:
        print(f"Error: {e}")


try:
    import pigpio
    PIGPIO_AVAILABLE = True
except ImportError:
    PIGPIO_AVAILABLE = False


class PigpioTM1637Display(DisplayInterface):
    """
    TM1637 Display Driver using pigpio (daemon-based).
    Allows non-root access to GPIO.
    """
    
    # Commands and Char Map match TM1637Display
    CMD_DATA = 0x40
    CMD_ADDR = 0xC0
    CMD_CTRL = 0x80
    CHAR_MAP = TM1637Display.CHAR_MAP
    
    # Bit timing (microseconds) — slightly relaxed for socket overhead
    BIT_DELAY_US = 20
    
    def __init__(self, clk_pin: int, dio_pin: int, brightness: int = 7):
        if not PIGPIO_AVAILABLE:
            raise ImportError("pigpio module is required for PigpioTM1637Display")
            
        self.clk_pin = clk_pin
        self.dio_pin = dio_pin
        self._brightness = min(7, max(0, brightness))
        self._display_on = True
        self._colon = False
        
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError("Cannot connect to pigpiod daemon")
            
        # Init pins (High Impedance / Input with Pull Up is default, but we drive them)
        # TM1637 uses open-drain-like signaling (Drive Low, Release High)
        # But standard push-pull works if we are careful. 
        # RPi.GPIO driver uses push-pull. We will match it.
        self.pi.set_mode(self.clk_pin, pigpio.OUTPUT)
        self.pi.set_mode(self.dio_pin, pigpio.OUTPUT)
        
        # Initial state HIGH
        self.pi.write(self.clk_pin, 1)
        self.pi.write(self.dio_pin, 1)
        
        self.clear()
        self._update_brightness()

    def _delay(self):
        # Precise delay using pigpio is hard over socket for small values.
        # time.sleep() in python is ~100us minimum usually.
        # We'll rely on python overhead being enough or use explicit sleep.
        time.sleep(self.BIT_DELAY_US / 1_000_000.0)

    def _start(self):
        self.pi.write(self.clk_pin, 1)
        self.pi.write(self.dio_pin, 1)
        self._delay()
        self.pi.write(self.dio_pin, 0)
        self._delay()
        self.pi.write(self.clk_pin, 0)
        self._delay()

    def _stop(self):
        self.pi.write(self.clk_pin, 0)
        self.pi.write(self.dio_pin, 0)
        self._delay()
        self.pi.write(self.clk_pin, 1)
        self._delay()
        self.pi.write(self.dio_pin, 1)
        self._delay()

    def _write_byte(self, data: int) -> bool:
        for i in range(8):
            self.pi.write(self.clk_pin, 0)
            self._delay()
            
            val = 1 if (data & (1 << i)) else 0
            self.pi.write(self.dio_pin, val)
            self._delay()
            
            self.pi.write(self.clk_pin, 1)
            self._delay()
            
        # ACK check
        self.pi.write(self.clk_pin, 0)
        self.pi.write(self.dio_pin, 1) # Release DATA
        self.pi.set_mode(self.dio_pin, pigpio.INPUT) # Switch to input
        self._delay()
        
        self.pi.write(self.clk_pin, 1)
        self._delay()
        
        ack = self.pi.read(self.dio_pin) == 0
        
        self.pi.write(self.clk_pin, 0)
        self.pi.set_mode(self.dio_pin, pigpio.OUTPUT) # Back to output
        self.pi.write(self.dio_pin, 0)
        self._delay()
        
        return ack
        
    def _update_brightness(self):
        self._start()
        cmd = self.CMD_CTRL | (0x08 if self._display_on else 0) | self._brightness
        self._write_byte(cmd)
        self._stop()
        
    def show_digits(self, digits: list, colon: bool = False):
        self._start()
        self._write_byte(self.CMD_DATA)
        self._stop()
        
        self._start()
        self._write_byte(self.CMD_ADDR)
        
        for i, digit in enumerate(digits[:4]):
            if i == 1 and colon:
                digit |= 0x80
            self._write_byte(digit)
            
        self._stop()
        self._update_brightness()
        
    def show_text(self, text: str):
        text = text.upper()[:4].ljust(4)
        digits = [self.CHAR_MAP.get(c, 0x00) for c in text]
        self.show_digits(digits, colon=self._colon)

    def show_time(self, minutes: int, seconds: int, colon: bool = True):
        minutes = max(0, min(99, minutes))
        seconds = max(0, min(59, seconds))
        self._colon = colon
        text = f"{minutes:02d}{seconds:02d}"
        self.show_text(text)
        self._colon = False

    def clear(self):
        self.show_digits([0, 0, 0, 0])

    def set_brightness(self, level: int):
        self._brightness = min(7, max(0, level))
        self._update_brightness()

    def cleanup(self):
        self.clear()
        self.pi.stop()


class PigpioDualTM1637Display(DualTM1637Display):
    """Wrapper for Dual Display using Pigpio backend."""
    pass # Inherits logic, just holds Pigpio instances



