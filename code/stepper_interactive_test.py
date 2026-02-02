#!/usr/bin/env python3
"""
Interactive Stepper Motor Test for A4988 Drivers + NEMA 11 Motors.

Features:
- Keyboard controls: +/- or arrow keys for speed, d for direction, q to quit
- Speed display on TM1637 clock (1-10 scale)
- Clock hit button pauses/resumes
- Limit switch safety (auto-stop if any limit hit)
- Proper motor stop on exit

DEBUGGING NOTES for jittery motor behavior:
1. A4988 requires stable DIR pin BEFORE stepping (setup time)
2. STEP pulse must be held HIGH long enough (min 1µs, using 10µs+)
3. Step rate may be too fast - NEMA 11 needs acceleration ramp
4. Current limit on A4988 must be set correctly via potentiometer
"""

import RPi.GPIO as GPIO
import time
import sys
import signal
import threading
import atexit

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# Stepper Motors (A4988 Drivers + NEMA 11)
MOTOR_A_DIR_PIN = 27    # Direction pin for Motor A
MOTOR_A_STEP_PIN = 22   # Step pin for Motor A
MOTOR_B_DIR_PIN = 6     # Direction pin for Motor B
MOTOR_B_STEP_PIN = 5    # Step pin for Motor B

# Limit Switches (Active HIGH: 1=Pressed)
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_PIN = 15

# Clock Displays (TM1637)
CLOCK_1 = {'clk': 25, 'dio': 8}
CLOCK_2 = {'clk': 7, 'dio': 1}

# ==========================
# TIMING CONSTANTS
# ==========================
# A4988 Timing Requirements:
# - DIR setup time: 200ns minimum (we use 10µs for safety)
# - STEP pulse width: 1µs minimum (we use 50µs for visibility)
# - STEP low time: 1µs minimum (we use step_delay)

DIR_SETUP_TIME_US = 10          # Microseconds to wait after setting DIR
STEP_PULSE_WIDTH_US = 50        # Microseconds to hold STEP high (increased for stability)

# Speed levels (1-10) mapped to step delays in MILLISECONDS
# Slower = higher delay, Faster = lower delay
# NEMA 11 can handle fast speeds, but start slow for debugging
SPEED_DELAYS_MS = {
    1: 20.0,    # Very slow - 50 steps/sec
    2: 15.0,    # Slow - 67 steps/sec
    3: 10.0,    # Moderate - 100 steps/sec
    4: 7.0,     # Medium - 143 steps/sec
    5: 5.0,     # Medium-fast - 200 steps/sec
    6: 3.0,     # Fast - 333 steps/sec
    7: 2.0,     # Faster - 500 steps/sec
    8: 1.5,     # Very fast - 667 steps/sec
    9: 1.0,     # Near max - 1000 steps/sec
    10: 0.5,    # Maximum - 2000 steps/sec
}

# ==========================
# GLOBAL STATE
# ==========================
running = True
motor_running = True
paused = False
current_speed = 1       # 1-10 scale
direction = 1           # 1 = forward, -1 = reverse
motors_enabled = False


# ==========================
# TM1637 DISPLAY DRIVER
# ==========================
class TM1637:
    """Simple TM1637 7-segment display driver."""
    
    # Segment patterns for digits 0-9
    DIGITS = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F]
    
    def __init__(self, clk, dio):
        self.clk = clk
        self.dio = dio
        GPIO.setup(self.clk, GPIO.OUT)
        GPIO.setup(self.dio, GPIO.OUT)
        
    def _start(self):
        GPIO.output(self.dio, 1)
        GPIO.output(self.clk, 1)
        GPIO.output(self.dio, 0)
        GPIO.output(self.clk, 0)

    def _stop(self):
        GPIO.output(self.clk, 0)
        GPIO.output(self.dio, 0)
        GPIO.output(self.clk, 1)
        GPIO.output(self.dio, 1)

    def _write_byte(self, byte):
        for i in range(8):
            GPIO.output(self.clk, 0)
            GPIO.output(self.dio, (byte >> i) & 1)
            GPIO.output(self.clk, 1)
        # ACK
        GPIO.output(self.clk, 0)
        GPIO.setup(self.dio, GPIO.IN)
        GPIO.output(self.clk, 1)
        GPIO.setup(self.dio, GPIO.OUT)

    def show_number(self, num, brightness=7):
        """Display a number (0-9999) on the display."""
        # Extract digits
        d1 = (num // 1000) % 10
        d2 = (num // 100) % 10
        d3 = (num // 10) % 10
        d4 = num % 10
        
        self._start()
        self._write_byte(0x40)  # Data command: write data
        self._stop()
        
        self._start()
        self._write_byte(0xC0)  # Address command: start at position 0
        
        # For speed display, show "SP X" where X is speed
        # Position 0: S (0x6D)
        # Position 1: P (0x73)
        # Position 2: space (0x00)
        # Position 3: digit
        if num <= 10:
            self._write_byte(0x6D)  # S
            self._write_byte(0x73)  # P
            self._write_byte(0x00)  # space
            self._write_byte(self.DIGITS[num % 10] if num < 10 else self.DIGITS[0])  # digit (10 shows as 0)
        else:
            self._write_byte(self.DIGITS[d1] if d1 > 0 else 0x00)
            self._write_byte(self.DIGITS[d2] if d1 > 0 or d2 > 0 else 0x00)
            self._write_byte(self.DIGITS[d3] if d1 > 0 or d2 > 0 or d3 > 0 else 0x00)
            self._write_byte(self.DIGITS[d4])
        self._stop()
        
        self._start()
        self._write_byte(0x88 | (brightness & 0x07))  # Display control
        self._stop()
    
    def show_text(self, text):
        """Display simple text (limited characters)."""
        # Character map for common letters
        chars = {
            ' ': 0x00, '-': 0x40, '_': 0x08,
            '0': 0x3F, '1': 0x06, '2': 0x5B, '3': 0x4F, '4': 0x66,
            '5': 0x6D, '6': 0x7D, '7': 0x07, '8': 0x7F, '9': 0x6F,
            'A': 0x77, 'b': 0x7C, 'C': 0x39, 'd': 0x5E, 'E': 0x79,
            'F': 0x71, 'G': 0x3D, 'H': 0x76, 'I': 0x06, 'J': 0x1E,
            'L': 0x38, 'n': 0x54, 'O': 0x3F, 'P': 0x73, 'r': 0x50,
            'S': 0x6D, 't': 0x78, 'U': 0x3E, 'Y': 0x6E,
        }
        
        self._start()
        self._write_byte(0x40)
        self._stop()
        
        self._start()
        self._write_byte(0xC0)
        for i in range(4):
            char = text[i].upper() if i < len(text) else ' '
            self._write_byte(chars.get(char, 0x00))
        self._stop()
        
        self._start()
        self._write_byte(0x8F)  # Max brightness
        self._stop()
    
    def clear(self):
        """Clear the display."""
        self._start()
        self._write_byte(0x40)
        self._stop()
        self._start()
        self._write_byte(0xC0)
        for _ in range(4):
            self._write_byte(0x00)
        self._stop()
        self._start()
        self._write_byte(0x80)  # Display off
        self._stop()


# ==========================
# MOTOR CONTROL
# ==========================
def stop_motors():
    """Immediately stop all motors and set pins low."""
    global motors_enabled
    motors_enabled = False
    GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)
    GPIO.output(MOTOR_A_DIR_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_DIR_PIN, GPIO.LOW)


def step_both_motors_once():
    """
    Execute a single synchronized step on both motors.
    
    A4988 Timing:
    1. DIR must be stable BEFORE stepping
    2. STEP pulse: LOW -> HIGH (hold) -> LOW
    3. Wait step_delay before next step
    """
    if not motors_enabled:
        return
    
    # Generate STEP pulse on both motors simultaneously
    GPIO.output(MOTOR_A_STEP_PIN, GPIO.HIGH)
    GPIO.output(MOTOR_B_STEP_PIN, GPIO.HIGH)
    
    # Hold pulse high (minimum 1µs, we use more for stability)
    time.sleep(STEP_PULSE_WIDTH_US / 1_000_000)
    
    GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
    GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)


def set_direction(dir_value):
    """
    Set motor direction with proper setup time.
    
    dir_value: 1 = forward, -1 = reverse
    """
    global direction
    direction = dir_value
    
    # Set DIR pins
    dir_state = GPIO.HIGH if direction > 0 else GPIO.LOW
    GPIO.output(MOTOR_A_DIR_PIN, dir_state)
    GPIO.output(MOTOR_B_DIR_PIN, dir_state)
    
    # Wait for DIR setup time (A4988 requires DIR stable before STEP)
    time.sleep(DIR_SETUP_TIME_US / 1_000_000)


def motor_loop():
    """Main motor stepping loop - runs in separate thread."""
    global running, paused, motors_enabled, current_speed
    
    while running:
        # Check if paused
        if paused or not motors_enabled:
            time.sleep(0.01)
            continue
        
        # Check limit switches
        if check_limits():
            print("\n⚠️  LIMIT SWITCH TRIGGERED - STOPPING MOTORS!")
            stop_motors()
            break
        
        # Execute one step
        step_both_motors_once()
        
        # Wait based on current speed
        delay_ms = SPEED_DELAYS_MS.get(current_speed, 10.0)
        time.sleep(delay_ms / 1000.0)
    
    # Ensure motors are stopped when loop exits
    stop_motors()


# ==========================
# SAFETY
# ==========================
def check_limits():
    """Check all limit switches. Returns True if any triggered."""
    x = GPIO.input(LIMIT_X_PIN)
    y = GPIO.input(LIMIT_Y_PIN)
    c = GPIO.input(LIMIT_CLOCK_PIN)
    # Active HIGH: 1 = pressed
    return x == 1 or y == 1


def emergency_stop(signum=None, frame=None):
    """Emergency stop handler for signals."""
    global running, motor_running
    print("\n🛑 EMERGENCY STOP - Shutting down motors...")
    running = False
    motor_running = False
    stop_motors()
    cleanup()
    sys.exit(0)


def cleanup():
    """Clean up GPIO and displays."""
    global running
    running = False
    stop_motors()
    try:
        # Clear displays
        display1.clear()
        display2.clear()
    except:
        pass
    time.sleep(0.1)  # Give motors time to stop
    GPIO.cleanup()


# ==========================
# INPUT HANDLING
# ==========================
def get_key():
    """Get a single keypress without requiring Enter."""
    import termios
    import tty
    
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        # Check for arrow keys (escape sequences)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A':  # Up arrow
                    return 'UP'
                elif ch3 == 'B':  # Down arrow
                    return 'DOWN'
                elif ch3 == 'C':  # Right arrow
                    return 'RIGHT'
                elif ch3 == 'D':  # Left arrow
                    return 'LEFT'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def handle_clock_button():
    """Monitor clock button for pause/resume in background thread."""
    global paused, running
    last_state = 0
    
    while running:
        current_state = GPIO.input(LIMIT_CLOCK_PIN)
        
        # Detect rising edge (button press)
        if current_state == 1 and last_state == 0:
            paused = not paused
            if paused:
                print("\n⏸️  PAUSED - Press clock button to resume")
                display1.show_text("PAUS")
            else:
                print("\n▶️  RESUMED")
                update_display()
            time.sleep(0.2)  # Debounce
        
        last_state = current_state
        time.sleep(0.05)


def update_display():
    """Update the clock display with current speed."""
    global current_speed, direction
    # Show speed on display 1
    display1.show_number(current_speed)
    # Show direction on display 2 (F = Forward, r = Reverse)
    if direction > 0:
        display2.show_text("FWD ")
    else:
        display2.show_text("rEU ")


# ==========================
# SETUP
# ==========================
def setup():
    """Initialize GPIO and displays."""
    global display1, display2, motors_enabled
    
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor pins
    for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    # Limit switches (Active HIGH)
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_PIN]:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Initialize displays
    display1 = TM1637(CLOCK_1['clk'], CLOCK_1['dio'])
    display2 = TM1637(CLOCK_2['clk'], CLOCK_2['dio'])
    
    # Set initial direction
    set_direction(1)
    
    # Enable motors
    motors_enabled = True
    
    # Register cleanup handlers
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)


# ==========================
# MAIN
# ==========================
def main():
    global running, paused, current_speed, direction, motors_enabled
    
    setup()
    
    print("╔════════════════════════════════════════════════════════════╗")
    print("║       INTERACTIVE STEPPER MOTOR TEST                       ║")
    print("║       (A4988 + NEMA 11 Motors)                             ║")
    print("╠════════════════════════════════════════════════════════════╣")
    print("║  CONTROLS:                                                  ║")
    print("║    ↑/+ : Increase speed                                     ║")
    print("║    ↓/- : Decrease speed                                     ║")
    print("║    d   : Toggle direction (Forward/Reverse)                 ║")
    print("║    p   : Pause/Resume                                       ║")
    print("║    Clock button : Pause/Resume                              ║")
    print("║    q   : Quit                                               ║")
    print("║                                                             ║")
    print("║  SAFETY:                                                    ║")
    print("║    - Any limit switch hit = EMERGENCY STOP                  ║")
    print("║    - Ctrl+C = EMERGENCY STOP                                ║")
    print("╠════════════════════════════════════════════════════════════╣")
    print("║  Speed: 1 (slowest) to 10 (fastest)                        ║")
    print("╚════════════════════════════════════════════════════════════╝")
    print()
    
    # Check if any limit is already triggered
    if check_limits():
        print("⚠️  WARNING: A limit switch is currently triggered!")
        print("   Release the limit switch before starting.")
        print()
    
    # Show initial state
    update_display()
    print(f"Starting at Speed: {current_speed}, Direction: {'Forward' if direction > 0 else 'Reverse'}")
    print("Press any control key to begin...\n")
    
    # Start motor thread
    motor_thread = threading.Thread(target=motor_loop, daemon=True)
    motor_thread.start()
    
    # Start clock button monitor thread
    clock_thread = threading.Thread(target=handle_clock_button, daemon=True)
    clock_thread.start()
    
    try:
        while running:
            key = get_key()
            
            if key.lower() == 'q':
                print("\n👋 Quitting...")
                running = False
                break
            
            elif key in ['UP', '+', '=']:
                if current_speed < 10:
                    current_speed += 1
                    print(f"Speed: {current_speed}")
                    update_display()
            
            elif key in ['DOWN', '-', '_']:
                if current_speed > 1:
                    current_speed -= 1
                    print(f"Speed: {current_speed}")
                    update_display()
            
            elif key.lower() == 'd':
                new_dir = -direction
                set_direction(new_dir)
                print(f"Direction: {'Forward' if direction > 0 else 'Reverse'}")
                update_display()
            
            elif key.lower() == 'p':
                paused = not paused
                if paused:
                    print("⏸️  PAUSED")
                    display1.show_text("PAUS")
                else:
                    print("▶️  RESUMED")
                    update_display()
            
            elif key in '1234567890':
                new_speed = int(key)
                if new_speed == 0:
                    new_speed = 10
                current_speed = new_speed
                print(f"Speed: {current_speed}")
                update_display()
    
    except Exception as e:
        print(f"\n❌ Error: {e}")
    
    finally:
        print("\n🛑 Stopping motors...")
        running = False
        stop_motors()
        time.sleep(0.2)  # Wait for motor thread to finish
        cleanup()
        print("✅ Motors stopped. Goodbye!")


if __name__ == "__main__":
    main()
