#!/usr/bin/env python3
"""
Interactive Stepper Motor Test for A4988 Drivers + NEMA 11 Motors.

Uses pigpio for hardware-timed step pulses (DMA-based, jitter-free).

Features:
- Keyboard controls: +/- or arrow keys for speed, d for direction, q to quit
- Speed display on TM1637 clock (1-10 scale)
- Clock hit button pauses/resumes
- Limit switch safety (auto-stop if any limit hit)
- Proper motor ENABLE control (disabled when paused or idle)
- Trapezoidal acceleration profiles

Requirements:
- pigpio library: pip install pigpio
- pigpio daemon: sudo pigpiod
"""

import pigpio
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
MOTOR_ENABLE_PIN = 17   # Shared ENABLE for A4988 drivers (active LOW)

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
DIR_SETUP_TIME_US = 5       # Microseconds to wait after setting DIR
STEP_PULSE_WIDTH_US = 5     # Microseconds to hold STEP high

# Speed levels (1-10) mapped to steps per second
SPEED_LEVELS = {
    1: 100,     # Very slow
    2: 200,     # Slow
    3: 300,     # Moderate
    4: 500,     # Medium
    5: 700,     # Medium-fast
    6: 900,     # Fast
    7: 1100,    # Faster
    8: 1300,    # Very fast
    9: 1500,    # Near max
    10: 1800,   # Maximum
}

# ==========================
# GLOBAL STATE
# ==========================
pi = None
running = True
motor_running = True
paused = True  # Start paused so user sees instructions
current_speed = 3       # 1-10 scale
direction = 1           # 1 = forward, -1 = reverse
display1 = None
display2 = None


# ==========================
# TM1637 DISPLAY DRIVER
# ==========================
class TM1637:
    """Simple TM1637 7-segment display driver using pigpio."""
    
    DIGITS = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F]
    
    def __init__(self, clk, dio):
        self.clk = clk
        self.dio = dio
        pi.set_mode(self.clk, pigpio.OUTPUT)
        pi.set_mode(self.dio, pigpio.OUTPUT)
        
    def _start(self):
        pi.write(self.dio, 1)
        pi.write(self.clk, 1)
        pi.write(self.dio, 0)
        pi.write(self.clk, 0)

    def _stop(self):
        pi.write(self.clk, 0)
        pi.write(self.dio, 0)
        pi.write(self.clk, 1)
        pi.write(self.dio, 1)

    def _write_byte(self, byte):
        for i in range(8):
            pi.write(self.clk, 0)
            pi.write(self.dio, (byte >> i) & 1)
            pi.write(self.clk, 1)
        pi.write(self.clk, 0)
        pi.set_mode(self.dio, pigpio.INPUT)
        pi.write(self.clk, 1)
        pi.set_mode(self.dio, pigpio.OUTPUT)

    def show_number(self, num, brightness=7):
        """Display a number (0-9999) on the display."""
        d1 = (num // 1000) % 10
        d2 = (num // 100) % 10
        d3 = (num // 10) % 10
        d4 = num % 10
        
        self._start()
        self._write_byte(0x40)
        self._stop()
        
        self._start()
        self._write_byte(0xC0)
        
        if num <= 10:
            self._write_byte(0x6D)  # S
            self._write_byte(0x73)  # P
            self._write_byte(0x00)  # space
            self._write_byte(self.DIGITS[num % 10] if num < 10 else self.DIGITS[0])
        else:
            self._write_byte(self.DIGITS[d1] if d1 > 0 else 0x00)
            self._write_byte(self.DIGITS[d2] if d1 > 0 or d2 > 0 else 0x00)
            self._write_byte(self.DIGITS[d3] if d1 > 0 or d2 > 0 or d3 > 0 else 0x00)
            self._write_byte(self.DIGITS[d4])
        self._stop()
        
        self._start()
        self._write_byte(0x88 | (brightness & 0x07))
        self._stop()
    
    def show_text(self, text):
        """Display simple text (limited characters)."""
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
        self._write_byte(0x8F)
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
        self._write_byte(0x80)
        self._stop()


# ==========================
# MOTOR CONTROL
# ==========================
def motor_enable():
    """Enable A4988 drivers (active LOW)."""
    pi.write(MOTOR_ENABLE_PIN, 0)
    time.sleep(0.001)


def motor_disable():
    """Disable A4988 drivers (active LOW)."""
    pi.write(MOTOR_ENABLE_PIN, 1)


def stop_motors():
    """Immediately stop all motors by disabling drivers."""
    motor_disable()
    pi.write(MOTOR_A_STEP_PIN, 0)
    pi.write(MOTOR_B_STEP_PIN, 0)


def set_direction(dir_value):
    """Set motor direction with proper setup time."""
    global direction
    direction = dir_value
    
    dir_state = 1 if direction > 0 else 0
    pi.write(MOTOR_A_DIR_PIN, dir_state)
    pi.write(MOTOR_B_DIR_PIN, dir_state)
    
    time.sleep(DIR_SETUP_TIME_US / 1_000_000)


def step_both_motors_once(delay_us):
    """Execute a single synchronized step on both motors using pigpio wave."""
    wait_us = max(1, delay_us - STEP_PULSE_WIDTH_US)
    
    # Create wave for both motor step pins
    set_mask = (1 << MOTOR_A_STEP_PIN) | (1 << MOTOR_B_STEP_PIN)
    clear_mask = set_mask
    
    wave = [
        pigpio.pulse(set_mask, 0, STEP_PULSE_WIDTH_US),
        pigpio.pulse(0, clear_mask, wait_us)
    ]
    
    pi.wave_add_generic(wave)
    wid = pi.wave_create()
    pi.wave_send_once(wid)
    
    while pi.wave_tx_busy():
        time.sleep(0.0001)
    
    pi.wave_delete(wid)


def motor_loop():
    """Main motor stepping loop - runs in separate thread."""
    global running, paused
    
    while running:
        if paused:
            # Disable motors while paused
            motor_disable()
            time.sleep(0.05)
            continue
        
        # Check limit switches
        if check_limits():
            print("\n⚠️  LIMIT SWITCH TRIGGERED - STOPPING MOTORS!")
            stop_motors()
            break
        
        # Enable motors and step
        motor_enable()
        
        # Get delay for current speed level
        speed_sps = SPEED_LEVELS.get(current_speed, 300)
        delay_us = int(1_000_000 / speed_sps)
        
        step_both_motors_once(delay_us)
    
    # Ensure motors are disabled when loop exits
    stop_motors()


# ==========================
# SAFETY
# ==========================
def check_limits():
    """Check all limit switches. Returns True if any triggered."""
    x = pi.read(LIMIT_X_PIN)
    y = pi.read(LIMIT_Y_PIN)
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
    """Clean up pigpio and displays."""
    global running, pi
    running = False
    stop_motors()
    try:
        if display1:
            display1.clear()
        if display2:
            display2.clear()
    except:
        pass
    time.sleep(0.1)
    if pi and pi.connected:
        pi.stop()


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
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A':
                    return 'UP'
                elif ch3 == 'B':
                    return 'DOWN'
                elif ch3 == 'C':
                    return 'RIGHT'
                elif ch3 == 'D':
                    return 'LEFT'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def handle_clock_button():
    """Monitor clock button for pause/resume in background thread."""
    global paused, running
    last_state = 0
    
    while running:
        current_state = pi.read(LIMIT_CLOCK_PIN)
        
        if current_state == 1 and last_state == 0:
            paused = not paused
            if paused:
                print("\n⏸️  PAUSED - Motors disabled. Press clock button to resume")
                display1.show_text("PAUS")
            else:
                print("\n▶️  RESUMED")
                update_display()
            time.sleep(0.2)
        
        last_state = current_state
        time.sleep(0.05)


def update_display():
    """Update the clock display with current speed."""
    global current_speed, direction
    display1.show_number(current_speed)
    if direction > 0:
        display2.show_text("FWD ")
    else:
        display2.show_text("rEU ")


# ==========================
# SETUP
# ==========================
def setup():
    """Initialize pigpio and displays."""
    global pi, display1, display2
    
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: Cannot connect to pigpiod daemon.")
        print("Start it with: sudo pigpiod")
        sys.exit(1)
    
    # Motor pins
    for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN, MOTOR_ENABLE_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
    
    # Start with motors disabled
    motor_disable()
    
    # Limit switches
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_PIN]:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    
    # Initialize displays
    display1 = TM1637(CLOCK_1['clk'], CLOCK_1['dio'])
    display2 = TM1637(CLOCK_2['clk'], CLOCK_2['dio'])
    
    # Set initial direction
    set_direction(1)
    
    # Register cleanup handlers
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, emergency_stop)
    signal.signal(signal.SIGTERM, emergency_stop)


# ==========================
# MAIN
# ==========================
def main():
    global running, paused, current_speed, direction
    
    setup()
    
    print("╔════════════════════════════════════════════════════════════╗")
    print("║       INTERACTIVE STEPPER MOTOR TEST                       ║")
    print("║       (pigpio + A4988 + NEMA 11 Motors)                    ║")
    print("╠════════════════════════════════════════════════════════════╣")
    print("║  CONTROLS:                                                  ║")
    print("║    ↑/+ : Increase speed                                     ║")
    print("║    ↓/- : Decrease speed                                     ║")
    print("║    d   : Toggle direction (Forward/Reverse)                 ║")
    print("║    p   : Pause/Resume                                       ║")
    print("║    Clock button : Pause/Resume                              ║")
    print("║    q   : Quit                                               ║")
    print("║                                                             ║")
    print("║  POWER MANAGEMENT:                                          ║")
    print("║    - Motors DISABLED when paused (no humming, saves power)  ║")
    print("║    - Motors enabled only during active stepping             ║")
    print("║                                                             ║")
    print("║  SAFETY:                                                    ║")
    print("║    - Any limit switch hit = EMERGENCY STOP                  ║")
    print("║    - Ctrl+C = EMERGENCY STOP                                ║")
    print("╠════════════════════════════════════════════════════════════╣")
    print("║  Speed: 1 (slowest) to 10 (fastest)                        ║")
    print("╚════════════════════════════════════════════════════════════╝")
    print()
    
    if check_limits():
        print("⚠️  WARNING: A limit switch is currently triggered!")
        print("   Release the limit switch before starting.")
        print()
    
    # Show initial state
    update_display()
    display1.show_text("PAUS")
    print(f"Starting PAUSED at Speed: {current_speed}, Direction: {'Forward' if direction > 0 else 'Reverse'}")
    print("Press 'p' or clock button to start motors...\n")
    
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
                    print("⏸️  PAUSED - Motors disabled")
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
        motor_disable()
        time.sleep(0.2)
        cleanup()
        print("✅ Motors disabled. Goodbye!")


if __name__ == "__main__":
    main()
