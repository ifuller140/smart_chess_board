#!/usr/bin/env python3
"""
Smart Chess Board Gantry Calibration System
Updated for NEMA 11 Motors + A4988 Drivers

This script provides:
1. Pre-flight limit switch verification
2. Prusa-style homing to (0,0) with safety offset
3. A1/H8 corner finding for accurate calibration
4. Interactive piece-based calibration
5. Persistent calibration and odometry storage
6. Safety mechanism: auto-stop on unexpected limit triggers

Physical Layout:
- Motor A at bottom-left corner of board
- Motor B at top-right corner of board
- Origin (0,0) at back-left corner (X-MIN, Y-MIN limit switches)

CoreXY Kinematics for this layout:
- +X (right): Motor A CW (+), Motor B CCW (-) = OPPOSITE directions
- +Y (up): Motor A CW (+), Motor B CW (+) = SAME direction

Board: 12" x 12" total, 8x8 grid = 1.5" per square
"""

import RPi.GPIO as GPIO
import time
import json
import os
import curses
import sys
import signal
import atexit
from datetime import datetime

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# A4988 Driver Configuration
# ==========================
MOTOR_A_STEP_PIN = 22   # Step pulse
MOTOR_A_DIR_PIN = 27    # Direction
MOTOR_B_STEP_PIN = 5    # Step pulse
MOTOR_B_DIR_PIN = 6     # Direction

# Limit Switches (Active HIGH: 1=Pressed, 0=Released)
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_PIN = 15

# Magnet Servo
SERVO_MAGNET_PIN = 12

# ==========================
# A4988 TIMING CONSTANTS
# ==========================
# A4988 Minimum Timing:
# - DIR setup time: 200ns (we use 5µs for safety)
# - STEP pulse width: 1µs minimum (we use 20µs for stability)
# - STEP low time: 1µs minimum

DIR_SETUP_US = 5            # Microseconds to wait after setting DIR
STEP_PULSE_US = 20          # Microseconds for step pulse width

# ==========================
# SPEED CONFIGURATION
# Step delays in MILLISECONDS (larger = slower)
# ==========================
# Speed percentages map to these delays:
SPEED_90_DELAY_MS = 3.0     # 90% speed - operational movement
SPEED_70_DELAY_MS = 10.0    # 70% speed - moderate
SPEED_50_DELAY_MS = 20.0    # 50% speed - calibration
SPEED_30_DELAY_MS = 35.0    # 30% speed - slow
SPEED_20_DELAY_MS = 50.0    # 20% speed - precision

# Named speeds for different operations
OPERATIONAL_SPEED = 90      # General movement
CALIBRATION_SPEED = 50      # Initial calibration moves
PRECISION_SPEED = 20        # Slow homing approach

# Current speed setting
current_speed = CALIBRATION_SPEED

def speed_to_delay(speed_percent):
    """Convert speed percentage (0-100) to step delay in seconds."""
    speed = max(0, min(100, speed_percent))
    
    # Linear interpolation between key points
    if speed >= 90:
        delay_ms = SPEED_90_DELAY_MS
    elif speed >= 70:
        delay_ms = SPEED_90_DELAY_MS + (90 - speed) / 20 * (SPEED_70_DELAY_MS - SPEED_90_DELAY_MS)
    elif speed >= 50:
        delay_ms = SPEED_70_DELAY_MS + (70 - speed) / 20 * (SPEED_50_DELAY_MS - SPEED_70_DELAY_MS)
    elif speed >= 30:
        delay_ms = SPEED_50_DELAY_MS + (50 - speed) / 20 * (SPEED_30_DELAY_MS - SPEED_50_DELAY_MS)
    elif speed >= 20:
        delay_ms = SPEED_30_DELAY_MS + (30 - speed) / 10 * (SPEED_20_DELAY_MS - SPEED_30_DELAY_MS)
    else:
        delay_ms = SPEED_20_DELAY_MS + (20 - speed) * 5  # Even slower below 20%
    
    return delay_ms / 1000.0  # Convert to seconds

# ==========================
# BOARD CONSTANTS
# ==========================
BOARD_SIZE_INCHES = 12.0
GRID_SIZE = 8
SQUARE_SIZE_INCHES = BOARD_SIZE_INCHES / GRID_SIZE  # 1.5"

# ==========================
# CALIBRATION & STATE FILE
# ==========================
CALIBRATION_FILE = os.path.expanduser("~/.chess_calibration.json")

# ==========================
# SAFETY STATE
# ==========================
class SafetyContext:
    """Track which limit switches are expected during current operation."""
    def __init__(self):
        self.x_limit_expected = False
        self.y_limit_expected = False
        self.safety_enabled = True
        self.emergency_stop = False

safety = SafetyContext()

# ==========================
# GLOBAL STATE (Odometry)
# ==========================
class GantryState:
    """Maintains gantry position and calibration."""
    def __init__(self):
        self.pos_x = 0  # Current X position in steps (from origin)
        self.pos_y = 0  # Current Y position in steps (from origin)
        self.calibration = {
            'steps_per_inch_x': 200.0,  # Initial estimate for NEMA 11 @ 200 steps/rev
            'steps_per_inch_y': 200.0,
            'a1_offset_x': 0,           # Steps from origin to A1 center
            'a1_offset_y': 0,
            'h8_x': 0,                  # H8 position (for reference)
            'h8_y': 0,
            'board_width_steps': 0,     # Calculated from A1-H8
            'board_height_steps': 0,
            'square_size_inches': SQUARE_SIZE_INCHES,
            'last_calibrated': None
        }
        self.homed = False

gantry = GantryState()
magnet_pwm = None

# ==========================
# CHESS SQUARE MAPPING
# ==========================
def square_to_steps(file_char: str, rank: int) -> tuple:
    """
    Convert chess notation to step coordinates.
    
    Args:
        file_char: 'a' through 'h'
        rank: 1 through 8
    
    Returns:
        (x_steps, y_steps) tuple for center of square
    
    Note: Origin is past H8, so we add offsets from A1.
    A1 is at (a1_offset_x, a1_offset_y) from origin.
    """
    file_idx = ord(file_char.lower()) - ord('a')  # 0-7 (a=0, h=7)
    rank_idx = rank - 1  # 0-7
    
    # Position relative to A1 (center of A1 is at 0.5 squares from A1 corner)
    x_from_a1 = file_idx * SQUARE_SIZE_INCHES * gantry.calibration['steps_per_inch_x']
    y_from_a1 = rank_idx * SQUARE_SIZE_INCHES * gantry.calibration['steps_per_inch_y']
    
    # Add A1 offset from origin
    x = gantry.calibration['a1_offset_x'] + x_from_a1
    y = gantry.calibration['a1_offset_y'] + y_from_a1
    
    return (int(x), int(y))

def get_all_squares() -> dict:
    """Return coordinate map for all 64 squares."""
    squares = {}
    for f in 'abcdefgh':
        for r in range(1, 9):
            squares[f"{f}{r}"] = square_to_steps(f, r)
    return squares

# ==========================
# GPIO SETUP
# ==========================
def setup():
    global magnet_pwm
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Motor outputs (A4988 STEP/DIR)
    for pin in [MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)
    
    # Limit switch inputs (Active HIGH: 1=Pressed)
    GPIO.setup(LIMIT_X_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(LIMIT_Y_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    GPIO.setup(LIMIT_CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    # Magnet servo
    GPIO.setup(SERVO_MAGNET_PIN, GPIO.OUT)
    magnet_pwm = GPIO.PWM(SERVO_MAGNET_PIN, 50)
    magnet_pwm.start(0)
    
    # Register cleanup handlers
    atexit.register(cleanup)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

def signal_handler(sig, frame):
    """Handle Ctrl+C and termination signals."""
    print("\n\n🛑 EMERGENCY STOP - Signal received!")
    stop_motors()
    cleanup()
    sys.exit(0)

def stop_motors():
    """Immediately stop all motor movement."""
    try:
        GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
        GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)
    except:
        pass

def cleanup():
    """Clean up GPIO and stop motors."""
    stop_motors()
    if magnet_pwm:
        magnet_pwm.stop()
    try:
        GPIO.cleanup()
    except:
        pass

# ==========================
# MAGNET SERVO CONTROL
# ==========================
SERVO_RELEASE_DUTY = 7.5  # Raised position (disengaged from pieces)
SERVO_ENGAGE_DUTY = 2.5   # Lowered position (engaged with pieces)

def disengage_magnet():
    """Raise the magnet servo to disengage from pieces (for homing)."""
    global magnet_pwm
    if magnet_pwm:
        print("  Disengaging magnet (raising servo)...")
        magnet_pwm.ChangeDutyCycle(SERVO_RELEASE_DUTY)
        time.sleep(0.5)  # Wait for servo to reach position
        magnet_pwm.ChangeDutyCycle(0)  # Stop pulses to avoid jitter

def engage_magnet():
    """Lower the magnet servo to engage with pieces."""
    global magnet_pwm
    if magnet_pwm:
        print("  Engaging magnet (lowering servo)...")
        magnet_pwm.ChangeDutyCycle(SERVO_ENGAGE_DUTY)
        time.sleep(0.5)  # Wait for servo to reach position
        magnet_pwm.ChangeDutyCycle(0)  # Stop pulses to avoid jitter

# ==========================
# LIMIT SWITCH READING
# ==========================
def read_x_limit():
    return GPIO.input(LIMIT_X_PIN) == 1

def read_y_limit():
    return GPIO.input(LIMIT_Y_PIN) == 1

def read_clock():
    return GPIO.input(LIMIT_CLOCK_PIN) == 1

def wait_for_clock(message="Press clock to continue..."):
    print(f"\n>>> {message}")
    while not read_clock():
        time.sleep(0.05)
    time.sleep(0.2)  # Debounce

# ==========================
# SAFETY MONITORING
# ==========================
def check_safety_limits():
    """
    Check if an unexpected limit switch is triggered.
    Returns True if safe to continue, False if emergency stop needed.
    """
    if not safety.safety_enabled:
        return True
    
    x_triggered = read_x_limit()
    y_triggered = read_y_limit()
    
    # Check for unexpected triggers
    if x_triggered and not safety.x_limit_expected:
        print("\n🚨 EMERGENCY STOP: X limit triggered unexpectedly!")
        safety.emergency_stop = True
        stop_motors()
        return False
    
    if y_triggered and not safety.y_limit_expected:
        print("\n🚨 EMERGENCY STOP: Y limit triggered unexpectedly!")
        safety.emergency_stop = True
        stop_motors()
        return False
    
    return True

def set_expected_limits(x_expected=False, y_expected=False):
    """Set which limits are expected during current operation."""
    safety.x_limit_expected = x_expected
    safety.y_limit_expected = y_expected

def clear_expected_limits():
    """Clear all expected limits - any trigger is unexpected."""
    safety.x_limit_expected = False
    safety.y_limit_expected = False

# ==========================
# A4988 MOTOR CONTROL
# ==========================
def step_pulse(step_pin):
    """Generate a single step pulse."""
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(STEP_PULSE_US / 1_000_000)
    GPIO.output(step_pin, GPIO.LOW)

def step_both_motors(steps_a: int, steps_b: int, speed: int = None, check_limits: bool = True):
    """
    Step both motors simultaneously with Bresenham interpolation.
    
    CoreXY Kinematics:
    - Pure X movement: Both motors same direction (A+, B+) or (A-, B-)
    - Pure Y movement: Motors opposite directions (A+, B-) or (A-, B+)
    
    Args:
        steps_a: Steps for motor A (negative = reverse)
        steps_b: Steps for motor B (negative = reverse)
        speed: Speed percentage (0-100), uses current_speed if None
        check_limits: If True, check safety limits during movement
    
    Returns:
        True if completed, False if emergency stopped
    """
    global current_speed
    
    if steps_a == 0 and steps_b == 0:
        return True
    
    if speed is None:
        speed = current_speed
    
    delay = speed_to_delay(speed)
    
    # Set directions
    dir_a = GPIO.HIGH if steps_a >= 0 else GPIO.LOW
    dir_b = GPIO.HIGH if steps_b >= 0 else GPIO.LOW
    GPIO.output(MOTOR_A_DIR_PIN, dir_a)
    GPIO.output(MOTOR_B_DIR_PIN, dir_b)
    
    # Wait for DIR to stabilize
    time.sleep(DIR_SETUP_US / 1_000_000)
    
    abs_a = abs(steps_a)
    abs_b = abs(steps_b)
    max_steps = max(abs_a, abs_b)
    
    # Bresenham interpolation
    err_a = 0
    err_b = 0
    
    for step_num in range(max_steps):
        # Safety check every 10 steps
        if check_limits and step_num % 10 == 0:
            if not check_safety_limits():
                return False
        
        do_step_a = False
        do_step_b = False
        
        err_a += abs_a
        if err_a >= max_steps:
            err_a -= max_steps
            do_step_a = True
            
        err_b += abs_b
        if err_b >= max_steps:
            err_b -= max_steps
            do_step_b = True
        
        # Pulse step pins simultaneously
        if do_step_a:
            GPIO.output(MOTOR_A_STEP_PIN, GPIO.HIGH)
        if do_step_b:
            GPIO.output(MOTOR_B_STEP_PIN, GPIO.HIGH)
        
        time.sleep(STEP_PULSE_US / 1_000_000)
        
        GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
        GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)
        
        time.sleep(delay)
    
    return True

def move_x(steps: int, speed: int = None, check_limits: bool = True):
    """
    Move in pure X direction.
    
    Physical layout: Motor A at bottom-left, Motor B at top-right.
    For +X (right): A turns CW (+), B turns CCW (-) = OPPOSITE directions.
    
    Returns True if completed, False if emergency stopped.
    """
    result = step_both_motors(steps, -steps, speed, check_limits)
    if result:
        gantry.pos_x += steps
    return result

def move_y(steps: int, speed: int = None, check_limits: bool = True):
    """
    Move in pure Y direction.
    
    Physical layout: Motor A at bottom-left, Motor B at top-right.
    For +Y (up): A turns CW (+), B turns CW (+) = SAME direction.
    For -Y (down): A turns CCW (-), B turns CCW (-) = SAME direction.
    
    Returns True if completed, False if emergency stopped.
    """
    result = step_both_motors(steps, steps, speed, check_limits)
    if result:
        gantry.pos_y += steps
    return result

def move_to(target_x: int, target_y: int, speed: int = None, check_limits: bool = True):
    """Move to absolute position. Returns True if completed."""
    dx = target_x - gantry.pos_x
    dy = target_y - gantry.pos_y
    
    # Convert to motor steps for CoreXY
    # Based on physical layout (Motor A at bottom-left, Motor B at top-right):
    # For +X: A+, B- (opposite)
    # For +Y: A+, B+ (same)
    # Combined: A = -X + Y (note: signs adjusted for our motor orientation)
    #           B = -X - Y (note: signs adjusted for our motor orientation)
    # Simplified: A = dx + dy, B = -dx + dy
    steps_a = dx + dy
    steps_b = -dx + dy
    
    result = step_both_motors(steps_a, steps_b, speed, check_limits)
    if result:
        gantry.pos_x = target_x
        gantry.pos_y = target_y
    return result

def move_to_square(file_char: str, rank: int, speed: int = None):
    """Move to center of a chess square."""
    target_x, target_y = square_to_steps(file_char, rank)
    print(f"Moving to {file_char.upper()}{rank} ({target_x}, {target_y})...")
    return move_to(target_x, target_y, speed)

# ==========================
# MAGNET CONTROL
# ==========================
def magnet_engage():
    magnet_pwm.ChangeDutyCycle(2.5)
    time.sleep(0.5)
    magnet_pwm.ChangeDutyCycle(0)

def magnet_release():
    magnet_pwm.ChangeDutyCycle(7.5)
    time.sleep(0.5)
    magnet_pwm.ChangeDutyCycle(0)

# ==========================
# PRE-FLIGHT TESTS
# ==========================
def test_limit_switches():
    """Verify all limit switches before calibration."""
    print("\n" + "="*50)
    print("PRE-FLIGHT LIMIT SWITCH VERIFICATION")
    print("="*50)
    
    print("\n[1/3] Testing X-MIN limit switch...")
    print("      Please PRESS the X limit switch.")
    while not read_x_limit():
        time.sleep(0.05)
    print("      ✓ X-MIN detected!")
    print("      Now RELEASE the switch.")
    while read_x_limit():
        time.sleep(0.05)
    print("      ✓ X-MIN released!")
    wait_for_clock("Press clock to confirm X limit works")
    
    print("\n[2/3] Testing Y-MIN limit switch...")
    print("      Please PRESS the Y limit switch.")
    while not read_y_limit():
        time.sleep(0.05)
    print("      ✓ Y-MIN detected!")
    print("      Now RELEASE the switch.")
    while read_y_limit():
        time.sleep(0.05)
    print("      ✓ Y-MIN released!")
    wait_for_clock("Press clock to confirm Y limit works")
    
    print("\n[3/3] Clock switch verified through confirmations!")
    print("      ✓ All limit switches working!")
    return True

# ==========================
# HOMING SEQUENCE
# ==========================
def pre_home_safety():
    """Move away from limits before homing to avoid crashes."""
    print("\n  Safety offset: Moving away from potential limits...")
    
    # Temporarily disable safety for this movement
    safety.safety_enabled = False
    
    # First check if we're already on a limit
    on_x = read_x_limit()
    on_y = read_y_limit()
    
    if on_x or on_y:
        print(f"    Currently on limits: X={on_x}, Y={on_y}")
        print("    Moving away from limits...")
        
        # Move away from limits (positive direction = away from origin)
        if on_x:
            move_x(500, CALIBRATION_SPEED, check_limits=False)
        if on_y:
            move_y(500, CALIBRATION_SPEED, check_limits=False)
    else:
        # Not on limits, but move a bit just to be safe
        move_x(200, CALIBRATION_SPEED, check_limits=False)
        move_y(200, CALIBRATION_SPEED, check_limits=False)
    
    safety.safety_enabled = True
    print("    ✓ Safety offset complete")

def home_x_axis(max_steps: int = 100000):
    """Home X axis using CoreXY movement."""
    print("\n  Homing X axis...")
    
    # Expect X limit during homing
    set_expected_limits(x_expected=True, y_expected=False)
    
    # Phase 1: Fast approach (50% speed)
    print("    Phase 1: Fast approach (50% speed)...")
    steps = 0
    while not read_x_limit() and steps < max_steps:
        move_x(-100, CALIBRATION_SPEED, check_limits=False)
        steps += 100
    
    if steps >= max_steps:
        print("    [ERROR] X limit not found!")
        clear_expected_limits()
        return False
    
    print(f"    Limit triggered at ~{steps} steps")
    
    # Phase 2: Back off (50% speed)
    print("    Phase 2: Back off...")
    move_x(200, CALIBRATION_SPEED, check_limits=False)
    
    # Phase 3: Slow approach (20% speed)
    print("    Phase 3: Slow approach (20% speed)...")
    while not read_x_limit():
        move_x(-10, PRECISION_SPEED, check_limits=False)
    
    # Phase 4: Small back off
    print("    Phase 4: Small back off...")
    move_x(50, CALIBRATION_SPEED, check_limits=False)
    
    # Phase 5: Final approach (20% speed)
    print("    Phase 5: Final approach (20% speed)...")
    while not read_x_limit():
        move_x(-1, PRECISION_SPEED, check_limits=False)
    
    clear_expected_limits()
    print("    ✓ X axis homed!")
    return True

def home_y_axis(max_steps: int = 100000):
    """Home Y axis using CoreXY movement."""
    print("\n  Homing Y axis...")
    
    # Expect Y limit during homing
    set_expected_limits(x_expected=False, y_expected=True)
    
    # Phase 1: Fast approach (50% speed)
    print("    Phase 1: Fast approach (50% speed)...")
    steps = 0
    while not read_y_limit() and steps < max_steps:
        move_y(-100, CALIBRATION_SPEED, check_limits=False)
        steps += 100
    
    if steps >= max_steps:
        print("    [ERROR] Y limit not found!")
        clear_expected_limits()
        return False
    
    print(f"    Limit triggered at ~{steps} steps")
    
    # Phase 2: Back off (50% speed)
    print("    Phase 2: Back off...")
    move_y(200, CALIBRATION_SPEED, check_limits=False)
    
    # Phase 3: Slow approach (20% speed)
    print("    Phase 3: Slow approach (20% speed)...")
    while not read_y_limit():
        move_y(-10, PRECISION_SPEED, check_limits=False)
    
    # Phase 4: Small back off
    print("    Phase 4: Small back off...")
    move_y(50, CALIBRATION_SPEED, check_limits=False)
    
    # Phase 5: Final approach (20% speed)
    print("    Phase 5: Final approach (20% speed)...")
    while not read_y_limit():
        move_y(-1, PRECISION_SPEED, check_limits=False)
    
    clear_expected_limits()
    print("    ✓ Y axis homed!")
    return True

def home_all():
    """Home both axes with safety offset. Disengages magnet first."""
    print("\n" + "="*50)
    print("PRUSA-STYLE HOMING SEQUENCE")
    print("="*50)
    
    # SAFETY: Disengage magnet before homing to avoid dragging on pieces
    disengage_magnet()
    
    # Safety offset first
    pre_home_safety()
    
    # Home X
    if not home_x_axis():
        return False
    
    # Home Y
    if not home_y_axis():
        return False
    
    # Reset position to origin
    gantry.pos_x = 0
    gantry.pos_y = 0
    gantry.homed = True
    
    print("\n✓ Homing complete! Position: (0, 0)")
    save_state()
    return True

# ==========================
# A1/H8 CORNER CALIBRATION
# ==========================
def find_a1_corner_manual():
    """
    Find A1 corner using keyboard control.
    Returns (x, y) position of A1 center.
    """
    print("\n" + "="*50)
    print("FIND A1 CORNER - MANUAL CONTROL")
    print("="*50)
    print("\nUse arrow keys to position the gantry over the CENTER of A1.")
    print("Press 'q' when A1 is centered.")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    try:
        curses.wrapper(_find_corner_loop, "A1")
    except Exception as e:
        print(f"Error: {e}")
        return None
    
    if safety.emergency_stop:
        return None
    
    a1_x = gantry.pos_x
    a1_y = gantry.pos_y
    print(f"\n✓ A1 position recorded: ({a1_x}, {a1_y})")
    return (a1_x, a1_y)

def find_h8_corner_manual():
    """
    Find H8 corner using keyboard control.
    Returns (x, y) position of H8 center.
    """
    print("\n" + "="*50)
    print("FIND H8 CORNER - MANUAL CONTROL")
    print("="*50)
    print("\nUse arrow keys to position the gantry over the CENTER of H8.")
    print("Press 'q' when H8 is centered.")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    try:
        curses.wrapper(_find_corner_loop, "H8")
    except Exception as e:
        print(f"Error: {e}")
        return None
    
    if safety.emergency_stop:
        return None
    
    h8_x = gantry.pos_x
    h8_y = gantry.pos_y
    print(f"\n✓ H8 position recorded: ({h8_x}, {h8_y})")
    return (h8_x, h8_y)

def _find_corner_loop(stdscr, corner_name):
    """Curses-based keyboard control for finding corners."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(10)
    
    steps_per_tick = 20
    min_steps = 5
    max_steps = 100
    
    running = True
    
    while running:
        if safety.emergency_stop:
            break
        
        stdscr.clear()
        stdscr.addstr(0, 0, f"=== FIND {corner_name} CORNER ===")
        stdscr.addstr(2, 0, f"Position: X={gantry.pos_x:6d}  Y={gantry.pos_y:6d} steps")
        
        if gantry.calibration['steps_per_inch_x'] > 0:
            x_in = gantry.pos_x / gantry.calibration['steps_per_inch_x']
            y_in = gantry.pos_y / gantry.calibration['steps_per_inch_y']
            stdscr.addstr(3, 0, f"Position: X={x_in:6.2f}\"  Y={y_in:6.2f}\"")
        
        stdscr.addstr(5, 0, f"Step size: {steps_per_tick} steps/tick")
        stdscr.addstr(6, 0, f"Limits: X={read_x_limit()}  Y={read_y_limit()}")
        stdscr.addstr(8, 0, "Controls:")
        stdscr.addstr(9, 2, "↑↓←→ : Move gantry")
        stdscr.addstr(10, 2, "+/-   : Adjust step size")
        stdscr.addstr(11, 2, "q     : Confirm position")
        stdscr.refresh()
        
        key = stdscr.getch()
        
        if key == ord('q') or key == ord('Q'):
            running = False
            continue
        
        if key == ord('+') or key == ord('='):
            steps_per_tick = min(max_steps, steps_per_tick + 5)
        elif key == ord('-') or key == ord('_'):
            steps_per_tick = max(min_steps, steps_per_tick - 5)
        elif key == curses.KEY_RIGHT:
            if not read_x_limit():
                move_x(steps_per_tick, CALIBRATION_SPEED, check_limits=False)
        elif key == curses.KEY_LEFT:
            move_x(-steps_per_tick, CALIBRATION_SPEED, check_limits=False)
        elif key == curses.KEY_UP:
            if not read_y_limit():
                move_y(steps_per_tick, CALIBRATION_SPEED, check_limits=False)
        elif key == curses.KEY_DOWN:
            move_y(-steps_per_tick, CALIBRATION_SPEED, check_limits=False)

def find_a1_corner_semi_auto():
    """
    Find A1 corner using semi-automatic movement.
    Gantry moves slowly and user presses clock when centered.
    """
    print("\n" + "="*50)
    print("FIND A1 CORNER - SEMI-AUTOMATIC")
    print("="*50)
    
    # Move to estimated A1 position from origin
    # Origin is past H8, so A1 is approximately 10.5" in both X and Y
    estimated_distance = 7 * SQUARE_SIZE_INCHES  # 7 squares worth
    estimated_steps = int(estimated_distance * gantry.calibration['steps_per_inch_x'])
    
    print(f"\nMoving towards estimated A1 ({estimated_steps} steps)...")
    move_to(estimated_steps, estimated_steps, CALIBRATION_SPEED)
    
    print("\n--- Find A1 X Position ---")
    print("Gantry will move in -X direction.")
    print("Press CLOCK when centered over A1 column (file A).")
    
    while not read_clock():
        move_x(-20, PRECISION_SPEED)
        time.sleep(0.01)
    time.sleep(0.3)
    
    a1_x = gantry.pos_x
    print(f"  A1 X position: {a1_x}")
    
    print("\n--- Find A1 Y Position ---")
    print("Gantry will move in +Y direction.")
    print("Press CLOCK when centered over rank 1.")
    
    while not read_clock():
        move_y(20, PRECISION_SPEED)
        time.sleep(0.01)
    time.sleep(0.3)
    
    a1_y = gantry.pos_y
    print(f"  A1 Y position: {a1_y}")
    
    print(f"\n✓ A1 position: ({a1_x}, {a1_y})")
    return (a1_x, a1_y)

def find_h8_corner_semi_auto():
    """
    Find H8 corner using semi-automatic movement.
    """
    print("\n" + "="*50)
    print("FIND H8 CORNER - SEMI-AUTOMATIC")
    print("="*50)
    
    print("\n--- Find H8 X Position ---")
    print("Gantry will move in +X direction.")
    print("Press CLOCK when centered over H8 column (file H).")
    
    while not read_clock():
        move_x(20, PRECISION_SPEED)
        time.sleep(0.01)
    time.sleep(0.3)
    
    h8_x = gantry.pos_x
    print(f"  H8 X position: {h8_x}")
    
    print("\n--- Find H8 Y Position ---")
    print("Gantry will move in -Y direction.")
    print("Press CLOCK when centered over rank 8.")
    
    while not read_clock():
        move_y(-20, PRECISION_SPEED)
        time.sleep(0.01)
    time.sleep(0.3)
    
    h8_y = gantry.pos_y
    print(f"  H8 Y position: {h8_y}")
    
    print(f"\n✓ H8 position: ({h8_x}, {h8_y})")
    return (h8_x, h8_y)

def calibrate_a1_h8():
    """
    Full A1/H8 calibration sequence.
    Calculates steps/inch from the known distance.
    """
    print("\n" + "="*50)
    print("A1/H8 CORNER CALIBRATION")
    print("="*50)
    print("\nThis will find the A1 and H8 corners to calculate steps/inch.")
    print("The distance from A1 to H8 is exactly 7 squares = 10.5 inches.")
    
    print("\nSelect calibration method:")
    print("  1. Manual (keyboard control) - DEFAULT")
    print("  2. Semi-automatic (motor moves, you press clock)")
    
    choice = input("\nSelect (1/2) [1]: ").strip()
    if choice == '2':
        use_semi_auto = True
    else:
        use_semi_auto = False
    
    # Find A1
    if use_semi_auto:
        a1 = find_a1_corner_semi_auto()
    else:
        a1 = find_a1_corner_manual()
    
    if a1 is None:
        print("[ERROR] Failed to find A1")
        return False
    
    # Find H8
    if use_semi_auto:
        h8 = find_h8_corner_semi_auto()
    else:
        h8 = find_h8_corner_manual()
    
    if h8 is None:
        print("[ERROR] Failed to find H8")
        return False
    
    # Calculate steps per inch
    # Distance from A1 to H8 = 7 squares = 10.5 inches
    board_diagonal_inches = 7 * SQUARE_SIZE_INCHES  # 10.5"
    
    dx = abs(h8[0] - a1[0])
    dy = abs(h8[1] - a1[1])
    
    # Calculate steps per inch for each axis
    if dx > 0:
        steps_per_inch_x = dx / board_diagonal_inches
    else:
        steps_per_inch_x = gantry.calibration['steps_per_inch_x']
        print("[WARNING] No X movement detected, using previous value")
    
    if dy > 0:
        steps_per_inch_y = dy / board_diagonal_inches
    else:
        steps_per_inch_y = gantry.calibration['steps_per_inch_y']
        print("[WARNING] No Y movement detected, using previous value")
    
    # Update calibration
    gantry.calibration['a1_offset_x'] = a1[0]
    gantry.calibration['a1_offset_y'] = a1[1]
    gantry.calibration['h8_x'] = h8[0]
    gantry.calibration['h8_y'] = h8[1]
    gantry.calibration['board_width_steps'] = dx
    gantry.calibration['board_height_steps'] = dy
    gantry.calibration['steps_per_inch_x'] = steps_per_inch_x
    gantry.calibration['steps_per_inch_y'] = steps_per_inch_y
    gantry.calibration['last_calibrated'] = datetime.now().isoformat()
    
    print("\n" + "="*50)
    print("✓ CALIBRATION COMPLETE!")
    print("="*50)
    print(f"  A1 offset: ({a1[0]}, {a1[1]}) steps")
    print(f"  H8 position: ({h8[0]}, {h8[1]}) steps")
    print(f"  Board width: {dx} steps ({dx/steps_per_inch_x:.2f}\")")
    print(f"  Board height: {dy} steps ({dy/steps_per_inch_y:.2f}\")")
    print(f"  Steps/inch X: {steps_per_inch_x:.1f}")
    print(f"  Steps/inch Y: {steps_per_inch_y:.1f}")
    
    save_state()
    return True

# ==========================
# LEGACY CALIBRATION (Interactive piece-based)
# ==========================
def interactive_calibrate():
    """Interactive piece-based calibration (legacy method)."""
    print("\n" + "="*50)
    print("INTERACTIVE PIECE CALIBRATION (Legacy)")
    print("="*50)
    print("\nNote: Prefer the A1/H8 calibration method for better accuracy.")
    
    # Move to approx center (4 inches from A1)
    center_steps = int(4 * gantry.calibration['steps_per_inch_x'])
    a1_x = gantry.calibration['a1_offset_x']
    a1_y = gantry.calibration['a1_offset_y']
    
    target_x = a1_x + center_steps
    target_y = a1_y + center_steps
    
    print(f"\nMoving to estimated center ({target_x}, {target_y})...")
    move_to(target_x, target_y)
    
    print("\nEngaging magnet...")
    magnet_engage()
    
    wait_for_clock("Place a chess piece under the magnet, then press clock")
    
    # Calibrate X
    print("\n--- X-Axis Calibration ---")
    print("Gantry will move in +X direction.")
    print("Press clock when piece is CENTERED on column H.")
    
    while not read_clock():
        move_x(50, PRECISION_SPEED)
        time.sleep(0.02)
    
    x_edge = gantry.pos_x
    time.sleep(0.3)
    
    # Move back to center for Y calibration
    move_to(target_x, gantry.pos_y)
    
    # Calibrate Y
    print("\n--- Y-Axis Calibration ---")
    print("Gantry will move in +Y direction.")
    print("Press clock when piece is CENTERED on rank 8.")
    
    while not read_clock():
        move_y(50, PRECISION_SPEED)
        time.sleep(0.02)
    
    y_edge = gantry.pos_y
    time.sleep(0.3)
    
    # Calculate steps per inch
    # H8 center is at 7.5 squares from A1 = 7.5 * 1.5 = 11.25 inches
    edge_inches = 7.5 * SQUARE_SIZE_INCHES
    
    gantry.calibration['steps_per_inch_x'] = (x_edge - a1_x) / edge_inches
    gantry.calibration['steps_per_inch_y'] = (y_edge - a1_y) / edge_inches
    gantry.calibration['last_calibrated'] = datetime.now().isoformat()
    
    print(f"\n✓ Calibration complete!")
    print(f"  Steps/inch X: {gantry.calibration['steps_per_inch_x']:.1f}")
    print(f"  Steps/inch Y: {gantry.calibration['steps_per_inch_y']:.1f}")
    
    magnet_release()
    return True

# ==========================
# STATE PERSISTENCE
# ==========================
def save_state():
    """Save calibration and current position."""
    state = {
        'calibration': gantry.calibration,
        'position': {'x': gantry.pos_x, 'y': gantry.pos_y},
        'homed': gantry.homed,
        'saved_at': datetime.now().isoformat()
    }
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(state, f, indent=2)
    print(f"✓ State saved to {CALIBRATION_FILE}")

def load_state():
    """Load calibration and position if available."""
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, 'r') as f:
            state = json.load(f)
        
        gantry.calibration = state.get('calibration', gantry.calibration)
        pos = state.get('position', {'x': 0, 'y': 0})
        gantry.pos_x = pos['x']
        gantry.pos_y = pos['y']
        gantry.homed = state.get('homed', False)
        
        print(f"✓ Loaded state from {CALIBRATION_FILE}")
        print(f"  Position: ({gantry.pos_x}, {gantry.pos_y})")
        print(f"  Steps/inch: X={gantry.calibration['steps_per_inch_x']:.0f}, Y={gantry.calibration['steps_per_inch_y']:.0f}")
        print(f"  A1 offset: ({gantry.calibration.get('a1_offset_x', 0)}, {gantry.calibration.get('a1_offset_y', 0)})")
        print(f"  Homed: {gantry.homed}")
        return True
    return False

# ==========================
# VERIFICATION
# ==========================
def verify_calibration():
    """Trace board perimeter to verify calibration."""
    print("\n--- Calibration Verification ---")
    
    if not gantry.homed:
        print("[WARNING] Gantry not homed. Homing first...")
        if not home_all():
            return False
    
    magnet_engage()
    wait_for_clock("Place a piece, then press clock to start")
    
    corners = [('a', 1), ('h', 1), ('h', 8), ('a', 8), ('a', 1)]
    
    for f, r in corners:
        if not move_to_square(f, r, OPERATIONAL_SPEED):
            print("[ERROR] Movement failed!")
            magnet_release()
            return False
        wait_for_clock(f"Confirm piece is centered on {f.upper()}{r}")
    
    magnet_release()
    save_state()
    print("\n✓ Verification complete!")
    return True

def manual_move():
    """Move to a specific square."""
    if not gantry.homed:
        print("[WARNING] Gantry not homed!")
    
    try:
        square = input("Enter square (e.g., e4): ").strip().lower()
        if len(square) == 2:
            file_char = square[0]
            rank = int(square[1])
            if file_char in 'abcdefgh' and 1 <= rank <= 8:
                move_to_square(file_char, rank, OPERATIONAL_SPEED)
                print(f"Current position: ({gantry.pos_x}, {gantry.pos_y})")
                return
        print("Invalid square.")
    except:
        print("Invalid input.")

def show_status():
    """Display current gantry status."""
    print("\n--- Gantry Status ---")
    print(f"Position: ({gantry.pos_x}, {gantry.pos_y}) steps")
    if gantry.calibration['steps_per_inch_x'] > 0:
        x_in = gantry.pos_x / gantry.calibration['steps_per_inch_x']
        y_in = gantry.pos_y / gantry.calibration['steps_per_inch_y']
        print(f"Position: ({x_in:.2f}\", {y_in:.2f}\")")
    print(f"Homed: {gantry.homed}")
    print(f"Steps/inch: X={gantry.calibration['steps_per_inch_x']:.0f}, Y={gantry.calibration['steps_per_inch_y']:.0f}")
    print(f"A1 offset: ({gantry.calibration.get('a1_offset_x', 0)}, {gantry.calibration.get('a1_offset_y', 0)})")
    print(f"Last calibrated: {gantry.calibration.get('last_calibrated', 'Never')}")

# ==========================
# MANUAL KEYBOARD CONTROL
# ==========================
def manual_keyboard_control():
    """Real-time keyboard control of gantry."""
    print("\n" + "="*50)
    print("MANUAL KEYBOARD CONTROL")
    print("="*50)
    print("Use arrow keys to move. Press 'q' to exit.")
    print("+/- to adjust speed. Diagonal movement supported.")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    try:
        curses.wrapper(_keyboard_control_loop)
    except Exception as e:
        print(f"Error: {e}")
    
    print(f"\nFinal position: ({gantry.pos_x}, {gantry.pos_y})")
    save_state()

def _keyboard_control_loop(stdscr):
    """Curses-based keyboard control loop."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(10)
    
    steps_per_tick = 50
    min_steps = 10
    max_steps = 200
    
    running = True
    
    while running:
        if safety.emergency_stop:
            break
        
        stdscr.clear()
        stdscr.addstr(0, 0, "=== MANUAL GANTRY CONTROL ===")
        stdscr.addstr(2, 0, f"Position: X={gantry.pos_x:6d}  Y={gantry.pos_y:6d} steps")
        
        if gantry.calibration['steps_per_inch_x'] > 0:
            x_in = gantry.pos_x / gantry.calibration['steps_per_inch_x']
            y_in = gantry.pos_y / gantry.calibration['steps_per_inch_y']
            stdscr.addstr(3, 0, f"Position: X={x_in:6.2f}\"  Y={y_in:6.2f}\"")
        
        stdscr.addstr(5, 0, f"Speed: {steps_per_tick} steps/tick")
        stdscr.addstr(6, 0, f"Limits: X={read_x_limit()}  Y={read_y_limit()}")
        stdscr.addstr(8, 0, "Controls:")
        stdscr.addstr(9, 2, "↑↓←→ : Move gantry")
        stdscr.addstr(10, 2, "+/-   : Adjust speed")
        stdscr.addstr(11, 2, "q     : Quit")
        stdscr.addstr(13, 0, "Hold multiple arrows for diagonal movement")
        stdscr.refresh()
        
        key = stdscr.getch()
        
        if key == ord('q') or key == ord('Q'):
            running = False
            continue
        
        if key == ord('+') or key == ord('='):
            steps_per_tick = min(max_steps, steps_per_tick + 10)
        elif key == ord('-') or key == ord('_'):
            steps_per_tick = max(min_steps, steps_per_tick - 10)
        elif key == curses.KEY_RIGHT:
            move_x(steps_per_tick, OPERATIONAL_SPEED, check_limits=False)
        elif key == curses.KEY_LEFT:
            if not read_x_limit():
                move_x(-steps_per_tick, OPERATIONAL_SPEED, check_limits=False)
        elif key == curses.KEY_UP:
            move_y(steps_per_tick, OPERATIONAL_SPEED, check_limits=False)
        elif key == curses.KEY_DOWN:
            if not read_y_limit():
                move_y(-steps_per_tick, OPERATIONAL_SPEED, check_limits=False)

# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    load_state()
    
    try:
        while True:
            if safety.emergency_stop:
                print("\n🚨 Emergency stop was triggered. Exiting.")
                break
            
            print("\n" + "="*50)
            print("SMART CHESS BOARD CALIBRATION")
            print("(NEMA 11 + A4988 Drivers)")
            print("="*50)
            print("1. Test Limit Switches")
            print("2. Home Gantry")
            print("3. A1/H8 Calibration (RECOMMENDED)")
            print("4. Full Calibration (Home + A1/H8)")
            print("5. Legacy Calibration (Piece-based)")
            print("6. Verify Calibration (Edge Trace)")
            print("7. Move to Square")
            print("8. Show Status")
            print("9. Manual Keyboard Control")
            print("q. Quit")
            
            choice = input("\nSelect option: ").strip().lower()
            
            if choice == '1':
                test_limit_switches()
            elif choice == '2':
                home_all()
            elif choice == '3':
                if not gantry.homed:
                    print("\n[WARNING] Gantry not homed. Homing first...")
                    if not home_all():
                        continue
                calibrate_a1_h8()
            elif choice == '4':
                if home_all():
                    calibrate_a1_h8()
            elif choice == '5':
                if not gantry.homed:
                    print("\n[WARNING] Gantry not homed. Homing first...")
                    if not home_all():
                        continue
                interactive_calibrate()
            elif choice == '6':
                verify_calibration()
            elif choice == '7':
                manual_move()
            elif choice == '8':
                show_status()
            elif choice == '9':
                manual_keyboard_control()
            elif choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        save_state()
        cleanup()
        print("GPIO cleaned up. Goodbye!")

if __name__ == "__main__":
    main()
