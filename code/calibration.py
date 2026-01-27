#!/usr/bin/env python3
"""
Smart Chess Board Gantry Calibration System

This script provides:
1. Pre-flight limit switch verification
2. Prusa-style homing to (0,0) with safety offset
3. Interactive piece-based calibration
4. Persistent calibration and odometry storage

Board: 12" x 12" total, 8x8 grid = 1.5" per square
Origin: Back-right corner (X-MIN, Y-MIN limit switches)
Coordinate System: +X = left, +Y = forward
"""

import RPi.GPIO as GPIO
import time
import json
import os
from datetime import datetime

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
MOTOR_A_PINS = [14, 4, 3, 2]    # IN1, IN2, IN3, IN4
MOTOR_B_PINS = [24, 23, 22, 27]  # IN1, IN2, IN3, IN4

# Limit Switches
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_PIN = 15

# Magnet Servo
SERVO_MAGNET_PIN = 12

# ==========================
# STEPPER SEQUENCE (Half-step for 28BYJ-48)
# ==========================
STEP_SEQUENCE = [
    [1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0],
    [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]
]
SEQ_LEN = len(STEP_SEQUENCE)

# ==========================
# TIMING CONSTANTS
# ==========================
FAST_DELAY = 0.0008   # Fast homing
NORMAL_DELAY = 0.001  # Normal movement
SLOW_DELAY = 0.003    # Precision approach

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
# GLOBAL STATE (Odometry)
# ==========================
class GantryState:
    """Maintains gantry position and calibration."""
    def __init__(self):
        self.pos_x = 0  # Current X position in steps
        self.pos_y = 0  # Current Y position in steps
        self.idx_a = 0  # Motor A step index
        self.idx_b = 0  # Motor B step index
        self.calibration = {
            'steps_per_inch_x': 2048.0,  # User confirmed value
            'steps_per_inch_y': 2048.0,
            'origin_offset_x': 0,
            'origin_offset_y': 0,
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
    """
    file_idx = ord(file_char.lower()) - ord('a')  # 0-7
    rank_idx = rank - 1  # 0-7
    
    x = (file_idx + 0.5) * SQUARE_SIZE_INCHES * gantry.calibration['steps_per_inch_x']
    y = (rank_idx + 0.5) * SQUARE_SIZE_INCHES * gantry.calibration['steps_per_inch_y']
    
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
    
    # Motor outputs
    for pin in MOTOR_A_PINS + MOTOR_B_PINS:
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

def cleanup():
    if magnet_pwm:
        magnet_pwm.stop()
    for pin in MOTOR_A_PINS + MOTOR_B_PINS:
        GPIO.output(pin, 0)
    GPIO.cleanup()

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
    time.sleep(0.2)

# ==========================
# COREXY MOTOR CONTROL
# ==========================
def step_both_motors(steps_a: int, steps_b: int, delay: float = NORMAL_DELAY):
    """
    Step both motors simultaneously with interpolation.
    
    CoreXY Kinematics:
    - Pure X movement: Both motors same direction (A+, B+) or (A-, B-)
    - Pure Y movement: Motors opposite directions (A+, B-) or (A-, B+)
    """
    if steps_a == 0 and steps_b == 0:
        return
    
    dir_a = 1 if steps_a >= 0 else -1
    dir_b = 1 if steps_b >= 0 else -1
    abs_a = abs(steps_a)
    abs_b = abs(steps_b)
    max_steps = max(abs_a, abs_b)
    
    err_a = 0
    err_b = 0
    
    for _ in range(max_steps):
        # Motor A
        err_a += abs_a
        if err_a >= max_steps:
            err_a -= max_steps
            gantry.idx_a = (gantry.idx_a + dir_a) % SEQ_LEN
            seq = STEP_SEQUENCE[gantry.idx_a]
            for i, pin in enumerate(MOTOR_A_PINS):
                GPIO.output(pin, seq[i])
        
        # Motor B
        err_b += abs_b
        if err_b >= max_steps:
            err_b -= max_steps
            gantry.idx_b = (gantry.idx_b + dir_b) % SEQ_LEN
            seq = STEP_SEQUENCE[gantry.idx_b]
            for i, pin in enumerate(MOTOR_B_PINS):
                GPIO.output(pin, seq[i])
        
        time.sleep(delay)

def move_x(steps: int, delay: float = NORMAL_DELAY):
    """
    Move in pure X direction.
    CoreXY: X = (A + B) / 2, so for X movement, A and B move SAME direction.
    """
    step_both_motors(steps, steps, delay)
    gantry.pos_x += steps

def move_y(steps: int, delay: float = NORMAL_DELAY):
    """
    Move in pure Y direction.
    CoreXY: Y = (A - B) / 2, so for Y movement, A and B move OPPOSITE directions.
    """
    step_both_motors(steps, -steps, delay)
    gantry.pos_y += steps

def move_to(target_x: int, target_y: int, delay: float = NORMAL_DELAY):
    """Move to absolute position."""
    dx = target_x - gantry.pos_x
    dy = target_y - gantry.pos_y
    
    # Convert to motor steps
    steps_a = dx + dy
    steps_b = dx - dy
    
    step_both_motors(steps_a, steps_b, delay)
    gantry.pos_x = target_x
    gantry.pos_y = target_y

def move_to_square(file_char: str, rank: int, delay: float = NORMAL_DELAY):
    """Move to center of a chess square."""
    target_x, target_y = square_to_steps(file_char, rank)
    print(f"Moving to {file_char}{rank} ({target_x}, {target_y})...")
    move_to(target_x, target_y, delay)

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
    
    # First check if we're already on a limit
    on_x = read_x_limit()
    on_y = read_y_limit()
    
    if on_x or on_y:
        print(f"    Currently on limits: X={on_x}, Y={on_y}")
        print("    Moving away from limits...")
        
        # Move away from limits (positive direction = away from origin)
        if on_x:
            move_x(1000, FAST_DELAY)
        if on_y:
            move_y(1000, FAST_DELAY)
    else:
        # Not on limits, but move a bit just to be safe
        move_x(500, FAST_DELAY)
        move_y(500, FAST_DELAY)
    
    print("    ✓ Safety offset complete")

def home_x_axis(max_steps: int = 50000):
    """Home X axis using CoreXY movement."""
    print("\n  Homing X axis...")
    
    # Phase 1: Fast approach
    print("    Phase 1: Fast approach...")
    steps = 0
    while not read_x_limit() and steps < max_steps:
        move_x(-100, FAST_DELAY)
        steps += 100
    
    if steps >= max_steps:
        print("    [ERROR] X limit not found!")
        return False
    
    print(f"    Limit triggered at ~{steps} steps")
    
    # Phase 2: Back off
    print("    Phase 2: Back off...")
    move_x(200, NORMAL_DELAY)
    
    # Phase 3: Slow approach
    print("    Phase 3: Slow approach...")
    while not read_x_limit():
        move_x(-10, SLOW_DELAY)
    
    # Phase 4: Small back off
    print("    Phase 4: Small back off...")
    move_x(50, NORMAL_DELAY)
    
    # Phase 5: Final approach
    print("    Phase 5: Final approach...")
    while not read_x_limit():
        move_x(-1, SLOW_DELAY)
    
    print("    ✓ X axis homed!")
    return True

def home_y_axis(max_steps: int = 50000):
    """Home Y axis using CoreXY movement."""
    print("\n  Homing Y axis...")
    
    # Phase 1: Fast approach
    print("    Phase 1: Fast approach...")
    steps = 0
    while not read_y_limit() and steps < max_steps:
        move_y(-100, FAST_DELAY)
        steps += 100
    
    if steps >= max_steps:
        print("    [ERROR] Y limit not found!")
        return False
    
    print(f"    Limit triggered at ~{steps} steps")
    
    # Phase 2: Back off
    print("    Phase 2: Back off...")
    move_y(200, NORMAL_DELAY)
    
    # Phase 3: Slow approach
    print("    Phase 3: Slow approach...")
    while not read_y_limit():
        move_y(-10, SLOW_DELAY)
    
    # Phase 4: Small back off
    print("    Phase 4: Small back off...")
    move_y(50, NORMAL_DELAY)
    
    # Phase 5: Final approach
    print("    Phase 5: Final approach...")
    while not read_y_limit():
        move_y(-1, SLOW_DELAY)
    
    print("    ✓ Y axis homed!")
    return True

def home_all():
    """Home both axes with safety offset."""
    print("\n" + "="*50)
    print("PRUSA-STYLE HOMING SEQUENCE")
    print("="*50)
    
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
# CALIBRATION
# ==========================
def interactive_calibrate():
    """Interactive piece-based calibration."""
    print("\n" + "="*50)
    print("INTERACTIVE PIECE CALIBRATION")
    print("="*50)
    
    # Move to approx center (4 inches from origin)
    center_steps = int(4 * gantry.calibration['steps_per_inch_x'])
    print(f"\nMoving to estimated center ({center_steps} steps)...")
    move_to(center_steps, center_steps)
    
    print("\nEngaging magnet...")
    magnet_engage()
    
    wait_for_clock("Place a chess piece under the magnet, then press clock")
    
    # Calibrate X
    print("\n--- X-Axis Calibration ---")
    print("Gantry will move in +X direction.")
    print("Press clock when piece is CENTERED on column H.")
    
    start_x = gantry.pos_x
    while not read_clock():
        move_x(50, SLOW_DELAY)
        time.sleep(0.02)
    
    x_edge = gantry.pos_x
    time.sleep(0.3)
    
    # Move back to center for Y calibration
    move_to(center_steps, gantry.pos_y)
    
    # Calibrate Y
    print("\n--- Y-Axis Calibration ---")
    print("Gantry will move in +Y direction.")
    print("Press clock when piece is CENTERED on rank 8.")
    
    while not read_clock():
        move_y(50, SLOW_DELAY)
        time.sleep(0.02)
    
    y_edge = gantry.pos_y
    time.sleep(0.3)
    
    # Calculate steps per inch
    # H8 center is at 7.5 squares from origin = 7.5 * 1.5 = 11.25 inches
    edge_inches = 7.5 * SQUARE_SIZE_INCHES
    
    gantry.calibration['steps_per_inch_x'] = x_edge / edge_inches
    gantry.calibration['steps_per_inch_y'] = y_edge / edge_inches
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
        move_to_square(f, r)
        wait_for_clock(f"Confirm piece is centered on {f}{r}")
    
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
                move_to_square(file_char, rank)
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

# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    load_state()
    
    try:
        while True:
            print("\n" + "="*50)
            print("SMART CHESS BOARD CALIBRATION")
            print("="*50)
            print("1. Test Limit Switches")
            print("2. Home Gantry")
            print("3. Full Calibration (Home + Interactive)")
            print("4. Verify Calibration (Edge Trace)")
            print("5. Move to Square")
            print("6. Show Status")
            print("q. Quit")
            
            choice = input("\nSelect option: ").strip().lower()
            
            if choice == '1':
                test_limit_switches()
            elif choice == '2':
                home_all()
            elif choice == '3':
                if test_limit_switches():
                    if home_all():
                        if interactive_calibrate():
                            save_state()
            elif choice == '4':
                verify_calibration()
            elif choice == '5':
                manual_move()
            elif choice == '6':
                show_status()
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
