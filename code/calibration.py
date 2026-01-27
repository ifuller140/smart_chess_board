#!/usr/bin/env python3
"""
Smart Chess Board Gantry Calibration System

This script provides:
1. Pre-flight limit switch verification
2. Prusa-style homing to (0,0)
3. Interactive piece-based calibration
4. Persistent calibration storage

Board: 12" x 12" total, 8x8 grid = 1.5" per square
Origin: Back-right corner (X-MIN, Y-MIN limit switches)
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
# STEPPER SEQUENCE (Half-step)
# ==========================
STEP_SEQUENCE = [
    [1, 0, 0, 0], [1, 1, 0, 0], [0, 1, 0, 0], [0, 1, 1, 0],
    [0, 0, 1, 0], [0, 0, 1, 1], [0, 0, 0, 1], [1, 0, 0, 1]
]

# ==========================
# TIMING CONSTANTS
# ==========================
FAST_DELAY = 0.001    # Fast homing
NORMAL_DELAY = 0.002  # Normal movement
SLOW_DELAY = 0.005    # Precision approach

# ==========================
# CALIBRATION FILE
# ==========================
CALIBRATION_FILE = os.path.expanduser("~/.chess_calibration.json")

# ==========================
# BOARD CONSTANTS
# ==========================
BOARD_SIZE_INCHES = 12.0
GRID_SIZE = 8
SQUARE_SIZE_INCHES = BOARD_SIZE_INCHES / GRID_SIZE  # 1.5"

# ==========================
# GLOBAL STATE
# ==========================
current_pos = {'x': 0, 'y': 0}  # In steps
calibration = {
    'steps_per_inch_x': 256.0,  # Default estimate
    'steps_per_inch_y': 256.0,
    'origin_offset_x': 0,
    'origin_offset_y': 0,
    'square_size_inches': SQUARE_SIZE_INCHES,
    'last_calibrated': None
}
magnet_pwm = None

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
    
    # Limit switch inputs
    GPIO.setup(LIMIT_X_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LIMIT_Y_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(LIMIT_CLOCK_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
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
    return GPIO.input(LIMIT_X_PIN) == 0  # Active LOW

def read_y_limit():
    return GPIO.input(LIMIT_Y_PIN) == 0

def read_clock():
    return GPIO.input(LIMIT_CLOCK_PIN) == 0

def wait_for_clock(message="Press clock to continue..."):
    """Wait for clock button press."""
    print(f"\n>>> {message}")
    while not read_clock():
        time.sleep(0.05)
    time.sleep(0.2)  # Debounce

# ==========================
# MOTOR CONTROL
# ==========================
def step_motor(pins, steps, delay=NORMAL_DELAY):
    """Move a single motor."""
    direction = 1 if steps > 0 else -1
    for _ in range(abs(steps)):
        for seq in (STEP_SEQUENCE if direction > 0 else reversed(STEP_SEQUENCE)):
            for i, pin in enumerate(pins):
                GPIO.output(pin, seq[i])
            time.sleep(delay)

def move_corexy(dx, dy, delay=NORMAL_DELAY):
    """Move in CoreXY coordinates."""
    global current_pos
    
    # CoreXY kinematics:
    # Motor A = X + Y
    # Motor B = X - Y
    steps_a = dx + dy
    steps_b = dx - dy
    
    max_steps = max(abs(steps_a), abs(steps_b))
    if max_steps == 0:
        return
    
    dir_a = 1 if steps_a > 0 else -1
    dir_b = 1 if steps_b > 0 else -1
    
    # Bresenham-style interpolation
    err_a = 0
    err_b = 0
    idx_a = 0
    idx_b = 0
    
    for _ in range(max_steps):
        err_a += abs(steps_a)
        if err_a >= max_steps:
            err_a -= max_steps
            seq_a = STEP_SEQUENCE[idx_a % len(STEP_SEQUENCE)]
            for i, pin in enumerate(MOTOR_A_PINS):
                GPIO.output(pin, seq_a[i])
            idx_a += dir_a
        
        err_b += abs(steps_b)
        if err_b >= max_steps:
            err_b -= max_steps
            seq_b = STEP_SEQUENCE[idx_b % len(STEP_SEQUENCE)]
            for i, pin in enumerate(MOTOR_B_PINS):
                GPIO.output(pin, seq_b[i])
            idx_b += dir_b
        
        time.sleep(delay)
    
    current_pos['x'] += dx
    current_pos['y'] += dy

def move_to_square(file_idx, rank_idx):
    """Move to a chess square (0-7 for file a-h, 0-7 for rank 1-8)."""
    target_x = int((file_idx + 0.5) * SQUARE_SIZE_INCHES * calibration['steps_per_inch_x'])
    target_y = int((rank_idx + 0.5) * SQUARE_SIZE_INCHES * calibration['steps_per_inch_y'])
    
    dx = target_x - current_pos['x']
    dy = target_y - current_pos['y']
    
    move_corexy(dx, dy)

# ==========================
# MAGNET CONTROL
# ==========================
def magnet_engage():
    magnet_pwm.ChangeDutyCycle(2.5)  # Down
    time.sleep(0.5)
    magnet_pwm.ChangeDutyCycle(0)

def magnet_release():
    magnet_pwm.ChangeDutyCycle(7.5)  # Up
    time.sleep(0.5)
    magnet_pwm.ChangeDutyCycle(0)

# ==========================
# PRE-FLIGHT LIMIT SWITCH TEST
# ==========================
def test_limit_switches():
    """Verify all limit switches before calibration."""
    print("\n" + "="*50)
    print("PRE-FLIGHT LIMIT SWITCH VERIFICATION")
    print("="*50)
    
    # Test X limit
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
    
    # Test Y limit
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
    
    # Test Clock limit (already used for confirmation)
    print("\n[3/3] Clock switch already verified through confirmations!")
    print("      ✓ All limit switches working!")
    
    return True

# ==========================
# PRUSA-STYLE HOMING
# ==========================
def home_axis(axis_name, motor_pins, limit_func, max_steps=10000):
    """Home a single axis using Prusa-style sequence."""
    print(f"\n  Homing {axis_name} axis...")
    
    # Phase 1: Fast approach
    print(f"    Phase 1: Fast approach...")
    steps = 0
    while not limit_func() and steps < max_steps:
        step_motor(motor_pins, -10, FAST_DELAY)
        steps += 10
    
    if steps >= max_steps:
        print(f"    [ERROR] {axis_name} limit not found!")
        return False
    
    print(f"    Limit triggered at {steps} steps")
    
    # Phase 2: Back off
    print(f"    Phase 2: Back off...")
    step_motor(motor_pins, 100, NORMAL_DELAY)
    
    # Phase 3: Slow approach
    print(f"    Phase 3: Slow approach...")
    steps = 0
    while not limit_func() and steps < 200:
        step_motor(motor_pins, -1, SLOW_DELAY)
        steps += 1
    
    # Phase 4: Small back off
    print(f"    Phase 4: Small back off...")
    step_motor(motor_pins, 20, NORMAL_DELAY)
    
    # Phase 5: Final approach
    print(f"    Phase 5: Final approach...")
    while not limit_func():
        step_motor(motor_pins, -1, SLOW_DELAY * 2)
    
    print(f"    ✓ {axis_name} axis homed!")
    return True

def home_all():
    """Home both axes to (0,0)."""
    global current_pos
    
    print("\n" + "="*50)
    print("PRUSA-STYLE HOMING SEQUENCE")
    print("="*50)
    
    if not home_axis("X", MOTOR_A_PINS, read_x_limit):
        return False
    
    if not home_axis("Y", MOTOR_B_PINS, read_y_limit):
        return False
    
    current_pos = {'x': 0, 'y': 0}
    print("\n✓ Homing complete! Position: (0, 0)")
    return True

# ==========================
# INTERACTIVE CALIBRATION
# ==========================
def interactive_calibrate():
    """Interactive piece-based calibration."""
    global calibration
    
    print("\n" + "="*50)
    print("INTERACTIVE PIECE CALIBRATION")
    print("="*50)
    
    # Estimate center position
    estimated_center_x = int(6 * SQUARE_SIZE_INCHES * calibration['steps_per_inch_x'])
    estimated_center_y = int(6 * SQUARE_SIZE_INCHES * calibration['steps_per_inch_y'])
    
    print(f"\nMoving to estimated board center...")
    move_corexy(estimated_center_x, estimated_center_y)
    
    print("\nEngaging magnet...")
    magnet_engage()
    
    wait_for_clock("Place a chess piece under the magnet, then press clock")
    
    # Calibrate X axis
    print("\n--- X-Axis Calibration ---")
    print("The gantry will move slowly in +X direction.")
    print("Press clock when piece is CENTERED on column H (far edge).")
    
    start_x = current_pos['x']
    while not read_clock():
        move_corexy(10, 0, SLOW_DELAY)
        time.sleep(0.05)
    
    x_edge_steps = current_pos['x']
    time.sleep(0.3)  # Debounce
    
    # Return to center
    move_corexy(estimated_center_x - current_pos['x'], 0)
    
    # Calibrate Y axis
    print("\n--- Y-Axis Calibration ---")
    print("The gantry will move slowly in +Y direction.")
    print("Press clock when piece is CENTERED on rank 8 (far edge).")
    
    while not read_clock():
        move_corexy(0, 10, SLOW_DELAY)
        time.sleep(0.05)
    
    y_edge_steps = current_pos['y']
    time.sleep(0.3)
    
    # Calculate steps per inch
    # Edge should be at 7.5 squares from origin (center of H8)
    edge_distance = 7.5 * SQUARE_SIZE_INCHES  # inches
    
    calibration['steps_per_inch_x'] = x_edge_steps / edge_distance
    calibration['steps_per_inch_y'] = y_edge_steps / edge_distance
    calibration['last_calibrated'] = datetime.now().isoformat()
    
    print(f"\n✓ Calibration complete!")
    print(f"  Steps/inch X: {calibration['steps_per_inch_x']:.2f}")
    print(f"  Steps/inch Y: {calibration['steps_per_inch_y']:.2f}")
    
    magnet_release()
    return True

# ==========================
# CALIBRATION PERSISTENCE
# ==========================
def save_calibration():
    """Save calibration to file."""
    with open(CALIBRATION_FILE, 'w') as f:
        json.dump(calibration, f, indent=2)
    print(f"\n✓ Calibration saved to {CALIBRATION_FILE}")

def load_calibration():
    """Load calibration from file."""
    global calibration
    if os.path.exists(CALIBRATION_FILE):
        with open(CALIBRATION_FILE, 'r') as f:
            calibration = json.load(f)
        print(f"✓ Loaded calibration from {CALIBRATION_FILE}")
        print(f"  Last calibrated: {calibration.get('last_calibrated', 'Unknown')}")
        return True
    return False

# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    load_calibration()
    
    try:
        while True:
            print("\n" + "="*50)
            print("SMART CHESS BOARD CALIBRATION")
            print("="*50)
            print("1. Test Limit Switches (Pre-flight)")
            print("2. Home Gantry (Prusa-style)")
            print("3. Full Calibration (Home + Interactive)")
            print("4. Verify Calibration (Edge Trace)")
            print("5. Move to Square (Manual Test)")
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
                            save_calibration()
            elif choice == '4':
                verify_calibration()
            elif choice == '5':
                manual_move()
            elif choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cleanup()
        print("GPIO cleaned up. Goodbye!")

def verify_calibration():
    """Trace the edge of the board to verify calibration."""
    print("\n--- Calibration Verification ---")
    print("The gantry will trace the board perimeter.")
    
    magnet_engage()
    wait_for_clock("Place a piece, then press clock to start")
    
    # Move to A1 (0,0)
    print("Moving to A1...")
    move_to_square(0, 0)
    wait_for_clock("Confirm piece is centered on A1")
    
    # A1 -> H1
    print("Moving to H1...")
    move_to_square(7, 0)
    wait_for_clock("Confirm piece is centered on H1")
    
    # H1 -> H8
    print("Moving to H8...")
    move_to_square(7, 7)
    wait_for_clock("Confirm piece is centered on H8")
    
    # H8 -> A8
    print("Moving to A8...")
    move_to_square(0, 7)
    wait_for_clock("Confirm piece is centered on A8")
    
    # A8 -> A1
    print("Returning to A1...")
    move_to_square(0, 0)
    
    magnet_release()
    print("\n✓ Verification complete!")

def manual_move():
    """Manually move to a specific square."""
    try:
        file_str = input("Enter file (a-h): ").strip().lower()
        rank_str = input("Enter rank (1-8): ").strip()
        
        file_idx = ord(file_str) - ord('a')
        rank_idx = int(rank_str) - 1
        
        if 0 <= file_idx <= 7 and 0 <= rank_idx <= 7:
            print(f"Moving to {file_str}{rank_str}...")
            move_to_square(file_idx, rank_idx)
            print("Done.")
        else:
            print("Invalid square.")
    except:
        print("Invalid input.")

if __name__ == "__main__":
    main()
