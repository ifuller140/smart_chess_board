#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time

# ==========================
# GPIO PIN DEFINITIONS
# ==========================
# Motor A
IN1_A = 2
IN2_A = 3
IN3_A = 4
IN4_A = 14

# Motor B
IN1_B = 24
IN2_B = 23
IN3_B = 22
IN4_B = 27

# Define GPIO pins in arrays
motorA_pins = [IN1_A, IN2_A, IN3_A, IN4_A]
motorB_pins = [IN1_B, IN2_B, IN3_B, IN4_B]

# ==========================
# STEPPER SEQUENCE
# ==========================
# Full_step sequence for 28BYJ-48
seq = [
    [1,1,0,0],
    [0,1,1,0],
    [0,0,1,1],
    [1,0,0,1]
]

# ==========================
# GPIO SETUP
# ==========================
GPIO.setmode(GPIO.BCM)
for pin in motorA_pins + motorB_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# ==========================
# STEPPER MOTOR FUNCTION
# ==========================
def step_motor(pins, direction=1, delay=0.002):
    """Step a single motor one step in the given direction."""
    global seq
    seq_len = len(seq)
    for step in range(seq_len):
        for pin in range(4):
            GPIO.output(pins[pin], seq[step][pin] if direction > 0 else seq[-step - 1][pin])
        time.sleep(delay)

def move_both(dirA, dirB, steps, delay=0.002):
    """Move both motors simultaneously."""
    for _ in range(steps):
        step_motor(motorA_pins, dirA, delay)
        step_motor(motorB_pins, dirB, delay)

# ==========================
# COREXY MOTION FUNCTIONS
# ==========================
def move_x(distance_steps, delay=0.002):
    """Move along X axis (both motors same direction)."""
    move_both(1, 1, distance_steps, delay)

def move_y(distance_steps, delay=0.002):
    """Move along Y axis (motors opposite directions)."""
    move_both(1, -1, distance_steps, delay)

def move_diag_xy(distance_steps, delay=0.002):
    """Move along diagonal bottom-left to top-right (X+Y+)."""
    move_both(1, 0, distance_steps, delay)  # Adjust ratio if needed

def move_diag_yx(distance_steps, delay=0.002):
    """Move along diagonal top-left to bottom-right (X-Y+)."""
    move_both(1, -1, distance_steps, delay)

# ==========================
# MAIN PROGRAM
# ==========================
try:
    steps_per_side = 300  # Adjust for desired size
    delay = 0.002

    print("Starting CoreXY square pattern...")

    # Square movement
    move_x(steps_per_side, delay)
    move_y(steps_per_side, delay)
    move_x(-steps_per_side, delay)
    move_y(-steps_per_side, delay)

    print("Square complete. Drawing X pattern...")

    # X pattern (diagonals)
    move_both(1, -1, steps_per_side, delay)   # diagonal 1
    move_both(-1, 1, steps_per_side, delay)   # diagonal 2

    print("Pattern complete.")

except KeyboardInterrupt:
    print("Interrupted by user.")
finally:
    GPIO.cleanup()
