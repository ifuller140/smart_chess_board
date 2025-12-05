#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import keyboard  # pip install keyboard

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
# STEPPER SEQUENCE (28BYJ-48)
# ==========================
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
# STEPPER CONTROL FUNCTIONS
# ==========================
def step_motor(pins, direction=1, delay=0.002):
    """Step a single motor one cycle in the given direction."""
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
# COREXY MOVEMENT FUNCTIONS
# ==========================
def move_x(steps, delay=0.002):
    """Move X-axis: both motors same direction."""
    direction = 1 if steps > 0 else -1
    move_both(direction, direction, abs(steps), delay)

def move_y(steps, delay=0.002):
    """Move Y-axis: motors opposite directions."""
    direction = 1 if steps > 0 else -1
    move_both(direction, -direction, abs(steps), delay)

# ==========================
# MAIN LOOP: KEYBOARD CONTROL
# ==========================
try:
    step_size = 20     # Number of steps per key press
    delay = 0.0015     # Step timing (lower = faster)

    print("Manual Gantry Control Active!")
    print("Use arrow keys to move. Press 'q' to quit.\n")

    while True:
        if keyboard.is_pressed('up'):
            print("Moving UP (Y+)")
            move_y(step_size, delay)

        elif keyboard.is_pressed('down'):
            print("Moving DOWN (Y-)")
            move_y(-step_size, delay)

        elif keyboard.is_pressed('left'):
            print("Moving LEFT (X-)")
            move_x(-step_size, delay)

        elif keyboard.is_pressed('right'):
            print("Moving RIGHT (X+)")
            move_x(step_size, delay)

        elif keyboard.is_pressed('q'):
            print("Exiting program...")
            break

        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nStopped by user.")

finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
