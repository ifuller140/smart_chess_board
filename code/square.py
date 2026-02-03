#!/usr/bin/env python3
"""
CoreXY Square Pattern Test for A4988 Drivers + NEMA 11 Motors.

A quick test to verify motor wiring and CoreXY movement.
"""

import RPi.GPIO as GPIO
import time

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# A4988 Driver Pins (NEMA 11 Motors)
MOTOR_A_DIR_PIN = 27    # Direction pin for Motor A
MOTOR_A_STEP_PIN = 22   # Step pin for Motor A
MOTOR_B_DIR_PIN = 6     # Direction pin for Motor B
MOTOR_B_STEP_PIN = 5    # Step pin for Motor B

# ==========================
# TIMING CONSTANTS
# ==========================
STEP_PULSE_US = 10      # Microseconds for step pulse width
DEFAULT_SPEED = 50      # Default speed (0-100)
MIN_STEP_DELAY_US = 100     # Maximum speed (100%)
MAX_STEP_DELAY_US = 5000    # Minimum speed (0%)


def speed_to_delay(speed_percent):
    """Convert speed percentage (0-100) to step delay in seconds."""
    speed = max(0, min(100, speed_percent))
    delay_us = MAX_STEP_DELAY_US - (speed / 100.0) * (MAX_STEP_DELAY_US - MIN_STEP_DELAY_US)
    return delay_us / 1_000_000


# ==========================
# GPIO SETUP
# ==========================
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)


# ==========================
# STEPPER FUNCTIONS
# ==========================
def step_both_motors(steps_a, steps_b, speed=DEFAULT_SPEED):
    """
    Step both motors simultaneously with Bresenham interpolation.
    
    Args:
        steps_a: Steps for motor A (negative = reverse)
        steps_b: Steps for motor B (negative = reverse)
        speed: Speed percentage (0-100)
    """
    delay = speed_to_delay(speed)
    pulse_sec = STEP_PULSE_US / 1_000_000
    
    if steps_a == 0 and steps_b == 0:
        return
    
    # Set directions
    GPIO.output(MOTOR_A_DIR_PIN, GPIO.HIGH if steps_a >= 0 else GPIO.LOW)
    GPIO.output(MOTOR_B_DIR_PIN, GPIO.HIGH if steps_b >= 0 else GPIO.LOW)
    
    abs_a = abs(steps_a)
    abs_b = abs(steps_b)
    max_steps = max(abs_a, abs_b)
    
    # Bresenham interpolation
    err_a = 0
    err_b = 0
    
    for _ in range(max_steps):
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
        
        time.sleep(pulse_sec)
        
        GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
        GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)
        
        time.sleep(delay)


def move_x(steps, speed=DEFAULT_SPEED):
    """Move along X axis. Motor A at bottom-left, Motor B at top-right.
    For +X (right): A CW (+), B CCW (-) = OPPOSITE directions."""
    step_both_motors(steps, -steps, speed)


def move_y(steps, speed=DEFAULT_SPEED):
    """Move along Y axis. Motor A at bottom-left, Motor B at top-right.
    For +Y (up): A CW (+), B CW (+) = SAME direction."""
    step_both_motors(steps, steps, speed)


# ==========================
# MAIN PROGRAM
# ==========================
try:
    steps_per_side = 200  # Adjust for desired size
    speed = 50  # Start at 50% speed
    
    print("╔════════════════════════════════════════════╗")
    print("║   CoreXY Square Pattern Test               ║")
    print("║   (A4988 + NEMA 11 Motors)                 ║")
    print("╠════════════════════════════════════════════╣")
    print(f"║   Steps per side: {steps_per_side:4d}                    ║")
    print(f"║   Speed: {speed:3d}%                              ║")
    print("╚════════════════════════════════════════════╝")
    print()
    
    print("Drawing square pattern...")
    print("  → Moving +X...")
    move_x(steps_per_side, speed)
    time.sleep(0.3)
    
    print("  ↑ Moving +Y...")
    move_y(steps_per_side, speed)
    time.sleep(0.3)
    
    print("  ← Moving -X...")
    move_x(-steps_per_side, speed)
    time.sleep(0.3)
    
    print("  ↓ Moving -Y...")
    move_y(-steps_per_side, speed)
    time.sleep(0.3)
    
    print()
    print("Square complete! Drawing X pattern (diagonals)...")
    
    # X pattern (diagonals) - Motor A only, then Motor B only
    print("  ╱ Moving diagonal (Motor A only)...")
    step_both_motors(steps_per_side, 0, speed)
    time.sleep(0.3)
    
    print("  ╲ Moving diagonal (Motor B only)...")
    step_both_motors(-steps_per_side, steps_per_side, speed)
    time.sleep(0.3)
    
    print()
    print("✓ Pattern complete!")

except KeyboardInterrupt:
    print("\nInterrupted by user.")
finally:
    GPIO.cleanup()
    print("GPIO cleaned up.")
