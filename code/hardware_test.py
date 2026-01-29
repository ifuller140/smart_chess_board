#!/usr/bin/env python3
"""
Hardware Test Script for Smart Chess Board.

Supports A4988 stepper drivers with NEMA 11 motors using STEP/DIR control.
"""

import RPi.GPIO as GPIO
import time
import sys

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# Stepper Motors (A4988 Drivers + NEMA 11)
MOTOR_A_DIR_PIN = 27    # Direction pin for Motor A
MOTOR_A_STEP_PIN = 22   # Step pin for Motor A
MOTOR_B_DIR_PIN = 6     # Direction pin for Motor B
MOTOR_B_STEP_PIN = 5    # Step pin for Motor B

# Servos
SERVO_CLOCK_PIN = 18    # Clock servo (see updated pinout.md)
SERVO_MAGNET_PIN = 12   # Z-axis magnet servo

# Limit Switches
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_PIN = 15

# Clock Displays (TM1637-style CLK, DIO)
CLOCK_1 = {'clk': 25, 'dio': 8}
CLOCK_2 = {'clk': 7, 'dio': 1}

# ==========================
# STEPPER TIMING (A4988)
# ==========================
STEP_PULSE_US = 10      # Microseconds for step pulse width (min 2µs for A4988)
MIN_STEP_DELAY_US = 100     # Maximum speed (100%)
MAX_STEP_DELAY_US = 5000    # Minimum speed (0%)

# Current speed setting (0-100)
current_speed = 50


# ==========================
# SPEED CONTROL
# ==========================
def speed_to_delay(speed_percent):
    """Convert speed percentage (0-100) to step delay in seconds."""
    speed = max(0, min(100, speed_percent))
    delay_us = MAX_STEP_DELAY_US - (speed / 100.0) * (MAX_STEP_DELAY_US - MIN_STEP_DELAY_US)
    return delay_us / 1_000_000


def set_speed(speed_percent):
    """Set the current motor speed (0-100)."""
    global current_speed
    current_speed = max(0, min(100, speed_percent))
    print(f"Speed set to {current_speed}%")


# ==========================
# TM1637 DRIVER (Simplified)
# ==========================
class TM1637:
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

    def display_test(self):
        """Display '8888' to test all segments."""
        self._start()
        self._write_byte(0x40)  # Data command
        self._stop()
        self._start()
        self._write_byte(0xC0)  # Address command
        for _ in range(4):
            self._write_byte(0x7F)  # All segments on (except colon)
        self._stop()
        self._start()
        self._write_byte(0x8F)  # Display control (on, max brightness)
        self._stop()


# ==========================
# HARDWARE SETUP
# ==========================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Steppers (A4988 STEP/DIR)
    for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN]:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)
        
    # Servos
    GPIO.setup(SERVO_CLOCK_PIN, GPIO.OUT)
    GPIO.setup(SERVO_MAGNET_PIN, GPIO.OUT)
    
    # Limit Switches (Active HIGH: 1=Pressed, 0=Released)
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_PIN]:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)


# ==========================
# STEPPER MOTOR FUNCTIONS
# ==========================
def step_pulse(step_pin):
    """Generate a single step pulse."""
    GPIO.output(step_pin, GPIO.HIGH)
    time.sleep(STEP_PULSE_US / 1_000_000)
    GPIO.output(step_pin, GPIO.LOW)


def step_motor(step_pin, dir_pin, steps, speed=None):
    """
    Move a single motor a given number of steps.
    
    Args:
        step_pin: GPIO pin for STEP signal
        dir_pin: GPIO pin for DIR signal
        steps: Number of steps (negative = reverse)
        speed: Speed percentage (0-100), uses current_speed if None
    """
    if speed is None:
        speed = current_speed
    
    delay = speed_to_delay(speed)
    
    # Set direction
    GPIO.output(dir_pin, GPIO.HIGH if steps >= 0 else GPIO.LOW)
    
    # Generate step pulses
    for _ in range(abs(steps)):
        step_pulse(step_pin)
        time.sleep(delay)


def step_both_motors(steps_a, steps_b, speed=None):
    """
    Step both motors simultaneously with Bresenham interpolation.
    
    Args:
        steps_a: Steps for motor A (negative = reverse)
        steps_b: Steps for motor B (negative = reverse)
        speed: Speed percentage (0-100)
    """
    if speed is None:
        speed = current_speed
    
    delay = speed_to_delay(speed)
    
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
        
        time.sleep(STEP_PULSE_US / 1_000_000)
        
        GPIO.output(MOTOR_A_STEP_PIN, GPIO.LOW)
        GPIO.output(MOTOR_B_STEP_PIN, GPIO.LOW)
        
        time.sleep(delay)


def move_x(steps, speed=None):
    """Move along X axis (motors opposite directions for CoreXY)."""
    step_both_motors(steps, -steps, speed)


def move_y(steps, speed=None):
    """Move along Y axis (motors same direction for CoreXY)."""
    step_both_motors(steps, steps, speed)


# ==========================
# TEST FUNCTIONS
# ==========================
def test_steppers():
    print("\n--- Stepper Motor Test (A4988 + NEMA 11) ---")
    print(f"Current speed: {current_speed}%")
    
    print("\nTesting Motor A (200 steps forward, then backward)...")
    step_motor(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, 200)
    time.sleep(0.5)
    step_motor(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, -200)
    
    print("\nTesting Motor B (200 steps forward, then backward)...")
    step_motor(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN, 200)
    time.sleep(0.5)
    step_motor(MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN, -200)
    
    print("\nTesting CoreXY X-axis movement...")
    move_x(200)
    time.sleep(0.5)
    move_x(-200)
    
    print("\nTesting CoreXY Y-axis movement...")
    move_y(200)
    time.sleep(0.5)
    move_y(-200)
    
    print("\nStepper test complete.")


def test_speed_range():
    """Test motors at different speeds."""
    print("\n--- Speed Range Test ---")
    print("Testing motor A at different speeds...")
    
    for speed in [20, 40, 60, 80, 100]:
        print(f"\n  Speed: {speed}%")
        step_motor(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, 100, speed)
        time.sleep(0.3)
        step_motor(MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, -100, speed)
        time.sleep(0.5)
    
    print("\nSpeed range test complete.")


def test_servos():
    print("\n--- Servo Test ---")
    p_clock = GPIO.PWM(SERVO_CLOCK_PIN, 50)  # 50Hz
    p_magnet = GPIO.PWM(SERVO_MAGNET_PIN, 50)
    
    p_clock.start(7.5)  # 90 degrees
    p_magnet.start(7.5)
    
    print("Sweeping Clock Servo...")
    for dc in [2.5, 7.5, 12.5, 7.5]:
        print(f"  Duty Cycle: {dc}")
        p_clock.ChangeDutyCycle(dc)
        time.sleep(1)
        
    print("Sweeping Magnet Servo...")
    for dc in [2.5, 7.5, 12.5, 7.5]:
        print(f"  Duty Cycle: {dc}")
        p_magnet.ChangeDutyCycle(dc)
        time.sleep(1)
        
    p_clock.stop()
    p_magnet.stop()
    print("Servo test complete.")


def test_limits():
    print("\n--- Limit Switch Test ---")
    print("Monitoring switches for 10 seconds. Press them manually!")
    print("X | Y | Clock")
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            x = GPIO.input(LIMIT_X_PIN)
            y = GPIO.input(LIMIT_Y_PIN)
            c = GPIO.input(LIMIT_CLOCK_PIN)
            # 1 means pressed (Active HIGH)
            print(f"\r{x} | {y} | {c}      ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\nLimit switch test complete.")


def test_limits_interactive():
    """Interactive limit switch verification with clock confirmation."""
    print("\n" + "=" * 50)
    print("INTERACTIVE LIMIT SWITCH VERIFICATION")
    print("=" * 50)
    print("\nThis test verifies each limit switch works correctly.")
    print("Use the CLOCK button to confirm each step.")
    
    # Test X limit
    print("\n[1/3] Testing X-MIN limit switch...")
    print("      Please PRESS the X limit switch.")
    while GPIO.input(LIMIT_X_PIN) == 0:
        time.sleep(0.05)
    print("      ✓ X-MIN detected!")
    print("      Now RELEASE the switch.")
    while GPIO.input(LIMIT_X_PIN) == 1:
        time.sleep(0.05)
    print("      ✓ X-MIN released!")
    print("      Press CLOCK to confirm X limit works.")
    while GPIO.input(LIMIT_CLOCK_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.2)
    print("      ✓ X limit confirmed!")
    
    # Test Y limit
    print("\n[2/3] Testing Y-MIN limit switch...")
    print("      Please PRESS the Y limit switch.")
    while GPIO.input(LIMIT_Y_PIN) == 0:
        time.sleep(0.05)
    print("      ✓ Y-MIN detected!")
    print("      Now RELEASE the switch.")
    while GPIO.input(LIMIT_Y_PIN) == 1:
        time.sleep(0.05)
    print("      ✓ Y-MIN released!")
    print("      Press CLOCK to confirm Y limit works.")
    while GPIO.input(LIMIT_CLOCK_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.2)
    print("      ✓ Y limit confirmed!")
    
    # Test Clock limit
    print("\n[3/3] Testing CLOCK limit switch...")
    print("      Clock switch already verified through confirmations!")
    
    print("\n" + "=" * 50)
    print("✓ ALL LIMIT SWITCHES VERIFIED SUCCESSFULLY!")
    print("=" * 50)


def test_clocks():
    print("\n--- Clock Display Test ---")
    print("Initializing displays...")
    c1 = TM1637(CLOCK_1['clk'], CLOCK_1['dio'])
    c2 = TM1637(CLOCK_2['clk'], CLOCK_2['dio'])
    
    print("Displaying '8888' on both clocks...")
    c1.display_test()
    c2.display_test()
    time.sleep(3)
    print("Clock test complete.")


def speed_menu():
    """Interactive speed selection menu."""
    global current_speed
    print("\n--- Speed Selection ---")
    print(f"Current speed: {current_speed}%")
    print("\nEnter speed (0-100) or press Enter for preset:")
    print("  [1] 20% (Slow)")
    print("  [2] 40%")
    print("  [3] 50% (Default)")
    print("  [4] 60%")
    print("  [5] 80% (Fast)")
    print("  [6] 100% (Maximum)")
    
    choice = input("\nSelect option or enter value: ").strip()
    
    presets = {'1': 20, '2': 40, '3': 50, '4': 60, '5': 80, '6': 100}
    
    if choice in presets:
        set_speed(presets[choice])
    elif choice.isdigit():
        set_speed(int(choice))
    else:
        print("Invalid input. Speed unchanged.")


# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    try:
        while True:
            print("\n╔════════════════════════════════════════════╗")
            print("║   Smart Chess Board Hardware Test          ║")
            print("║   (A4988 + NEMA 11 Motors)                 ║")
            print("╠════════════════════════════════════════════╣")
            print("║   Current Speed: {:3d}%                      ║".format(current_speed))
            print("╠════════════════════════════════════════════╣")
            print("║   1. Test Stepper Motors                   ║")
            print("║   2. Test Speed Range (20% to 100%)        ║")
            print("║   3. Test Servos (Clock & Magnet)          ║")
            print("║   4. Test Limit Switches (Monitor Mode)    ║")
            print("║   5. Test Limit Switches (Interactive)     ║")
            print("║   6. Test Clock Displays                   ║")
            print("║   7. Set Motor Speed                       ║")
            print("║   8. Run All Tests                         ║")
            print("║   q. Quit                                  ║")
            print("╚════════════════════════════════════════════╝")
            
            choice = input("\nSelect an option: ").strip().lower()
            
            if choice == '1':
                test_steppers()
            elif choice == '2':
                test_speed_range()
            elif choice == '3':
                test_servos()
            elif choice == '4':
                test_limits()
            elif choice == '5':
                test_limits_interactive()
            elif choice == '6':
                test_clocks()
            elif choice == '7':
                speed_menu()
            elif choice == '8':
                test_limits_interactive()
                test_steppers()
                test_speed_range()
                test_servos()
                test_clocks()
            elif choice == 'q':
                break
            else:
                print("Invalid choice.")
                
    except KeyboardInterrupt:
        print("\nTest interrupted.")
    finally:
        GPIO.cleanup()
        print("GPIO cleaned up. Goodbye!")


if __name__ == "__main__":
    main()
