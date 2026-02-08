#!/usr/bin/env python3
"""
Hardware Test Script for Smart Chess Board.

Supports A4988 stepper drivers with NEMA 11 motors using pigpio for
hardware-timed step pulses. Uses shared ENABLE pin for proper power management.

Features:
- Hardware-timed stepping via pigpio DMA (jitter-free)
- Proper motor ENABLE control (motors disabled when idle)
- Trapezoidal acceleration profiles
- CoreXY kinematics

Requirements:
- pigpio library: pip install pigpio
- pigpio daemon: sudo pigpiod
"""

import pigpio
import time
import sys
import signal
import atexit
import curses

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# Stepper Motors (A4988 Drivers + NEMA 11)
MOTOR_A_DIR_PIN = 27    # Direction pin for Motor A
MOTOR_A_STEP_PIN = 22   # Step pin for Motor A
MOTOR_B_DIR_PIN = 6     # Direction pin for Motor B
MOTOR_B_STEP_PIN = 5    # Step pin for Motor B
MOTOR_ENABLE_PIN = 17   # Shared ENABLE for A4988 drivers (active LOW)

# Servos
SERVO_CLOCK_PIN = 18    # Clock servo
SERVO_MAGNET_PIN = 12   # Z-axis magnet servo

# Limit Switches
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_PIN = 15

# Clock Displays (TM1637-style CLK, DIO)
CLOCK_1 = {'clk': 25, 'dio': 8}
CLOCK_2 = {'clk': 7, 'dio': 1}

# ==========================
# TIMING PARAMETERS
# ==========================
# A4988 Timing Requirements:
# - DIR setup time: 200ns minimum (we use 5µs for safety)
# - STEP pulse width: 1µs minimum (we use 5µs for stability)

DIR_SETUP_US = 5        # Microseconds to wait after setting DIR
STEP_PULSE_US = 10      # Microseconds for step pulse width (longer for driver stability)

# Speed Parameters (steps/second) - Tuned for torque
MAX_SPEED = 1200        # Maximum step rate (reduced for torque)
MIN_SPEED = 250         # Starting speed for acceleration (slower for torque buildup)
ACCEL_STEPS = 100       # Steps to accelerate/decelerate (more gradual ramp)

# Named speeds (percentage of max)
OPERATIONAL_SPEED = 90      # Normal movement
CALIBRATION_SPEED = 50      # Calibration movements
PRECISION_SPEED = 20        # Slow/precise movements

# Current speed setting (0-100)
current_speed = CALIBRATION_SPEED

# ==========================
# GLOBAL STATE
# ==========================
pi = None  # pigpio instance
motors_enabled = False

# ==========================
# SPEED CONTROL
# ==========================
def speed_to_steps_per_sec(speed_percent):
    """Convert speed percentage (0-100) to steps per second."""
    speed = max(0, min(100, speed_percent))
    return int(MIN_SPEED + (speed / 100.0) * (MAX_SPEED - MIN_SPEED))


def set_speed(speed_percent):
    """Set the current motor speed (0-100)."""
    global current_speed
    current_speed = max(0, min(100, speed_percent))
    print(f"Speed set to {current_speed}%")


# ==========================
# TM1637 DRIVER (Using pigpio)
# ==========================
class TM1637:
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
        # ACK
        pi.write(self.clk, 0)
        pi.set_mode(self.dio, pigpio.INPUT)
        pi.write(self.clk, 1)
        pi.set_mode(self.dio, pigpio.OUTPUT)

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
# MOTOR ENABLE/DISABLE
# ==========================
def motor_enable():
    """Enable A4988 drivers (allow stepping). ENABLE is active LOW."""
    global motors_enabled
    pi.write(MOTOR_ENABLE_PIN, 0)
    motors_enabled = True
    time.sleep(0.001)  # Small delay for driver stabilization


def motor_disable():
    """Disable A4988 drivers (no current, no holding torque)."""
    global motors_enabled
    pi.write(MOTOR_ENABLE_PIN, 1)
    motors_enabled = False


# ==========================
# MOTOR STOP FUNCTION
# ==========================
def stop_motors():
    """Immediately stop all motors by disabling A4988 drivers."""
    try:
        motor_disable()
        pi.write(MOTOR_A_STEP_PIN, 0)
        pi.write(MOTOR_B_STEP_PIN, 0)
    except:
        pass


# ==========================
# HARDWARE SETUP
# ==========================
def setup():
    global pi
    
    # Connect to pigpio daemon
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: Cannot connect to pigpiod daemon.")
        print("Start it with: sudo pigpiod")
        sys.exit(1)
    
    # Motor control pins
    for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN, MOTOR_ENABLE_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
    
    # Start with motors disabled
    motor_disable()
    
    # Servos
    pi.set_mode(SERVO_CLOCK_PIN, pigpio.OUTPUT)
    pi.set_mode(SERVO_MAGNET_PIN, pigpio.OUTPUT)
    
    # Limit Switches (with pull-down)
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_PIN]:
        pi.set_mode(pin, pigpio.INPUT)
        pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    
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


def cleanup():
    """Clean up pigpio and stop motors."""
    global pi
    if pi and pi.connected:
        stop_motors()
        time.sleep(0.1)  # Wait for motors to settle
        pi.stop()


# ==========================
# PIGPIO WAVE-BASED STEPPING
# ==========================
def calculate_speed_profile(total_steps, target_speed_pct):
    """
    Calculate trapezoidal speed profile for smooth acceleration.
    
    Returns list of (step_count, delay_us) tuples.
    """
    if total_steps <= 0:
        return []
    
    target_speed = speed_to_steps_per_sec(target_speed_pct)
    
    # For very short moves, use slow constant speed
    if total_steps <= ACCEL_STEPS * 2:
        delay_us = int(1_000_000 / MIN_SPEED)
        return [(total_steps, delay_us)]
    
    accel_steps = min(ACCEL_STEPS, total_steps // 3)
    decel_steps = accel_steps
    cruise_steps = total_steps - accel_steps - decel_steps
    
    profile = []
    
    # Acceleration phase
    for i in range(accel_steps):
        t = (i + 1) / accel_steps
        speed = MIN_SPEED + t * (target_speed - MIN_SPEED)
        delay_us = int(1_000_000 / speed)
        profile.append((1, delay_us))
    
    # Cruise phase
    if cruise_steps > 0:
        delay_us = int(1_000_000 / target_speed)
        profile.append((cruise_steps, delay_us))
    
    # Deceleration phase
    for i in range(decel_steps):
        t = (i + 1) / decel_steps
        speed = target_speed - t * (target_speed - MIN_SPEED)
        delay_us = int(1_000_000 / speed)
        profile.append((1, delay_us))
    
    return profile


def create_step_wave(step_a, step_b, delay_us):
    """Create a pigpio wave for stepping motors."""
    wait_us = max(1, delay_us - STEP_PULSE_US)
    
    set_mask = 0
    clear_mask = 0
    
    if step_a:
        set_mask |= (1 << MOTOR_A_STEP_PIN)
        clear_mask |= (1 << MOTOR_A_STEP_PIN)
    if step_b:
        set_mask |= (1 << MOTOR_B_STEP_PIN)
        clear_mask |= (1 << MOTOR_B_STEP_PIN)
    
    wave = [
        pigpio.pulse(set_mask, 0, STEP_PULSE_US),
        pigpio.pulse(0, clear_mask, wait_us)
    ]
    
    pi.wave_add_generic(wave)
    return pi.wave_create()


def step_both_motors(steps_a, steps_b, speed=None):
    """
    Step both motors simultaneously with Bresenham interpolation.
    Uses pigpio DMA waves for hardware-timed pulses.
    
    Args:
        steps_a: Steps for motor A (negative = reverse)
        steps_b: Steps for motor B (negative = reverse)
        speed: Speed percentage (0-100)
    """
    if speed is None:
        speed = current_speed
    
    if steps_a == 0 and steps_b == 0:
        return
    
    # Set directions
    pi.write(MOTOR_A_DIR_PIN, 1 if steps_a >= 0 else 0)
    pi.write(MOTOR_B_DIR_PIN, 1 if steps_b >= 0 else 0)
    time.sleep(DIR_SETUP_US / 1_000_000)
    
    abs_a = abs(steps_a)
    abs_b = abs(steps_b)
    max_steps = max(abs_a, abs_b)
    
    profile = calculate_speed_profile(max_steps, speed)
    
    # Enable motors
    motor_enable()
    
    try:
        err_a = 0
        err_b = 0
        
        for step_count, delay_us in profile:
            for _ in range(step_count):
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
                
                if do_step_a or do_step_b:
                    wid = create_step_wave(do_step_a, do_step_b, delay_us)
                    pi.wave_send_once(wid)
                    while pi.wave_tx_busy():
                        time.sleep(0.0001)
                    pi.wave_delete(wid)
    finally:
        # Always disable motors when done
        motor_disable()


def step_motor(step_pin, dir_pin, steps, speed=None):
    """
    Move a single motor a given number of steps.
    """
    if speed is None:
        speed = current_speed
    
    if step_pin == MOTOR_A_STEP_PIN:
        step_both_motors(steps, 0, speed)
    else:
        step_both_motors(0, steps, speed)


def move_x(steps, speed=None):
    """Move along X axis. Motor A at bottom-left, Motor B at top-right.
    For +X (right): A CW (+), B CCW (-) = OPPOSITE directions."""
    step_both_motors(steps, -steps, speed)


def move_y(steps, speed=None):
    """Move along Y axis. Motor A at bottom-left, Motor B at top-right.
    For +Y (up): A CW (+), B CW (+) = SAME direction."""
    step_both_motors(steps, steps, speed)


# ==========================
# TEST FUNCTIONS
# ==========================
def test_steppers():
    print("\n--- Stepper Motor Test (pigpio + A4988 + NEMA 11) ---")
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
    print("✓ Motors should now be silent (disabled)")


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
    
    # Set servo frequencies
    pi.set_PWM_frequency(SERVO_CLOCK_PIN, 50)
    pi.set_PWM_frequency(SERVO_MAGNET_PIN, 50)
    
    print("Sweeping Clock Servo...")
    for duty in [500, 1500, 2500, 1500]:  # pigpio uses 500-2500 for servos
        print(f"  Position: {duty}")
        pi.set_servo_pulsewidth(SERVO_CLOCK_PIN, duty)
        time.sleep(1)
    pi.set_servo_pulsewidth(SERVO_CLOCK_PIN, 0)  # Stop
    
    print("Sweeping Magnet Servo...")
    for duty in [500, 1500, 2500, 1500]:
        print(f"  Position: {duty}")
        pi.set_servo_pulsewidth(SERVO_MAGNET_PIN, duty)
        time.sleep(1)
    pi.set_servo_pulsewidth(SERVO_MAGNET_PIN, 0)  # Stop
    
    print("Servo test complete.")


def test_limits():
    print("\n--- Limit Switch Test ---")
    print("Monitoring switches for 10 seconds. Press them manually!")
    print("X | Y | Clock")
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            x = pi.read(LIMIT_X_PIN)
            y = pi.read(LIMIT_Y_PIN)
            c = pi.read(LIMIT_CLOCK_PIN)
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
    while pi.read(LIMIT_X_PIN) == 0:
        time.sleep(0.05)
    print("      ✓ X-MIN detected!")
    print("      Now RELEASE the switch.")
    while pi.read(LIMIT_X_PIN) == 1:
        time.sleep(0.05)
    print("      ✓ X-MIN released!")
    print("      Press CLOCK to confirm X limit works.")
    while pi.read(LIMIT_CLOCK_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.2)
    print("      ✓ X limit confirmed!")
    
    # Test Y limit
    print("\n[2/3] Testing Y-MIN limit switch...")
    print("      Please PRESS the Y limit switch.")
    while pi.read(LIMIT_Y_PIN) == 0:
        time.sleep(0.05)
    print("      ✓ Y-MIN detected!")
    print("      Now RELEASE the switch.")
    while pi.read(LIMIT_Y_PIN) == 1:
        time.sleep(0.05)
    print("      ✓ Y-MIN released!")
    print("      Press CLOCK to confirm Y limit works.")
    while pi.read(LIMIT_CLOCK_PIN) == 0:
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


def test_enable_disable():
    """Test motor enable/disable functionality."""
    print("\n--- Motor Enable/Disable Test ---")
    print("This tests the A4988 ENABLE pin control.")
    
    print("\n1. Motors DISABLED (should be silent, can move by hand)")
    motor_disable()
    input("   Try moving the gantry by hand. Press Enter to continue...")
    
    print("\n2. Motors ENABLED (coils energized, should resist movement)")
    motor_enable()
    input("   Try moving the gantry by hand - it should resist. Press Enter...")
    
    print("\n3. Motors DISABLED again")
    motor_disable()
    
    print("\n✓ Enable/disable test complete!")


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


def manual_gantry_control():
    """Interactive manual gantry control with curses."""
    print("\n" + "=" * 60)
    print("  MANUAL GANTRY CONTROL")
    print("="* 60)
    print("Use arrow keys to move gantry (player perspective).")
    print("Press 'q' to exit.")
    print("\nStarting in 2 seconds...")
    time.sleep(2)
    
    try:
        curses.wrapper(_gantry_control_loop)
    except Exception as e:
        print(f"\nError: {e}")
    
    print("\nManual control ended.")


def _gantry_control_loop(stdscr):
    """Curses control loop for manual gantry."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.timeout(50)
    
    pos_x = 0
    pos_y = 0
    steps_per_tick = 25
    min_steps = 5
    max_steps = 100
    
    running = True
    
    while running:
        stdscr.clear()
        
        # Header
        stdscr.addstr(0, 0, "=" * 60)
        stdscr.addstr(1, 0, "  MANUAL GANTRY CONTROL - Player Perspective")
        stdscr.addstr(2, 0, "=" * 60)
        
        # Position and settings
        stdscr.addstr(4, 0, f"Position: X={pos_x:6d}  Y={pos_y:6d} steps")
        stdscr.addstr(5, 0, f"Step size: {steps_per_tick} steps/tick")
        x_lim = pi.read(LIMIT_X_PIN)
        y_lim = pi.read(LIMIT_Y_PIN)
        stdscr.addstr(6, 0, f"Limits: X={x_lim}  Y={y_lim}")
        
        # Motor layout diagram
        stdscr.addstr(8, 0, "Motor Layout (from player's view):")
        stdscr.addstr(9, 0, "                     +Y (UP)")
        stdscr.addstr(10, 0, "                        |")
        stdscr.addstr(11, 0, "    Motor B ----------- + ----------- ")
        stdscr.addstr(12, 0, "   (top-right)      -X --+-- +X (RIGHT)")
        stdscr.addstr(13, 0, "                         |")
        stdscr.addstr(14, 0, "    Motor A ----------- + -----------")
        stdscr.addstr(15, 0, "  (bottom-left)        -Y (DOWN)")
        
        # Direction chart
        stdscr.addstr(17, 0, "Direction Chart:")
        stdscr.addstr(18, 0, "+--------+-----------------+-----------------+")
        stdscr.addstr(19, 0, "| Key    | Motor A (BL)    | Motor B (TR)    |")
        stdscr.addstr(20, 0, "+--------+-----------------+-----------------+")
        stdscr.addstr(21, 0, "| RIGHT  | Clockwise (+)   | Counter-CW (-)  |")
        stdscr.addstr(22, 0, "| LEFT   | Counter-CW (-)  | Clockwise (+)   |")
        stdscr.addstr(23, 0, "| UP     | Clockwise (+)   | Clockwise (+)   |")
        stdscr.addstr(24, 0, "| DOWN   | Counter-CW (-)  | Counter-CW (-)  |")
        stdscr.addstr(25, 0, "+--------+-----------------+-----------------+")
        
        # Controls
        stdscr.addstr(27, 0, "Controls: Arrow keys=Move  +/-=Step size  q=Quit")
        
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
            # +X: Motor A CW, Motor B CCW
            move_x(steps_per_tick)
            pos_x += steps_per_tick
        elif key == curses.KEY_LEFT:
            if pi.read(LIMIT_X_PIN) == 0:
                move_x(-steps_per_tick)
                pos_x -= steps_per_tick
        elif key == curses.KEY_UP:
            # +Y: Both motors CW
            move_y(steps_per_tick)
            pos_y += steps_per_tick
        elif key == curses.KEY_DOWN:
            if pi.read(LIMIT_Y_PIN) == 0:
                move_y(-steps_per_tick)
                pos_y -= steps_per_tick


# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    try:
        while True:
            print("\n╔════════════════════════════════════════════╗")
            print("║   Smart Chess Board Hardware Test          ║")
            print("║   (pigpio + A4988 + NEMA 11 Motors)        ║")
            print("╠════════════════════════════════════════════╣")
            print("║   Current Speed: {:3d}%                      ║".format(current_speed))
            print("║   Motors: {}                       ║".format("ENABLED " if motors_enabled else "DISABLED"))
            print("╠════════════════════════════════════════════╣")
            print("║   1. Test Stepper Motors                   ║")
            print("║   2. Test Speed Range (20% to 100%)        ║")
            print("║   3. Test Servos (Clock & Magnet)          ║")
            print("║   4. Test Limit Switches (Monitor Mode)    ║")
            print("║   5. Test Limit Switches (Interactive)     ║")
            print("║   6. Test Clock Displays                   ║")
            print("║   7. Set Motor Speed                       ║")
            print("║   8. Test Enable/Disable                   ║")
            print("║   9. Run All Tests                         ║")
            print("║  10. Interactive Stepper Test              ║")
            print("║  11. Manual Gantry Control (Arrow Keys)    ║")
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
                test_enable_disable()
            elif choice == '9':
                test_limits_interactive()
                test_steppers()
                test_speed_range()
                test_servos()
                test_clocks()
            elif choice == '10':
                print("\nLaunching interactive stepper test...")
                print("Run: python3 stepper_interactive_test.py")
                import subprocess
                subprocess.run(['python3', 'stepper_interactive_test.py'])
            elif choice == '11':
                manual_gantry_control()
            elif choice == 'q':
                break
            else:
                print("Invalid choice.")
                
    except KeyboardInterrupt:
        print("\nTest interrupted.")
    finally:
        print("\n🛑 Stopping motors...")
        motor_disable()
        time.sleep(0.1)
        cleanup()
        print("✅ Motors disabled. GPIO cleaned up. Goodbye!")


if __name__ == "__main__":
    main()
