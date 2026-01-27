#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# Stepper Motors (ULN2003 / 28BYJ-48)
MOTOR_A_PINS = [14, 4, 3, 2]   # IN1, IN2, IN3, IN4
MOTOR_B_PINS = [24, 23, 22, 27] # IN1, IN2, IN3, IN4

# Servos
SERVO_CLOCK_PIN = 16
SERVO_MAGNET_PIN = 12

# Limit Switches
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_1_PIN = 15 # Clock Limit Switch
LIMIT_CLOCK_2_PIN = 15 # Redundant, ensuring single variable availability

# Clock Displays (TM1637-style CLK, DIO)
CLOCK_1 = {'clk': 25, 'dio': 8}
CLOCK_2 = {'clk': 7, 'dio': 1}

# ==========================
# STEPPER SEQUENCE
# ==========================
# Full-step sequence for 28BYJ-48
SEQ = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]

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
        self._write_byte(0x40) # Data command
        self._stop()
        self._start()
        self._write_byte(0xC0) # Address command
        for _ in range(4):
            self._write_byte(0x7F) # All segments on (except colon)
        self._stop()
        self._start()
        self._write_byte(0x8F) # Display control (on, max brightness)
        self._stop()

# ==========================
# HARDWARE SETUP
# ==========================
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Steppers
    for pin in MOTOR_A_PINS + MOTOR_B_PINS:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, 0)
        
    # Servos
    GPIO.setup(SERVO_CLOCK_PIN, GPIO.OUT)
    GPIO.setup(SERVO_MAGNET_PIN, GPIO.OUT)
    
    # Limit Switches
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_1_PIN]:
        # Active HIGH: 1=Pressed (VCC), 0=Released (GND)
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# ==========================
# TEST FUNCTIONS
# ==========================
def test_steppers():
    print("\n--- Stepper Motor Test ---")
    print("Moving X-axis (both motors same direction)...")
    move_steppers(1, 1, 100)
    time.sleep(0.5)
    move_steppers(-1, -1, 100)
    
    print("Moving Y-axis (motors opposite directions)...")
    move_steppers(1, -1, 100)
    time.sleep(0.5)
    move_steppers(-1, 1, 100)
    print("Stepper test complete.")

def move_steppers(dirA, dirB, steps, delay=0.002):
    for _ in range(steps):
        for step in range(len(SEQ)):
            for pin in range(4):
                if dirA != 0:
                    GPIO.output(MOTOR_A_PINS[pin], SEQ[step][pin] if dirA > 0 else SEQ[-step-1][pin])
                if dirB != 0:
                    GPIO.output(MOTOR_B_PINS[pin], SEQ[step][pin] if dirB > 0 else SEQ[-step-1][pin])
            time.sleep(delay)

def test_servos():
    print("\n--- Servo Test ---")
    p_clock = GPIO.PWM(SERVO_CLOCK_PIN, 50) # 50Hz
    p_magnet = GPIO.PWM(SERVO_MAGNET_PIN, 50)
    
    p_clock.start(7.5) # 90 degrees
    p_magnet.start(7.5)
    
    print("Sweeping Clock Servo (Seesaw)...")
    for dc in [2.5, 7.5, 12.5, 7.5]: # 0, 90, 180, 90
        print(f"Duty Cycle: {dc}")
        p_clock.ChangeDutyCycle(dc)
        time.sleep(1)
        
    print("Sweeping Magnet Servo...")
    for dc in [2.5, 7.5, 12.5, 7.5]:
        print(f"Duty Cycle: {dc}")
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
            c1 = GPIO.input(LIMIT_CLOCK_1_PIN)
            # 1 means pressed (Active HIGH)
            print(f"\r{x} | {y} | {c1}      ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\nLimit switch test complete.")

def test_limits_interactive():
    """Interactive limit switch verification with clock confirmation."""
    print("\n" + "="*50)
    print("INTERACTIVE LIMIT SWITCH VERIFICATION")
    print("="*50)
    print("\nThis test verifies each limit switch works correctly.")
    print("Use the CLOCK button to confirm each step.")
    
    # Test X limit
    print("\n[1/3] Testing X-MIN limit switch...")
    print("      Please PRESS the X limit switch.")
    while GPIO.input(LIMIT_X_PIN) == 0:  # Wait for press (active HIGH logic: 0->1)
        time.sleep(0.05)
    print("      ✓ X-MIN detected!")
    print("      Now RELEASE the switch.")
    while GPIO.input(LIMIT_X_PIN) == 1:  # Wait for release (1->0)
        time.sleep(0.05)
    print("      ✓ X-MIN released!")
    print("      Press CLOCK to confirm X limit works.")
    while GPIO.input(LIMIT_CLOCK_1_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.2)  # Debounce
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
    while GPIO.input(LIMIT_CLOCK_1_PIN) == 0:
        time.sleep(0.05)
    time.sleep(0.2)
    print("      ✓ Y limit confirmed!")
    
    # Test Clock limit
    print("\n[3/3] Testing CLOCK limit switch...")
    print("      Clock switch already verified through confirmations!")
    
    print("\n" + "="*50)
    print("✓ ALL LIMIT SWITCHES VERIFIED SUCCESSFULLY!")
    print("="*50)

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

# ==========================
# MAIN MENU
# ==========================
def main():
    setup()
    try:
        while True:
            print("\n=== Smart Chess Board Hardware Test ===")
            print("1. Test Stepper Motors")
            print("2. Test Servos (Clock & Magnet)")
            print("3. Test Limit Switches (Monitor Mode)")
            print("4. Test Limit Switches (Interactive Verification)")
            print("5. Test Clock Displays")
            print("6. Run All Tests")
            print("q. Quit")
            
            choice = input("\nSelect an option: ").strip().lower()
            
            if choice == '1':
                test_steppers()
            elif choice == '2':
                test_servos()
            elif choice == '3':
                test_limits()
            elif choice == '4':
                test_limits_interactive()
            elif choice == '5':
                test_clocks()
            elif choice == '6':
                test_limits_interactive()
                test_steppers()
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
