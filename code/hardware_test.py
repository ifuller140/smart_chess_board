#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
import sys

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
# Stepper Motors (ULN2003 / 28BYJ-48)
# Labeled as "Servo 1" and "Servo 2" in user table
MOTOR_A_PINS = [2, 3, 4, 14]
MOTOR_B_PINS = [24, 23, 22, 27]

# Servos
SERVO_CLOCK_PIN = 18
SERVO_MAGNET_PIN = 17  # Provisional assumption

# Limit Switches
LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9
LIMIT_CLOCK_1_PIN = 15 # Pin 10
LIMIT_CLOCK_2_PIN = 11 # Pin 23

# Clock Displays (TM1637-style CLK, DIO)
CLOCK_1 = {'clk': 8, 'dio': 7}
CLOCK_2 = {'clk': 0, 'dio': 1}

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
    for pin in [LIMIT_X_PIN, LIMIT_Y_PIN, LIMIT_CLOCK_1_PIN, LIMIT_CLOCK_2_PIN]:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

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
    print("X | Y | Clock1 | Clock2")
    start_time = time.time()
    try:
        while time.time() - start_time < 10:
            x = GPIO.input(LIMIT_X_PIN)
            y = GPIO.input(LIMIT_Y_PIN)
            c1 = GPIO.input(LIMIT_CLOCK_1_PIN)
            c2 = GPIO.input(LIMIT_CLOCK_2_PIN)
            # 0 usually means pressed (PULL_UP)
            print(f"\r{x} | {y} | {c1}      | {c2}     ", end="")
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    print("\nLimit switch test complete.")

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
            print("3. Test Limit Switches")
            print("4. Test Clock Displays")
            print("5. Run All Tests")
            print("q. Quit")
            
            choice = input("\nSelect an option: ").strip().lower()
            
            if choice == '1':
                test_steppers()
            elif choice == '2':
                test_servos()
            elif choice == '3':
                test_limits()
            elif choice == '4':
                test_clocks()
            elif choice == '5':
                test_steppers()
                test_servos()
                test_limits()
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
