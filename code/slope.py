import RPi.GPIO as GPIO
import time

# Define GPIO pins connected to ULN2003 IN1-IN4
IN1 = 2
IN2 = 3
IN3 = 4
IN4 = 14

# Setup GPIO mode
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Setup all pins as output
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Define the half-step sequence for 28BYJ-48 stepper motor
half_step_seq = [
    [1, 0, 0, 0],
    [1, 1, 0, 0],
    [0, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 0],
    [0, 0, 1, 1],
    [0, 0, 0, 1],
    [1, 0, 0, 1]
]

half_step_back = [
    [1, 0, 0, 1],
    [0, 0, 0, 1],
    [0, 0, 1, 1],
    [0, 0, 1, 0],
    [0, 1, 1, 0],
    [0, 1, 0, 0],
    [1, 1, 0, 0],
    [1, 0, 0, 0]
]
# Full-step sequence
full_step_seq = [
    [1, 1, 0, 0],
    [0, 1, 1, 0],
    [0, 0, 1, 1],
    [1, 0, 0, 1]
]

full_step_back = [
    [0, 0, 1, 1],
    [0, 1, 1, 0],
    [1, 1, 0, 0],
    [1, 0, 0, 1]
]

fast_step_seq = [
    [1, 0, 0, 0],
    [0, 0, 0, 1]
]

# Function to perform one full cycle through the sequence
def rotate_motor(delay=0.002):
    for step in full_step_back:
        print("Step:", step)
        GPIO.output(IN1, step[0])
        GPIO.output(IN2, step[1])
        GPIO.output(IN3, step[2])
        GPIO.output(IN4, step[3])
        time.sleep(delay)

try:
    print("Stepper motor running. Press CTRL+C to stop.")
    while True:
        rotate_motor()  # Keep rotating

except KeyboardInterrupt:
    print("\nStopping motor and cleaning up GPIO.")
    GPIO.cleanup()

