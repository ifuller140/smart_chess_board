# Test for basic servo control

import RPi.GPIO as GPIO
import time

# Define GPIO pins for IN1–IN4
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Full-step sequence for 28BYJ-48 via ULN2003
step_seq = [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]

pins = [IN1, IN2, IN3, IN4]

try:
    while True:
        for step in step_seq:
            for pin in range(4):
                GPIO.output(pins[pin], step[pin])
            time.sleep(0.01)  # adjust speed here

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
