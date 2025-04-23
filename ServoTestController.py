# Test for basic servo control

import RPi.GPIO as GPIO
import time

# Define GPIO pins connected to IN1-IN4
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# Setup
GPIO.setmode(GPIO.BCM)
GPIO.setup(IN1, GPIO.OUT)
GPIO.setup(IN2, GPIO.OUT)
GPIO.setup(IN3, GPIO.OUT)
GPIO.setup(IN4, GPIO.OUT)

# Simple test pattern
def servo_test():
    pins = [IN1, IN2, IN3, IN4]
    for pin in pins:
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.5)
        GPIO.output(pin, GPIO.LOW)

try:
    while True:
        servo_test()

except KeyboardInterrupt:
    print("Stopped by user")

finally:
    GPIO.cleanup()
