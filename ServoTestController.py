# Test for basic servo control

import RPi.GPIO as GPIO
import time

IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

# Coil energizing sequence
step_seq = [
    [1,0,0,1],
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1]
]

GPIO.setmode(GPIO.BCM)
pins = [IN1, IN2, IN3, IN4]

for pin in pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

try:
    while True:
        for step in step_seq:
            for pin in range(4):
                GPIO.output(pins[pin], step[pin])
            time.sleep(0.01)  # speed adjustment

except KeyboardInterrupt:
    print("Exiting...")
    GPIO.cleanup()

    print("Stopped by user")

finally:
    GPIO.cleanup()
