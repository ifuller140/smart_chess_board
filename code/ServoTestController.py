import RPi.GPIO as GPIO
import time

# GPIO pin setup
IN1 = 17
IN2 = 18
IN3 = 27
IN4 = 22

control_pins = [IN1, IN2, IN3, IN4]
GPIO.setmode(GPIO.BCM)
for pin in control_pins:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, 0)

# Half-step sequence
half_step_seq = [
    [1,0,0,0],
    [1,1,0,0],
    [0,1,0,0],
    [0,1,1,0],
    [0,0,1,0],
    [0,0,1,1],
    [0,0,0,1],
    [1,0,0,1]
]

STEPS_PER_REV = 4096
DEGREES_PER_STEP = 360.0 / STEPS_PER_REV

# Position tracking
current_angle = 0

def cdmove_to_angle(target_angle, delay=0.001):
    global current_angle
    step_diff = int((target_angle - current_angle) / DEGREES_PER_STEP)
    
    # Decide direction
    direction = 1 if step_diff > 0 else -1
    steps_to_move = abs(step_diff)

    for _ in range(steps_to_move):
        for halfstep in (half_step_seq if direction > 0 else reversed(half_step_seq)):
            for pin in range(4):
                GPIO.output(control_pins[pin], halfstep[pin])
            time.sleep(delay)
    
    current_angle = target_angle

try:
    while True:
        print("Moving to 90°")
        move_to_angle(90)
        time.sleep(1)

        print("Moving to 180°")
        move_to_angle(180)
        time.sleep(1)

        print("Returning to 90°")
        move_to_angle(90)
        time.sleep(1)

        print("Returning to 0°")
        move_to_angle(0)
        time.sleep(1)

except KeyboardInterrupt:
    print("Interrupted by user.")

finally:
    GPIO.cleanup()
