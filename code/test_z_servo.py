#!/usr/bin/env python3
"""Quick test script for the Z-axis SG90 servo on GPIO 12 (BCM)."""

import RPi.GPIO as GPIO
import time

Z_SERVO_PIN = 12
PWM_FREQ = 50  # Hz — standard for SG90

# SG90 pulse widths as duty cycles at 50 Hz (period = 20ms)
#   0°  -> 1.0ms -> 5.0%
#   90° -> 1.5ms -> 7.5%  (center / magnet raised)
# 180°  -> 2.0ms -> 10.0% (magnet lowered)
UP   = 7.5   # magnet raised
DOWN = 10.0  # magnet lowered


def angle_to_duty(degrees: float) -> float:
    """Convert 0-180° to SG90 duty cycle."""
    return 5.0 + (degrees / 180.0) * 5.0


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(Z_SERVO_PIN, GPIO.OUT)

    pwm = GPIO.PWM(Z_SERVO_PIN, PWM_FREQ)
    pwm.start(UP)
    print(f"Z-axis servo on GPIO {Z_SERVO_PIN} — started at UP position (90°)")
    time.sleep(1)

    print("\nRunning sweep: UP -> DOWN -> UP")
    print("  Moving DOWN (180°) ...")
    pwm.ChangeDutyCycle(DOWN)
    time.sleep(1.5)

    print("  Moving UP (90°) ...")
    pwm.ChangeDutyCycle(UP)
    time.sleep(1.5)

    print("\nInteractive mode — enter angle (0-180) or q to quit:")
    while True:
        try:
            raw = input("  angle> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            deg = float(raw)
            if not 0 <= deg <= 180:
                print("  Out of range (0-180)")
                continue
            duty = angle_to_duty(deg)
            print(f"  -> {deg}° (duty {duty:.2f}%)")
            pwm.ChangeDutyCycle(duty)
            time.sleep(0.5)
        except ValueError:
            print("  Enter a number or 'q'")
        except KeyboardInterrupt:
            break

    print("\nReturning to UP and cleaning up.")
    pwm.ChangeDutyCycle(UP)
    time.sleep(0.5)
    pwm.stop()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
