#!/usr/bin/env python3
"""Quick test script for the Z-axis SG90 servo on GPIO 12 (BCM).

Uses pigpio (matches src/chess_hw_interface servo_node.py and
pins.yaml's servo_node parameters) instead of RPi.GPIO. This talks to
the pigpiod daemon over a socket, so the script itself does not need
to run as root.

Requires the pigpio daemon running on the Pi (see .agent/workflows/deploy.md):
    sudo systemctl enable --now pigpiod
"""

import sys
import time

import pigpio

Z_SERVO_PIN = 12

# Calibrated positions — must match src/chess_hw_interface/config/pins.yaml
# (servo_node: engage_pulse_us / release_pulse_us)
UP = 1500   # release position — magnet clear of board
DOWN = 500  # engage position — magnet approaches board

# Wider sweep range for interactive calibration (typical SG90 full travel)
MIN_PULSE = 500
MAX_PULSE = 2500


def angle_to_pulsewidth(degrees: float) -> int:
    """Convert 0-180° to a pigpio pulse width in microseconds."""
    return int(MIN_PULSE + (degrees / 180.0) * (MAX_PULSE - MIN_PULSE))


def main():
    pi = pigpio.pi()
    if not pi.connected:
        print("Cannot connect to pigpiod. Start it with: sudo systemctl start pigpiod")
        sys.exit(1)

    try:
        pi.set_mode(Z_SERVO_PIN, pigpio.OUTPUT)
        pi.set_servo_pulsewidth(Z_SERVO_PIN, UP)
        print(f"Z-axis servo on GPIO {Z_SERVO_PIN} — started at UP position ({UP}us)")
        time.sleep(1)

        print("\nRunning sweep: UP -> DOWN -> UP")
        print(f"  Moving DOWN ({DOWN}us) ...")
        pi.set_servo_pulsewidth(Z_SERVO_PIN, DOWN)
        time.sleep(1.5)

        print(f"  Moving UP ({UP}us) ...")
        pi.set_servo_pulsewidth(Z_SERVO_PIN, UP)
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
                pulse = angle_to_pulsewidth(deg)
                print(f"  -> {deg}° (pulse {pulse}us)")
                pi.set_servo_pulsewidth(Z_SERVO_PIN, pulse)
                time.sleep(0.5)
            except ValueError:
                print("  Enter a number or 'q'")
            except KeyboardInterrupt:
                break
    finally:
        print("\nReturning to UP and cleaning up.")
        pi.set_servo_pulsewidth(Z_SERVO_PIN, UP)
        time.sleep(0.5)
        pi.set_servo_pulsewidth(Z_SERVO_PIN, 0)  # stop pulses to avoid jitter
        pi.stop()


if __name__ == "__main__":
    main()
