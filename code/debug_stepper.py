#!/usr/bin/env python3
"""
Minimal pigpio stepper test - Diagnostic script.

This script bypasses the test framework to isolate motor behavior.
Run directly: sudo python3 debug_stepper.py

Tests:
1. Single motor, constant speed (no interpolation)
2. Batch wave generation vs per-step wave creation
3. Timing measurement
"""

import pigpio
import time
import sys

# Pin definitions
MOTOR_A_DIR_PIN = 27
MOTOR_A_STEP_PIN = 22
MOTOR_B_DIR_PIN = 6
MOTOR_B_STEP_PIN = 5
MOTOR_ENABLE_PIN = 17

# Timing
STEP_PULSE_US = 10
DIR_SETUP_US = 5


def test_single_motor_simple(pi, steps=200, delay_us=2000):
    """
    Test 1: Simple single-wave-per-step approach.
    This is what the current pigpio_stepper.py does.
    """
    print(f"\n[Test 1] Single wave per step: {steps} steps @ {delay_us}us delay")
    
    # Enable motor
    pi.write(MOTOR_ENABLE_PIN, 0)
    pi.write(MOTOR_A_DIR_PIN, 1)
    time.sleep(DIR_SETUP_US / 1_000_000)
    
    start = time.perf_counter()
    
    for _ in range(steps):
        wait_us = max(1, delay_us - STEP_PULSE_US)
        wave = [
            pigpio.pulse(1 << MOTOR_A_STEP_PIN, 0, STEP_PULSE_US),
            pigpio.pulse(0, 1 << MOTOR_A_STEP_PIN, wait_us)
        ]
        pi.wave_add_generic(wave)
        wid = pi.wave_create()
        pi.wave_send_once(wid)
        while pi.wave_tx_busy():
            time.sleep(0.0001)
        pi.wave_delete(wid)
    
    elapsed = time.perf_counter() - start
    actual_freq = steps / elapsed
    
    pi.write(MOTOR_ENABLE_PIN, 1)
    print(f"    Elapsed: {elapsed:.3f}s, Actual freq: {actual_freq:.1f} steps/sec")
    print(f"    Expected freq: {1_000_000 / delay_us:.1f} steps/sec")
    return elapsed


def test_single_motor_batched(pi, steps=200, delay_us=2000):
    """
    Test 2: Batch wave creation - create ALL pulses in one wave.
    This is much more efficient.
    """
    print(f"\n[Test 2] Batched wave: {steps} steps @ {delay_us}us delay")
    
    pi.write(MOTOR_ENABLE_PIN, 0)
    pi.write(MOTOR_A_DIR_PIN, 1)
    time.sleep(DIR_SETUP_US / 1_000_000)
    
    # Create one big wave with all pulses
    wait_us = max(1, delay_us - STEP_PULSE_US)
    for _ in range(steps):
        pi.wave_add_generic([
            pigpio.pulse(1 << MOTOR_A_STEP_PIN, 0, STEP_PULSE_US),
            pigpio.pulse(0, 1 << MOTOR_A_STEP_PIN, wait_us)
        ])
    
    wid = pi.wave_create()
    
    start = time.perf_counter()
    pi.wave_send_once(wid)
    while pi.wave_tx_busy():
        time.sleep(0.001)
    elapsed = time.perf_counter() - start
    
    pi.wave_delete(wid)
    pi.write(MOTOR_ENABLE_PIN, 1)
    
    actual_freq = steps / elapsed
    print(f"    Elapsed: {elapsed:.3f}s, Actual freq: {actual_freq:.1f} steps/sec")
    print(f"    Expected freq: {1_000_000 / delay_us:.1f} steps/sec")
    return elapsed


def test_python_loop(pi, steps=200, delay_us=2000):
    """
    Test 3: Pure Python timing (no pigpio waves) for comparison.
    """
    print(f"\n[Test 3] Python time.sleep(): {steps} steps @ {delay_us}us delay")
    
    pi.write(MOTOR_ENABLE_PIN, 0)
    pi.write(MOTOR_A_DIR_PIN, 1)
    time.sleep(DIR_SETUP_US / 1_000_000)
    
    start = time.perf_counter()
    
    for _ in range(steps):
        pi.write(MOTOR_A_STEP_PIN, 1)
        time.sleep(STEP_PULSE_US / 1_000_000)
        pi.write(MOTOR_A_STEP_PIN, 0)
        time.sleep(delay_us / 1_000_000)
    
    elapsed = time.perf_counter() - start
    actual_freq = steps / elapsed
    
    pi.write(MOTOR_ENABLE_PIN, 1)
    print(f"    Elapsed: {elapsed:.3f}s, Actual freq: {actual_freq:.1f} steps/sec")
    print(f"    Expected freq: {1_000_000 / delay_us:.1f} steps/sec")
    return elapsed


def main():
    print("=" * 60)
    print("MINIMAL PIGPIO STEPPER DIAGNOSTIC")
    print("=" * 60)
    
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: Cannot connect to pigpiod. Run: sudo pigpiod")
        sys.exit(1)
    
    # Setup pins
    for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, MOTOR_B_DIR_PIN, 
                MOTOR_B_STEP_PIN, MOTOR_ENABLE_PIN]:
        pi.set_mode(pin, pigpio.OUTPUT)
        pi.write(pin, 0)
    
    # Disable motors initially
    pi.write(MOTOR_ENABLE_PIN, 1)
    
    try:
        print("\nThis will move Motor A. Ensure gantry is clear!")
        input("Press Enter to continue...")
        
        # Test 1: Current approach (slow)
        t1 = test_single_motor_simple(pi, steps=200, delay_us=2000)
        time.sleep(0.5)
        
        # Test 2: Batched waves (fast)
        t2 = test_single_motor_batched(pi, steps=200, delay_us=2000)
        time.sleep(0.5)
        
        # Test 3: Python loop (jittery)
        t3 = test_python_loop(pi, steps=200, delay_us=2000)
        
        print("\n" + "=" * 60)
        print("RESULTS SUMMARY")
        print("=" * 60)
        print(f"Test 1 (single wave/step): {t1:.3f}s")
        print(f"Test 2 (batched waves):    {t2:.3f}s")
        print(f"Test 3 (Python loop):      {t3:.3f}s")
        print()
        print("If Test 1 >> Test 2, the per-step wave creation is the bottleneck.")
        print("If motors jitter in Test 2 but not Test 1, it's a timing issue.")
        
    except KeyboardInterrupt:
        print("\nInterrupted!")
    finally:
        pi.write(MOTOR_ENABLE_PIN, 1)
        pi.stop()
        print("\nMotors disabled. Done.")


if __name__ == "__main__":
    main()
