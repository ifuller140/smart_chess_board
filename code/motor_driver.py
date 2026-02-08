#!/usr/bin/env python3
"""
Motor Driver for A4988 + NEMA 11 using pigpio DMA waves.

This module provides hardware-timed step pulses using the Pi's DMA engine,
ensuring microsecond-accurate timing immune to Linux scheduling jitter.

Features:
- Hardware-timed step pulses (microsecond accuracy via DMA)
- Trapezoidal acceleration/deceleration ramps
- ENABLE pin control for proper power management
- CoreXY kinematics for gantry movement
- Position tracking (odometry)

Usage:
    from motor_driver import MotorDriver
    
    driver = MotorDriver()
    driver.move_x(1000)   # Move 1000 steps in +X
    driver.move_y(-500)   # Move 500 steps in -Y
    driver.move_to(2000, 1500)  # Move to absolute position
    driver.cleanup()

Requirements:
    - pigpio library: pip install pigpio
    - pigpio daemon running: sudo pigpiod

Author: Smart Chess Board Project
"""

import pigpio
import time
import math
from typing import Tuple, Optional

# ==========================
# GPIO PIN DEFINITIONS (BCM)
# ==========================
MOTOR_A_STEP_PIN = 22   # Step pulse for Motor A
MOTOR_A_DIR_PIN = 27    # Direction for Motor A
MOTOR_B_STEP_PIN = 5    # Step pulse for Motor B
MOTOR_B_DIR_PIN = 6     # Direction for Motor B
MOTOR_ENABLE_PIN = 17   # Shared ENABLE for both A4988 drivers (active LOW)

# ==========================
# TIMING PARAMETERS (microseconds)
# ==========================
STEP_PULSE_US = 10      # Step pulse width (longer for driver stability)
DIR_SETUP_US = 5        # Direction setup time before stepping (A4988 min: 200 ns)

# ==========================
# SPEED PARAMETERS (steps/second)
# Tuned for torque: slower start, more gradual acceleration
# ==========================
MAX_SPEED = 1200        # Maximum step rate (reduced for torque)
MIN_SPEED = 250         # Starting/ending speed for acceleration ramp (slower for torque buildup)
ACCEL_STEPS = 100       # Number of steps to accelerate/decelerate (more gradual ramp)

# ==========================
# MOTOR DRIVER CLASS
# ==========================
class MotorDriver:
    """
    Hardware-timed stepper motor driver using pigpio DMA waves.
    
    Provides CoreXY kinematics with trapezoidal velocity profiles
    and proper ENABLE pin control for power management.
    """
    
    def __init__(self, auto_connect: bool = True):
        """
        Initialize the motor driver.
        
        Args:
            auto_connect: If True, connect to pigpiod immediately
        """
        self.pi = None
        self.connected = False
        
        # Position tracking (steps from origin)
        self.pos_x = 0
        self.pos_y = 0
        
        # Motor state
        self._motors_enabled = False
        
        if auto_connect:
            self.connect()
    
    def connect(self) -> bool:
        """
        Connect to the pigpio daemon and setup GPIO pins.
        
        Returns:
            True if connected successfully
            
        Raises:
            RuntimeError if pigpiod is not running
        """
        self.pi = pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError(
                "Cannot connect to pigpiod daemon. "
                "Start it with: sudo pigpiod"
            )
        
        self.connected = True
        
        # Setup all motor control pins as outputs
        for pin in [MOTOR_A_STEP_PIN, MOTOR_A_DIR_PIN, 
                    MOTOR_B_STEP_PIN, MOTOR_B_DIR_PIN, 
                    MOTOR_ENABLE_PIN]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)
        
        # Start with motors disabled (ENABLE is active LOW)
        self.disable()
        
        return True
    
    def enable(self):
        """
        Enable A4988 drivers (allow stepping).
        
        ENABLE is active LOW: LOW = motors powered
        """
        if not self.connected:
            return
        self.pi.write(MOTOR_ENABLE_PIN, 0)
        self._motors_enabled = True
        # Small delay to let driver stabilize
        time.sleep(0.001)
    
    def disable(self):
        """
        Disable A4988 drivers (no current, no holding torque).
        
        ENABLE is active LOW: HIGH = motors disabled
        
        For a horizontal chessboard gantry, this is safe because:
        - No gravity load on the gantry
        - Belts provide passive position holding
        - Significantly reduces power consumption and heat
        """
        if not self.connected:
            return
        self.pi.write(MOTOR_ENABLE_PIN, 1)
        self._motors_enabled = False
    
    @property
    def is_enabled(self) -> bool:
        """Check if motors are currently enabled."""
        return self._motors_enabled
    
    def set_direction(self, dir_a: int, dir_b: int):
        """
        Set motor directions.
        
        Args:
            dir_a: Direction for motor A (1 = forward, -1 = reverse)
            dir_b: Direction for motor B (1 = forward, -1 = reverse)
        """
        self.pi.write(MOTOR_A_DIR_PIN, 1 if dir_a >= 0 else 0)
        self.pi.write(MOTOR_B_DIR_PIN, 1 if dir_b >= 0 else 0)
        # Wait for direction to stabilize
        time.sleep(DIR_SETUP_US / 1_000_000)
    
    def _calculate_speed_profile(self, total_steps: int) -> list:
        """
        Calculate trapezoidal speed profile for smooth acceleration.
        
        Args:
            total_steps: Total number of steps to take
            
        Returns:
            List of (step_count, delay_us) tuples for each phase
        """
        if total_steps <= 0:
            return []
        
        # For very short moves, just use minimum speed
        if total_steps <= ACCEL_STEPS * 2:
            delay_us = int(1_000_000 / MIN_SPEED)
            return [(total_steps, delay_us)]
        
        # Calculate acceleration profile
        accel_steps = min(ACCEL_STEPS, total_steps // 3)
        decel_steps = accel_steps
        cruise_steps = total_steps - accel_steps - decel_steps
        
        profile = []
        
        # Acceleration phase - linearly increase speed
        for i in range(accel_steps):
            # Linear interpolation from MIN_SPEED to MAX_SPEED
            t = (i + 1) / accel_steps
            speed = MIN_SPEED + t * (MAX_SPEED - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))
        
        # Cruise phase - constant max speed
        if cruise_steps > 0:
            delay_us = int(1_000_000 / MAX_SPEED)
            profile.append((cruise_steps, delay_us))
        
        # Deceleration phase - linearly decrease speed
        for i in range(decel_steps):
            # Linear interpolation from MAX_SPEED to MIN_SPEED
            t = (i + 1) / decel_steps
            speed = MAX_SPEED - t * (MAX_SPEED - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))
        
        return profile
    
    def _create_step_wave(self, step_pin: int, delay_us: int) -> int:
        """
        Create a pigpio wave for a single step pulse.
        
        Args:
            step_pin: GPIO pin for STEP signal
            delay_us: Total delay for this step (pulse + wait)
            
        Returns:
            Wave ID
        """
        # Pulse HIGH for STEP_PULSE_US, then LOW for remaining time
        wait_us = max(1, delay_us - STEP_PULSE_US)
        
        wave = [
            pigpio.pulse(1 << step_pin, 0, STEP_PULSE_US),      # Set HIGH
            pigpio.pulse(0, 1 << step_pin, wait_us)              # Set LOW
        ]
        
        self.pi.wave_add_generic(wave)
        return self.pi.wave_create()
    
    def _create_dual_step_wave(self, step_a: bool, step_b: bool, delay_us: int) -> int:
        """
        Create a pigpio wave for stepping both motors simultaneously.
        
        Args:
            step_a: Whether to step motor A
            step_b: Whether to step motor B
            delay_us: Total delay for this step
            
        Returns:
            Wave ID
        """
        wait_us = max(1, delay_us - STEP_PULSE_US)
        
        # Build bitmasks for which pins to set/clear
        set_mask = 0
        clear_mask = 0
        
        if step_a:
            set_mask |= (1 << MOTOR_A_STEP_PIN)
            clear_mask |= (1 << MOTOR_A_STEP_PIN)
        if step_b:
            set_mask |= (1 << MOTOR_B_STEP_PIN)
            clear_mask |= (1 << MOTOR_B_STEP_PIN)
        
        wave = [
            pigpio.pulse(set_mask, 0, STEP_PULSE_US),    # Set pins HIGH
            pigpio.pulse(0, clear_mask, wait_us)          # Set pins LOW
        ]
        
        self.pi.wave_add_generic(wave)
        return self.pi.wave_create()
    
    def step_motors(self, steps_a: int, steps_b: int, 
                    max_speed: int = None, min_speed: int = None):
        """
        Step both motors with Bresenham interpolation and acceleration.
        
        Uses pigpio DMA waves for hardware-timed pulses.
        Motors are automatically enabled before and disabled after stepping.
        
        Args:
            steps_a: Steps for motor A (negative = reverse direction)
            steps_b: Steps for motor B (negative = reverse direction)
            max_speed: Override max speed (steps/sec)
            min_speed: Override min speed (steps/sec)
        """
        if steps_a == 0 and steps_b == 0:
            return
        
        if not self.connected:
            raise RuntimeError("Not connected to pigpiod")
        
        # Use provided speeds or defaults
        _max_speed = max_speed if max_speed else MAX_SPEED
        _min_speed = min_speed if min_speed else MIN_SPEED
        
        # Set directions
        self.set_direction(steps_a, steps_b)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        # Calculate speed profile for the longer axis
        profile = self._calculate_speed_profile(max_steps)
        
        # Enable motors
        self.enable()
        
        try:
            # Bresenham error accumulators
            err_a = 0
            err_b = 0
            step_idx = 0
            
            for step_count, delay_us in profile:
                for _ in range(step_count):
                    # Determine which motors should step
                    do_step_a = False
                    do_step_b = False
                    
                    err_a += abs_a
                    if err_a >= max_steps:
                        err_a -= max_steps
                        do_step_a = True
                    
                    err_b += abs_b
                    if err_b >= max_steps:
                        err_b -= max_steps
                        do_step_b = True
                    
                    if do_step_a or do_step_b:
                        # Create and transmit wave
                        wid = self._create_dual_step_wave(do_step_a, do_step_b, delay_us)
                        self.pi.wave_send_once(wid)
                        
                        # Wait for wave to complete
                        while self.pi.wave_tx_busy():
                            time.sleep(0.0001)
                        
                        # Clean up wave
                        self.pi.wave_delete(wid)
                    
                    step_idx += 1
        
        finally:
            # Always disable motors when done
            self.disable()
    
    def move_x(self, steps: int, speed: int = None):
        """
        Move along X axis using CoreXY kinematics.
        
        Physical layout: Motor A at bottom-left, Motor B at top-right.
        For +X (right): A CW (+), B CCW (-) = OPPOSITE directions.
        
        Args:
            steps: Steps to move (positive = right, negative = left)
            speed: Optional speed override
        """
        self.step_motors(steps, -steps, max_speed=speed)
        self.pos_x += steps
    
    def move_y(self, steps: int, speed: int = None):
        """
        Move along Y axis using CoreXY kinematics.
        
        Physical layout: Motor A at bottom-left, Motor B at top-right.
        For +Y (up): A CW (+), B CW (+) = SAME direction.
        
        Args:
            steps: Steps to move (positive = up, negative = down)
            speed: Optional speed override
        """
        self.step_motors(steps, steps, max_speed=speed)
        self.pos_y += steps
    
    def move_to(self, target_x: int, target_y: int, speed: int = None) -> bool:
        """
        Move to an absolute position.
        
        Args:
            target_x: Target X position in steps
            target_y: Target Y position in steps
            speed: Optional speed override
            
        Returns:
            True if move completed successfully
        """
        dx = target_x - self.pos_x
        dy = target_y - self.pos_y
        
        if dx == 0 and dy == 0:
            return True
        
        # CoreXY kinematics conversion
        # For our motor layout (A at bottom-left, B at top-right):
        # +X: A+, B- (opposite directions)
        # +Y: A+, B+ (same direction)
        # Combined: steps_a = dx + dy, steps_b = -dx + dy
        steps_a = dx + dy
        steps_b = -dx + dy
        
        self.step_motors(steps_a, steps_b, max_speed=speed)
        
        self.pos_x = target_x
        self.pos_y = target_y
        
        return True
    
    def move_relative(self, dx: int, dy: int, speed: int = None) -> bool:
        """
        Move relative to current position.
        
        Args:
            dx: X displacement in steps
            dy: Y displacement in steps
            speed: Optional speed override
            
        Returns:
            True if move completed successfully
        """
        return self.move_to(self.pos_x + dx, self.pos_y + dy, speed)
    
    def set_position(self, x: int, y: int):
        """
        Set the current position without moving.
        
        Useful after homing to reset the origin.
        
        Args:
            x: New X position
            y: New Y position
        """
        self.pos_x = x
        self.pos_y = y
    
    def get_position(self) -> Tuple[int, int]:
        """
        Get current position.
        
        Returns:
            (x, y) position in steps
        """
        return (self.pos_x, self.pos_y)
    
    def stop(self):
        """
        Emergency stop - immediately disable motors.
        
        Note: This does not stop an in-progress wave, but waves
        are short enough that this provides reasonable stopping.
        """
        self.disable()
    
    def cleanup(self):
        """
        Clean shutdown of the motor driver.
        
        Call this before exiting to properly release resources.
        """
        if self.connected and self.pi:
            # Ensure motors are disabled
            self.disable()
            time.sleep(0.1)  # Brief delay for driver to settle
            
            # Clear any pending waves
            try:
                self.pi.wave_tx_stop()
                self.pi.wave_clear()
            except:
                pass
            
            # Disconnect from pigpio
            self.pi.stop()
            self.connected = False


# ==========================
# STANDALONE TEST
# ==========================
def test_driver():
    """Quick test of the motor driver."""
    print("Motor Driver Test")
    print("=" * 40)
    
    try:
        driver = MotorDriver()
        print("✓ Connected to pigpiod")
        
        print("\nTest 1: Enable/Disable")
        driver.enable()
        print(f"  Enabled: {driver.is_enabled}")
        time.sleep(0.5)
        driver.disable()
        print(f"  Disabled: {driver.is_enabled}")
        
        print("\nTest 2: Move X (200 steps right, then left)")
        driver.move_x(200)
        print(f"  Position: {driver.get_position()}")
        time.sleep(0.3)
        driver.move_x(-200)
        print(f"  Position: {driver.get_position()}")
        
        print("\nTest 3: Move Y (200 steps up, then down)")
        driver.move_y(200)
        print(f"  Position: {driver.get_position()}")
        time.sleep(0.3)
        driver.move_y(-200)
        print(f"  Position: {driver.get_position()}")
        
        print("\nTest 4: Diagonal move")
        driver.move_to(100, 100)
        print(f"  Position: {driver.get_position()}")
        time.sleep(0.3)
        driver.move_to(0, 0)
        print(f"  Position: {driver.get_position()}")
        
        print("\n✓ All tests passed!")
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
    
    finally:
        if 'driver' in dir():
            driver.cleanup()
            print("\n✓ Cleanup complete")


if __name__ == "__main__":
    test_driver()
