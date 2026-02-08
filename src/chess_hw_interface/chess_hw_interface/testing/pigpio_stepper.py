#!/usr/bin/env python3
"""
Pigpio-based stepper motor control for hardware tests.

Uses DMA waves for jitter-free, hardware-timed step pulses.
This module provides reliable motor control for the gantry test suite.

Requirements:
- pigpio library: pip install pigpio
- pigpio daemon running: sudo pigpiod
"""

import time
from typing import List, Tuple, Optional

import pigpio


# ==========================
# PIN DEFINITIONS (BCM)
# ==========================
MOTOR_A_DIR_PIN = 27
MOTOR_A_STEP_PIN = 22
MOTOR_B_DIR_PIN = 6
MOTOR_B_STEP_PIN = 5
MOTOR_ENABLE_PIN = 17  # A4988 ENABLE, active LOW

LIMIT_X_PIN = 10
LIMIT_Y_PIN = 9

# ==========================
# TIMING PARAMETERS
# ==========================
DIR_SETUP_US = 5        # Microseconds to wait after setting DIR
STEP_PULSE_US = 10      # Step pulse width in microseconds

# Speed parameters (steps per second)
MAX_SPEED = 1200        # Maximum step rate
MIN_SPEED = 250         # Starting speed for acceleration
ACCEL_STEPS = 100       # Steps for acceleration/deceleration ramp


class PigpioStepper:
    """
    Pigpio-based stepper motor controller with DMA wave generation.
    
    Provides jitter-free stepping for CoreXY gantry systems.
    """
    
    def __init__(self, pi: Optional[pigpio.pi] = None):
        """
        Initialize stepper controller.
        
        Args:
            pi: Existing pigpio.pi instance, or None to create new connection.
        """
        self._owns_pi = pi is None
        self.pi = pi if pi else pigpio.pi()
        
        if not self.pi.connected:
            raise RuntimeError(
                "Cannot connect to pigpiod daemon. "
                "Start it with: sudo pigpiod"
            )
        
        self._motors_enabled = False
        self._pos_x = 0
        self._pos_y = 0
        self._current_speed = 50  # Default speed percentage
        
        self._setup_pins()
    
    def _setup_pins(self):
        """Configure GPIO pins for motor control."""
        for pin in [MOTOR_A_DIR_PIN, MOTOR_A_STEP_PIN, 
                    MOTOR_B_DIR_PIN, MOTOR_B_STEP_PIN, MOTOR_ENABLE_PIN]:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, 0)
        
        # Start with motors disabled
        self.motor_disable()
        
        # Limit switches with pull-down (active HIGH)
        for pin in [LIMIT_X_PIN, LIMIT_Y_PIN]:
            self.pi.set_mode(pin, pigpio.INPUT)
            self.pi.set_pull_up_down(pin, pigpio.PUD_DOWN)
    
    def motor_enable(self):
        """Enable A4988 drivers (active LOW)."""
        self.pi.write(MOTOR_ENABLE_PIN, 0)
        self._motors_enabled = True
        time.sleep(0.001)
    
    def motor_disable(self):
        """Disable A4988 drivers."""
        self.pi.write(MOTOR_ENABLE_PIN, 1)
        self._motors_enabled = False
    
    def read_x_limit(self) -> bool:
        """Read X limit switch (active HIGH)."""
        return self.pi.read(LIMIT_X_PIN) == 1
    
    def read_y_limit(self) -> bool:
        """Read Y limit switch (active HIGH)."""
        return self.pi.read(LIMIT_Y_PIN) == 1
    
    def set_speed(self, speed_percent: int):
        """Set motor speed (0-100%)."""
        self._current_speed = max(0, min(100, speed_percent))
    
    def _speed_to_steps_per_sec(self, speed_percent: int) -> int:
        """Convert speed percentage to steps per second."""
        speed = max(0, min(100, speed_percent))
        return int(MIN_SPEED + (speed / 100.0) * (MAX_SPEED - MIN_SPEED))
    
    def _calculate_speed_profile(self, total_steps: int, speed_percent: int) -> List[Tuple[int, int]]:
        """
        Calculate trapezoidal speed profile.
        
        Returns:
            List of (step_count, delay_us) tuples.
        """
        if total_steps <= 0:
            return []
        
        target_speed = self._speed_to_steps_per_sec(speed_percent)
        
        # Short moves: constant slow speed
        if total_steps <= ACCEL_STEPS * 2:
            delay_us = int(1_000_000 / MIN_SPEED)
            return [(total_steps, delay_us)]
        
        accel_steps = min(ACCEL_STEPS, total_steps // 3)
        decel_steps = accel_steps
        cruise_steps = total_steps - accel_steps - decel_steps
        
        profile = []
        
        # Acceleration
        for i in range(accel_steps):
            t = (i + 1) / accel_steps
            speed = MIN_SPEED + t * (target_speed - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))
        
        # Cruise
        if cruise_steps > 0:
            delay_us = int(1_000_000 / target_speed)
            profile.append((cruise_steps, delay_us))
        
        # Deceleration
        for i in range(decel_steps):
            t = (i + 1) / decel_steps
            speed = target_speed - t * (target_speed - MIN_SPEED)
            delay_us = int(1_000_000 / speed)
            profile.append((1, delay_us))
        
        return profile
    
    def _create_step_wave(self, step_a: bool, step_b: bool, delay_us: int) -> int:
        """Create a pigpio wave for stepping motors."""
        wait_us = max(1, delay_us - STEP_PULSE_US)
        
        set_mask = 0
        clear_mask = 0
        
        if step_a:
            set_mask |= (1 << MOTOR_A_STEP_PIN)
            clear_mask |= (1 << MOTOR_A_STEP_PIN)
        if step_b:
            set_mask |= (1 << MOTOR_B_STEP_PIN)
            clear_mask |= (1 << MOTOR_B_STEP_PIN)
        
        wave = [
            pigpio.pulse(set_mask, 0, STEP_PULSE_US),
            pigpio.pulse(0, clear_mask, wait_us)
        ]
        
        self.pi.wave_add_generic(wave)
        return self.pi.wave_create()
    
    def step_both_motors(self, steps_a: int, steps_b: int, speed_percent: Optional[int] = None):
        """
        Step both motors with Bresenham interpolation using DMA waves.
        
        Args:
            steps_a: Steps for motor A (negative = reverse)
            steps_b: Steps for motor B (negative = reverse)
            speed_percent: Speed (0-100), or None to use current speed
        """
        if speed_percent is None:
            speed_percent = self._current_speed
        
        if steps_a == 0 and steps_b == 0:
            return
        
        # Set directions
        self.pi.write(MOTOR_A_DIR_PIN, 1 if steps_a >= 0 else 0)
        self.pi.write(MOTOR_B_DIR_PIN, 1 if steps_b >= 0 else 0)
        time.sleep(DIR_SETUP_US / 1_000_000)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        profile = self._calculate_speed_profile(max_steps, speed_percent)
        
        self.motor_enable()
        
        try:
            err_a = 0
            err_b = 0
            
            for step_count, delay_us in profile:
                for _ in range(step_count):
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
                        wid = self._create_step_wave(do_step_a, do_step_b, delay_us)
                        self.pi.wave_send_once(wid)
                        while self.pi.wave_tx_busy():
                            time.sleep(0.0001)
                        self.pi.wave_delete(wid)
        finally:
            self.motor_disable()
    
    def move_x(self, steps: int, speed_percent: Optional[int] = None):
        """
        Move along X axis (CoreXY).
        
        +X (right): A+, B-
        """
        self.step_both_motors(steps, -steps, speed_percent)
        self._pos_x += steps
    
    def move_y(self, steps: int, speed_percent: Optional[int] = None):
        """
        Move along Y axis (CoreXY).
        
        +Y (up): A+, B+
        """
        self.step_both_motors(steps, steps, speed_percent)
        self._pos_y += steps
    
    def step_single_motor(self, motor: str, steps: int, speed_percent: Optional[int] = None):
        """Step a single motor (A or B)."""
        if motor.lower() == 'a':
            self.step_both_motors(steps, 0, speed_percent)
        else:
            self.step_both_motors(0, steps, speed_percent)
    
    def stop(self):
        """Stop motors and clean up step pins."""
        self.pi.write(MOTOR_A_STEP_PIN, 0)
        self.pi.write(MOTOR_B_STEP_PIN, 0)
        self.motor_disable()
    
    def cleanup(self):
        """Clean up resources."""
        self.stop()
        if self._owns_pi and self.pi.connected:
            self.pi.stop()
    
    @property
    def pos_x(self) -> int:
        return self._pos_x
    
    @property
    def pos_y(self) -> int:
        return self._pos_y
    
    def reset_position(self):
        """Reset position counters to zero (for homing)."""
        self._pos_x = 0
        self._pos_y = 0
