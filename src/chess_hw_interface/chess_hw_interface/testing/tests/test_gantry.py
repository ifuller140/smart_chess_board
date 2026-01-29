#!/usr/bin/env python3
"""
Gantry Test Suite for A4988 Drivers + NEMA 11 Motors.

Tests:
1. X-MIN limit switch
2. Y-MIN limit switch  
3. Motor A movement (with speed selection)
4. Motor B movement (with speed selection)
5. Prusa-style homing sequence
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep, TestResult


class GantryTest(HardwareTest):
    """
    Comprehensive gantry test including limit switches, motors, and homing.
    Uses A4988 drivers with STEP/DIR control.
    """
    
    # Motor pins (BCM) - per pinout.md ground truth (A4988 drivers)
    MOTOR_A_DIR_PIN = 27    # Direction pin
    MOTOR_A_STEP_PIN = 22   # Step pin
    MOTOR_B_DIR_PIN = 6     # Direction pin
    MOTOR_B_STEP_PIN = 5    # Step pin
    
    # Timing constants
    STEP_PULSE_US = 10      # Microseconds for step pulse width (min 2µs for A4988)
    
    # Speed mapping (0-100 scale)
    # Step delay determines speed: smaller delay = faster
    MIN_STEP_DELAY_US = 100     # Maximum speed (100%)
    MAX_STEP_DELAY_US = 5000    # Minimum speed (0%)
    
    # Default speeds for different operations
    DEFAULT_SPEED = 50          # Default test speed (%)
    FAST_SPEED = 80             # Fast approach speed (%)
    SLOW_SPEED = 20             # Slow/precise speed (%)
    
    # Current speed setting
    _current_speed = DEFAULT_SPEED
    
    @property
    def name(self) -> str:
        return "Gantry"
    
    @property
    def description(self) -> str:
        return "Test limit switches, stepper motors (A4988), and homing sequence"
    
    def setup(self) -> bool:
        """Setup motor pins as outputs."""
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        try:
            # Setup motor output pins
            for pin in [self.MOTOR_A_DIR_PIN, self.MOTOR_A_STEP_PIN,
                        self.MOTOR_B_DIR_PIN, self.MOTOR_B_STEP_PIN]:
                self.gpio.setup_output(pin)
            
            # Setup limit switch inputs - per pinout.md ground truth
            # Active HIGH: 1=Pressed, 0=Released. Use pull_down=True
            self.gpio.setup_input(10, pull_down=True)  # X-MIN: GPIO10, Physical Pin 19
            self.gpio.setup_input(9, pull_down=True)   # Y-MIN: GPIO9, Physical Pin 21
            self.gpio.setup_input(15, pull_down=True)  # Clock: GPIO15, Physical Pin 10
            
            return True
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            return False
    
    def teardown(self):
        """Turn off all motor pins."""
        if self.gpio:
            for pin in [self.MOTOR_A_STEP_PIN, self.MOTOR_B_STEP_PIN]:
                self.gpio.write(pin, False)
    
    def set_speed(self, speed_percent: int):
        """Set the current motor speed (0-100)."""
        self._current_speed = max(0, min(100, speed_percent))
        print(f"  Speed set to {self._current_speed}%")
    
    def speed_to_delay(self, speed_percent: int = None) -> float:
        """
        Convert speed percentage (0-100) to step delay in seconds.
        
        0 = slowest (MAX_STEP_DELAY_US)
        100 = fastest (MIN_STEP_DELAY_US)
        """
        if speed_percent is None:
            speed_percent = self._current_speed
        speed = max(0, min(100, speed_percent))
        delay_us = self.MAX_STEP_DELAY_US - (speed / 100.0) * (self.MAX_STEP_DELAY_US - self.MIN_STEP_DELAY_US)
        return delay_us / 1_000_000  # Convert to seconds
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Step 1: Speed selection
            TestStep(
                name="Speed Selection",
                display_text="SPEED",
                action=self._select_speed,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=60.0,
                success_message="SPD SET",
                failure_message="SPD ERR"
            ),
            
            # Step 2: X limit switch
            TestStep(
                name="X-MIN Limit Switch",
                display_text="CLICK X",
                action=lambda: True,
                wait_for_input=True,
                input_type="x_limit",
                timeout_seconds=30.0,
                success_message="X OK",
                failure_message="X FAIL"
            ),
            
            # Step 3: Y limit switch
            TestStep(
                name="Y-MIN Limit Switch",
                display_text="CLICK Y",
                action=lambda: True,
                wait_for_input=True,
                input_type="y_limit",
                timeout_seconds=30.0,
                success_message="Y OK",
                failure_message="Y FAIL"
            ),
            
            # Step 4: Motor A test
            TestStep(
                name="Motor A Movement",
                display_text="MOTOR A",
                action=self._test_motor_a,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="A OK?",
                failure_message="A FAIL"
            ),
            
            # Step 5: Motor A confirmation
            TestStep(
                name="Confirm Motor A",
                display_text="A OK?",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="A CONF",
                failure_message="A BAD"
            ),
            
            # Step 6: Motor B test
            TestStep(
                name="Motor B Movement",
                display_text="MOTOR B",
                action=self._test_motor_b,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="B OK?",
                failure_message="B FAIL"
            ),
            
            # Step 7: Motor B confirmation
            TestStep(
                name="Confirm Motor B",
                display_text="B OK?",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="B CONF",
                failure_message="B BAD"
            ),
            
            # Step 8: Homing sequence
            TestStep(
                name="Homing Sequence",
                display_text="HOMING",
                action=self._run_homing_sequence,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="HOME OK",
                failure_message="HOME ER"
            ),
        ]
    
    def _select_speed(self) -> bool:
        """Allow user to select motor speed."""
        print("\n  ╔════════════════════════════════════╗")
        print("  ║     MOTOR SPEED SELECTION          ║")
        print("  ╠════════════════════════════════════╣")
        print("  ║  Press CLOCK to cycle speeds:      ║")
        print("  ║    20% → 40% → 60% → 80% → 100%   ║")
        print("  ║                                    ║")
        print("  ║  Hold for 2s to confirm selection  ║")
        print("  ╚════════════════════════════════════╝")
        
        speeds = [20, 40, 60, 80, 100]
        speed_idx = 0
        self._current_speed = speeds[speed_idx]
        
        print(f"\n  Current speed: {self._current_speed}%")
        print("  (Press CLOCK to change, hold 2s to confirm)")
        
        # For now, use default speed - clock button cycling would require
        # more complex input handling. User can press clock to confirm.
        self._current_speed = self.DEFAULT_SPEED
        print(f"\n  Using default speed: {self._current_speed}%")
        return True
    
    def _step_pulse(self, step_pin: int):
        """Generate a single step pulse."""
        self.gpio.write(step_pin, True)
        time.sleep(self.STEP_PULSE_US / 1_000_000)
        self.gpio.write(step_pin, False)
    
    def _step_motor(self, step_pin: int, dir_pin: int, steps: int, speed: int = None):
        """
        Move a single motor a given number of steps using A4988 driver.
        
        Args:
            step_pin: GPIO pin for STEP signal
            dir_pin: GPIO pin for DIR signal
            steps: Number of steps (negative = reverse)
            speed: Speed percentage (0-100), uses current speed if None
        """
        if speed is None:
            speed = self._current_speed
        
        delay = self.speed_to_delay(speed)
        
        # Set direction
        direction = steps >= 0
        self.gpio.write(dir_pin, direction)
        
        # Generate step pulses
        for _ in range(abs(steps)):
            self._step_pulse(step_pin)
            time.sleep(delay)
    
    def _step_both_motors(self, steps_a: int, steps_b: int, speed: int = None):
        """
        Step both motors simultaneously with Bresenham interpolation.
        Required for CoreXY kinematics.
        
        Args:
            steps_a: Steps for motor A (negative = reverse)
            steps_b: Steps for motor B (negative = reverse)
            speed: Speed percentage (0-100)
        """
        if speed is None:
            speed = self._current_speed
        
        delay = self.speed_to_delay(speed)
        
        if steps_a == 0 and steps_b == 0:
            return
        
        # Set directions
        dir_a = steps_a >= 0
        dir_b = steps_b >= 0
        self.gpio.write(self.MOTOR_A_DIR_PIN, dir_a)
        self.gpio.write(self.MOTOR_B_DIR_PIN, dir_b)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        # Bresenham interpolation
        err_a = 0
        err_b = 0
        
        for _ in range(max_steps):
            step_a = False
            step_b = False
            
            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                step_a = True
                
            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                step_b = True
            
            # Pulse step pins (simultaneously if both need to step)
            if step_a:
                self.gpio.write(self.MOTOR_A_STEP_PIN, True)
            if step_b:
                self.gpio.write(self.MOTOR_B_STEP_PIN, True)
            
            time.sleep(self.STEP_PULSE_US / 1_000_000)
            
            self.gpio.write(self.MOTOR_A_STEP_PIN, False)
            self.gpio.write(self.MOTOR_B_STEP_PIN, False)
            
            time.sleep(delay)
    
    def _move_x(self, steps: int, speed: int = None):
        """
        Pure X movement using CoreXY kinematics.
        X movement: Motors move OPPOSITE directions.
        """
        self._step_both_motors(steps, -steps, speed)
    
    def _move_y(self, steps: int, speed: int = None):
        """
        Pure Y movement using CoreXY kinematics.
        Y movement: Both motors move SAME direction.
        """
        self._step_both_motors(steps, steps, speed)
    
    def _test_motor_a(self) -> bool:
        """Test Motor A movement."""
        print(f"\n  Testing Motor A at {self._current_speed}% speed...")
        print("  Moving Motor A forward 200 steps...")
        self._step_motor(self.MOTOR_A_STEP_PIN, self.MOTOR_A_DIR_PIN, 200)
        time.sleep(0.3)
        
        print("  Moving Motor A backward 200 steps...")
        self._step_motor(self.MOTOR_A_STEP_PIN, self.MOTOR_A_DIR_PIN, -200)
        
        return True
    
    def _test_motor_b(self) -> bool:
        """Test Motor B movement."""
        print(f"\n  Testing Motor B at {self._current_speed}% speed...")
        print("  Moving Motor B forward 200 steps...")
        self._step_motor(self.MOTOR_B_STEP_PIN, self.MOTOR_B_DIR_PIN, 200)
        time.sleep(0.3)
        
        print("  Moving Motor B backward 200 steps...")
        self._step_motor(self.MOTOR_B_STEP_PIN, self.MOTOR_B_DIR_PIN, -200)
        
        return True
    
    def _run_homing_sequence(self) -> bool:
        """
        Run Prusa-style homing sequence with proper CoreXY movement.
        """
        print("\n  Starting Prusa-style homing sequence...")
        
        # Safety: Move away from limits first
        print("  Safety offset: Moving away from limits...")
        self._move_x(500, self.FAST_SPEED)
        self._move_y(500, self.FAST_SPEED)
        
        # Home X axis with CoreXY movement
        if not self._home_x_axis():
            return False
        
        # Home Y axis with CoreXY movement
        if not self._home_y_axis():
            return False
        
        print("  Homing complete!")
        return True
    
    def _home_x_axis(self, max_steps: int = 50000) -> bool:
        """Home X axis using proper CoreXY movement."""
        print("  Homing X axis...")
        limit_func = self.gpio.read_x_limit if self.gpio else lambda: False
        
        # Phase 1: Fast approach
        print("    Phase 1: Fast approach...")
        steps = 0
        while not limit_func() and steps < max_steps:
            self._move_x(-100, self.FAST_SPEED)
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] X limit not found!")
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off
        print("    Phase 2: Back off...")
        self._move_x(200, self.DEFAULT_SPEED)
        
        # Phase 3: Slow approach
        print("    Phase 3: Slow approach...")
        while not limit_func():
            self._move_x(-10, self.SLOW_SPEED)
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        self._move_x(50, self.DEFAULT_SPEED)
        
        # Phase 5: Final approach
        print("    Phase 5: Final approach...")
        while not limit_func():
            self._move_x(-1, self.SLOW_SPEED)
        
        print("    ✓ X axis homed!")
        return True
    
    def _home_y_axis(self, max_steps: int = 50000) -> bool:
        """Home Y axis using proper CoreXY movement."""
        print("  Homing Y axis...")
        limit_func = self.gpio.read_y_limit if self.gpio else lambda: False
        
        # Phase 1: Fast approach
        print("    Phase 1: Fast approach...")
        steps = 0
        while not limit_func() and steps < max_steps:
            self._move_y(-100, self.FAST_SPEED)
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] Y limit not found!")
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off
        print("    Phase 2: Back off...")
        self._move_y(200, self.DEFAULT_SPEED)
        
        # Phase 3: Slow approach
        print("    Phase 3: Slow approach...")
        while not limit_func():
            self._move_y(-10, self.SLOW_SPEED)
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        self._move_y(50, self.DEFAULT_SPEED)
        
        # Phase 5: Final approach
        print("    Phase 5: Final approach...")
        while not limit_func():
            self._move_y(-1, self.SLOW_SPEED)
        
        print("    ✓ Y axis homed!")
        return True
