#!/usr/bin/env python3
"""
Gantry Test Suite.

Tests:
1. X-MIN limit switch
2. Y-MIN limit switch  
3. Motor A movement
4. Motor B movement
5. Prusa-style homing sequence
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep, TestResult


class GantryTest(HardwareTest):
    """
    Comprehensive gantry test including limit switches, motors, and homing.
    """
    
    # Motor pins (BCM) - per pinout.md ground truth
    MOTOR_A_PINS = [14, 4, 3, 2]    # IN1, IN2, IN3, IN4
    MOTOR_B_PINS = [24, 23, 22, 27]  # IN1, IN2, IN3, IN4
    
    # Step sequence (half-step)
    STEP_SEQUENCE = [
        [1, 0, 0, 0],
        [1, 1, 0, 0],
        [0, 1, 0, 0],
        [0, 1, 1, 0],
        [0, 0, 1, 0],
        [0, 0, 1, 1],
        [0, 0, 0, 1],
        [1, 0, 0, 1],
    ]
    
    # Timing
    STEP_DELAY = 0.002  # 2ms between steps
    FAST_STEP_DELAY = 0.001
    SLOW_STEP_DELAY = 0.005
    
    @property
    def name(self) -> str:
        return "Gantry"
    
    @property
    def description(self) -> str:
        return "Test limit switches, stepper motors, and homing sequence"
    
    def setup(self) -> bool:
        """Setup motor pins as outputs."""
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        try:
            for pin in self.MOTOR_A_PINS + self.MOTOR_B_PINS:
                self.gpio.setup_output(pin)
            
            # Setup limit switch inputs - per pinout.md ground truth diagram
            self.gpio.setup_input(10, pull_up=True)  # X-MIN: GPIO10, Physical Pin 19
            self.gpio.setup_input(9, pull_up=True)   # Y-MIN: GPIO9, Physical Pin 21
            self.gpio.setup_input(15, pull_up=True)  # Clock: GPIO15, Physical Pin 10
            
            return True
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            return False
    
    def teardown(self):
        """Turn off all motor pins."""
        if self.gpio:
            for pin in self.MOTOR_A_PINS + self.MOTOR_B_PINS:
                self.gpio.write(pin, False)
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Step 1: X limit switch
            TestStep(
                name="X-MIN Limit Switch",
                display_text="CLICK X",
                action=lambda: True,  # Action happens after input
                wait_for_input=True,
                input_type="x_limit",
                timeout_seconds=30.0,
                success_message="X OK",
                failure_message="X FAIL"
            ),
            
            # Step 2: Y limit switch
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
            
            # Step 3: Motor A test
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
            
            # Step 4: Motor A confirmation
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
            
            # Step 5: Motor B test
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
            
            # Step 6: Motor B confirmation
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
            
            # Step 7: Homing sequence
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
    
    def _step_motor(self, pins: List[int], steps: int, delay: float = None):
        """
        Move a motor a given number of steps.
        
        Args:
            pins: Motor pin list [IN1, IN2, IN3, IN4]
            steps: Number of steps (positive = forward, negative = backward)
            delay: Delay between steps (uses default if None)
        """
        if delay is None:
            delay = self.STEP_DELAY
        
        direction = 1 if steps > 0 else -1
        step_count = abs(steps)
        
        for _ in range(step_count):
            for seq in (self.STEP_SEQUENCE if direction > 0 else reversed(self.STEP_SEQUENCE)):
                for pin_idx, pin in enumerate(pins):
                    self.gpio.write(pin, bool(seq[pin_idx]))
                time.sleep(delay)
    
    def _test_motor_a(self) -> bool:
        """Test Motor A movement."""
        print("  Moving Motor A forward 100 steps...")
        self._step_motor(self.MOTOR_A_PINS, 100)
        time.sleep(0.3)
        
        print("  Moving Motor A backward 100 steps...")
        self._step_motor(self.MOTOR_A_PINS, -100)
        
        return True
    
    def _test_motor_b(self) -> bool:
        """Test Motor B movement."""
        print("  Moving Motor B forward 100 steps...")
        self._step_motor(self.MOTOR_B_PINS, 100)
        time.sleep(0.3)
        
        print("  Moving Motor B backward 100 steps...")
        self._step_motor(self.MOTOR_B_PINS, -100)
        
        return True
    
    def _run_homing_sequence(self) -> bool:
        """
        Run Prusa-style homing sequence.
        
        1. Fast move to limit switch
        2. Back off
        3. Slow approach
        4. Back off slightly
        5. Very slow final approach
        """
        print("\n  Starting Prusa-style homing sequence...")
        
        # Home X axis
        if not self._home_axis("X", self.MOTOR_A_PINS, self.gpio.read_x_limit if self.gpio else lambda: False):
            return False
        
        # Home Y axis
        if not self._home_axis("Y", self.MOTOR_B_PINS, self.gpio.read_y_limit if self.gpio else lambda: False):
            return False
        
        print("  Homing complete!")
        return True
    
    def _home_axis(self, axis_name: str, motor_pins: List[int], limit_func) -> bool:
        """
        Home a single axis using Prusa-style sequence.
        
        Args:
            axis_name: Name for logging
            motor_pins: Motor pins for this axis
            limit_func: Function to read limit switch state
            
        Returns:
            True if homing successful
        """
        print(f"  Homing {axis_name} axis...")
        
        # Phase 1: Fast approach
        print(f"    Phase 1: Fast approach to {axis_name}-MIN...")
        steps_taken = 0
        max_steps = 5000  # Safety limit
        
        while not limit_func() and steps_taken < max_steps:
            self._step_motor(motor_pins, -10, self.FAST_STEP_DELAY)
            steps_taken += 10
        
        if steps_taken >= max_steps:
            print(f"    [ERROR] {axis_name}-MIN not found within {max_steps} steps")
            return False
        
        print(f"    {axis_name}-MIN triggered after {steps_taken} steps")
        
        # Phase 2: Back off 100 steps
        print(f"    Phase 2: Backing off...")
        self._step_motor(motor_pins, 100, self.STEP_DELAY)
        
        # Phase 3: Slow approach
        print(f"    Phase 3: Slow approach...")
        steps_taken = 0
        while not limit_func() and steps_taken < 200:
            self._step_motor(motor_pins, -1, self.SLOW_STEP_DELAY)
            steps_taken += 1
        
        # Phase 4: Back off 20 steps
        print(f"    Phase 4: Small backoff...")
        self._step_motor(motor_pins, 20, self.STEP_DELAY)
        
        # Phase 5: Very slow final approach
        print(f"    Phase 5: Final approach...")
        steps_taken = 0
        while not limit_func() and steps_taken < 50:
            self._step_motor(motor_pins, -1, self.SLOW_STEP_DELAY * 2)
            steps_taken += 1
        
        print(f"    {axis_name}-axis homed successfully")
        return True
