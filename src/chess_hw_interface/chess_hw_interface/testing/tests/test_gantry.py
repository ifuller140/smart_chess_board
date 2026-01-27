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
    
    def _step_both_motors(self, steps_a: int, steps_b: int, delay: float = None):
        """
        Step both motors simultaneously with interpolation.
        Required for CoreXY kinematics.
        """
        if delay is None:
            delay = self.STEP_DELAY
        
        if steps_a == 0 and steps_b == 0:
            return
        
        dir_a = 1 if steps_a >= 0 else -1
        dir_b = 1 if steps_b >= 0 else -1
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        err_a = 0
        err_b = 0
        idx_a = 0
        idx_b = 0
        
        for _ in range(max_steps):
            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                idx_a = (idx_a + dir_a) % len(self.STEP_SEQUENCE)
                seq = self.STEP_SEQUENCE[idx_a]
                for i, pin in enumerate(self.MOTOR_A_PINS):
                    self.gpio.write(pin, bool(seq[i]))
            
            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                idx_b = (idx_b + dir_b) % len(self.STEP_SEQUENCE)
                seq = self.STEP_SEQUENCE[idx_b]
                for i, pin in enumerate(self.MOTOR_B_PINS):
                    self.gpio.write(pin, bool(seq[i]))
            
            time.sleep(delay)
    
    def _move_x(self, steps: int, delay: float = None):
        """
        Pure X movement using CoreXY kinematics.
        X movement: Motors move OPPOSITE directions.
        """
        self._step_both_motors(steps, -steps, delay)
    
    def _move_y(self, steps: int, delay: float = None):
        """
        Pure Y movement using CoreXY kinematics.
        Y movement: Both motors move SAME direction.
        """
        self._step_both_motors(steps, steps, delay)
    
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
        Run Prusa-style homing sequence with proper CoreXY movement.
        """
        print("\n  Starting Prusa-style homing sequence...")
        
        # Safety: Move away from limits first
        print("  Safety offset: Moving away from limits...")
        self._move_x(500, self.FAST_STEP_DELAY)
        self._move_y(500, self.FAST_STEP_DELAY)
        
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
            self._move_x(-100, self.FAST_STEP_DELAY)
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] X limit not found!")
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off
        print("    Phase 2: Back off...")
        self._move_x(200, self.STEP_DELAY)
        
        # Phase 3: Slow approach
        print("    Phase 3: Slow approach...")
        while not limit_func():
            self._move_x(-10, self.SLOW_STEP_DELAY)
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        self._move_x(50, self.STEP_DELAY)
        
        # Phase 5: Final approach
        print("    Phase 5: Final approach...")
        while not limit_func():
            self._move_x(-1, self.SLOW_STEP_DELAY)
        
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
            self._move_y(-100, self.FAST_STEP_DELAY)
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] Y limit not found!")
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off
        print("    Phase 2: Back off...")
        self._move_y(200, self.STEP_DELAY)
        
        # Phase 3: Slow approach
        print("    Phase 3: Slow approach...")
        while not limit_func():
            self._move_y(-10, self.SLOW_STEP_DELAY)
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        self._move_y(50, self.STEP_DELAY)
        
        # Phase 5: Final approach
        print("    Phase 5: Final approach...")
        while not limit_func():
            self._move_y(-1, self.SLOW_STEP_DELAY)
        
        print("    ✓ Y axis homed!")
        return True
    
    # Legacy _home_axis method removed - now using _home_x_axis and _home_y_axis
    # with proper CoreXY kinematics
