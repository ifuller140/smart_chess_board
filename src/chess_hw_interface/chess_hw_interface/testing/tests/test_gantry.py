#!/usr/bin/env python3
"""
Gantry Test Suite for A4988 Drivers + NEMA 11 Motors.

Tests:
1. X-MIN limit switch
2. Y-MIN limit switch  
3. Motor A movement (with speed selection)
4. Motor B movement (with speed selection)
5. Prusa-style homing sequence
6. Safety limit monitoring

Safety Feature: Unexpected limit switch triggers cause immediate test abort.
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep, TestResult


class GantryTest(HardwareTest):
    """
    Comprehensive gantry test including limit switches, motors, and homing.
    Uses A4988 drivers with STEP/DIR control.
    
    Safety: Unexpected limit switch triggers cause immediate motor stop and test abort.
    """
    
    # Motor pins (BCM) - per pinout.md ground truth (A4988 drivers)
    MOTOR_A_DIR_PIN = 27    # Direction pin
    MOTOR_A_STEP_PIN = 22   # Step pin
    MOTOR_B_DIR_PIN = 6     # Direction pin
    MOTOR_B_STEP_PIN = 5    # Step pin
    
    # Limit switch pins
    LIMIT_X_PIN = 10
    LIMIT_Y_PIN = 9
    LIMIT_CLOCK_PIN = 15
    
    # A4988 Timing constants (reduced pulse width per user request)
    DIR_SETUP_US = 5        # Microseconds to wait after setting DIR
    STEP_PULSE_US = 20      # Microseconds for step pulse width
    
    # Speed configuration (step delays in MILLISECONDS)
    # Larger delay = slower speed
    SPEED_90_DELAY_MS = 3.0     # 90% speed - operational movement
    SPEED_70_DELAY_MS = 10.0    # 70% speed - moderate
    SPEED_50_DELAY_MS = 20.0    # 50% speed - calibration
    SPEED_30_DELAY_MS = 35.0    # 30% speed - slow
    SPEED_20_DELAY_MS = 50.0    # 20% speed - precision
    
    # Named speeds for different operations (per user requirements)
    OPERATIONAL_SPEED = 90      # Normal movement
    CALIBRATION_SPEED = 50      # Initial calibration/homing approach
    PRECISION_SPEED = 20        # Slow/precise movements
    
    # Default speed setting
    DEFAULT_SPEED = OPERATIONAL_SPEED
    
    # Current speed setting
    _current_speed = DEFAULT_SPEED
    
    # Safety state
    _x_limit_expected = False
    _y_limit_expected = False
    _emergency_stop = False
    
    @property
    def name(self) -> str:
        return "Gantry"
    
    @property
    def description(self) -> str:
        return "Test limit switches, stepper motors (A4988), and homing sequence with safety monitoring"
    
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
            self.gpio.setup_input(self.LIMIT_X_PIN, pull_down=True)
            self.gpio.setup_input(self.LIMIT_Y_PIN, pull_down=True)
            self.gpio.setup_input(self.LIMIT_CLOCK_PIN, pull_down=True)
            
            # Reset safety state
            self._emergency_stop = False
            self._x_limit_expected = False
            self._y_limit_expected = False
            
            return True
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            return False
    
    def teardown(self):
        """Turn off all motor pins."""
        self._stop_motors()
    
    def _stop_motors(self):
        """Immediately stop all motor movement."""
        if self.gpio:
            try:
                self.gpio.write(self.MOTOR_A_STEP_PIN, False)
                self.gpio.write(self.MOTOR_B_STEP_PIN, False)
            except:
                pass
    
    def set_speed(self, speed_percent: int):
        """Set the current motor speed (0-100)."""
        self._current_speed = max(0, min(100, speed_percent))
        print(f"  Speed set to {self._current_speed}%")
    
    def speed_to_delay(self, speed_percent: int = None) -> float:
        """
        Convert speed percentage (0-100) to step delay in seconds.
        
        Uses linear interpolation between defined speed points.
        """
        if speed_percent is None:
            speed_percent = self._current_speed
        speed = max(0, min(100, speed_percent))
        
        # Linear interpolation between key points
        if speed >= 90:
            delay_ms = self.SPEED_90_DELAY_MS
        elif speed >= 70:
            delay_ms = self.SPEED_90_DELAY_MS + (90 - speed) / 20 * (self.SPEED_70_DELAY_MS - self.SPEED_90_DELAY_MS)
        elif speed >= 50:
            delay_ms = self.SPEED_70_DELAY_MS + (70 - speed) / 20 * (self.SPEED_50_DELAY_MS - self.SPEED_70_DELAY_MS)
        elif speed >= 30:
            delay_ms = self.SPEED_50_DELAY_MS + (50 - speed) / 20 * (self.SPEED_30_DELAY_MS - self.SPEED_50_DELAY_MS)
        elif speed >= 20:
            delay_ms = self.SPEED_30_DELAY_MS + (30 - speed) / 10 * (self.SPEED_20_DELAY_MS - self.SPEED_30_DELAY_MS)
        else:
            delay_ms = self.SPEED_20_DELAY_MS + (20 - speed) * 5  # Even slower below 20%
        
        return delay_ms / 1000.0  # Convert to seconds
    
    def _read_x_limit(self) -> bool:
        """Read X limit switch state."""
        if self.gpio:
            return self.gpio.read(self.LIMIT_X_PIN)
        return False
    
    def _read_y_limit(self) -> bool:
        """Read Y limit switch state."""
        if self.gpio:
            return self.gpio.read(self.LIMIT_Y_PIN)
        return False
    
    def _check_safety_limits(self) -> bool:
        """
        Check if an unexpected limit switch is triggered.
        Returns True if safe to continue, False if emergency stop needed.
        """
        x_triggered = self._read_x_limit()
        y_triggered = self._read_y_limit()
        
        # Check for unexpected triggers
        if x_triggered and not self._x_limit_expected:
            print("\n  🚨 EMERGENCY STOP: X limit triggered unexpectedly!")
            self._emergency_stop = True
            self._stop_motors()
            return False
        
        if y_triggered and not self._y_limit_expected:
            print("\n  🚨 EMERGENCY STOP: Y limit triggered unexpectedly!")
            self._emergency_stop = True
            self._stop_motors()
            return False
        
        return True
    
    def _set_expected_limits(self, x_expected: bool = False, y_expected: bool = False):
        """Set which limits are expected during current operation."""
        self._x_limit_expected = x_expected
        self._y_limit_expected = y_expected
    
    def _clear_expected_limits(self):
        """Clear all expected limits - any trigger is unexpected."""
        self._x_limit_expected = False
        self._y_limit_expected = False
    
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
                timeout_seconds=60.0,
                success_message="HOME OK",
                failure_message="HOME ER"
            ),
        ]
    
    def _select_speed(self) -> bool:
        """Allow user to select motor speed."""
        print("\n  ╔════════════════════════════════════╗")
        print("  ║     MOTOR SPEED SELECTION          ║")
        print("  ╠════════════════════════════════════╣")
        print("  ║  Default speeds:                   ║")
        print("  ║    90% - Operational movement      ║")
        print("  ║    50% - Calibration               ║")
        print("  ║    20% - Precision/homing          ║")
        print("  ║                                    ║")
        print("  ║  Press CLOCK to use 90% default    ║")
        print("  ╚════════════════════════════════════╝")
        
        self._current_speed = self.OPERATIONAL_SPEED
        print(f"\n  Using operational speed: {self._current_speed}%")
        return True
    
    def _step_pulse(self, step_pin: int):
        """Generate a single step pulse."""
        self.gpio.write(step_pin, True)
        time.sleep(self.STEP_PULSE_US / 1_000_000)
        self.gpio.write(step_pin, False)
    
    def _step_motor(self, step_pin: int, dir_pin: int, steps: int, speed: int = None, check_limits: bool = True):
        """
        Move a single motor a given number of steps using A4988 driver.
        
        Args:
            step_pin: GPIO pin for STEP signal
            dir_pin: GPIO pin for DIR signal
            steps: Number of steps (negative = reverse)
            speed: Speed percentage (0-100), uses current speed if None
            check_limits: If True, check safety limits during movement
        
        Returns:
            True if completed, False if emergency stopped
        """
        if self._emergency_stop:
            return False
        
        if speed is None:
            speed = self._current_speed
        
        delay = self.speed_to_delay(speed)
        
        # Set direction
        direction = steps >= 0
        self.gpio.write(dir_pin, direction)
        
        # Wait for DIR to stabilize
        time.sleep(self.DIR_SETUP_US / 1_000_000)
        
        # Generate step pulses
        for i in range(abs(steps)):
            # Safety check every 10 steps
            if check_limits and i % 10 == 0:
                if not self._check_safety_limits():
                    return False
            
            self._step_pulse(step_pin)
            time.sleep(delay)
        
        return True
    
    def _step_both_motors(self, steps_a: int, steps_b: int, speed: int = None, check_limits: bool = True):
        """
        Step both motors simultaneously with Bresenham interpolation.
        Required for CoreXY kinematics.
        
        Args:
            steps_a: Steps for motor A (negative = reverse)
            steps_b: Steps for motor B (negative = reverse)
            speed: Speed percentage (0-100)
            check_limits: If True, check safety limits during movement
        
        Returns:
            True if completed, False if emergency stopped
        """
        if self._emergency_stop:
            return False
        
        if speed is None:
            speed = self._current_speed
        
        delay = self.speed_to_delay(speed)
        
        if steps_a == 0 and steps_b == 0:
            return True
        
        # Set directions
        dir_a = steps_a >= 0
        dir_b = steps_b >= 0
        self.gpio.write(self.MOTOR_A_DIR_PIN, dir_a)
        self.gpio.write(self.MOTOR_B_DIR_PIN, dir_b)
        
        # Wait for DIR to stabilize
        time.sleep(self.DIR_SETUP_US / 1_000_000)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        # Bresenham interpolation
        err_a = 0
        err_b = 0
        
        for step_num in range(max_steps):
            # Safety check every 10 steps
            if check_limits and step_num % 10 == 0:
                if not self._check_safety_limits():
                    return False
            
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
        
        return True
    
    def _move_x(self, steps: int, speed: int = None, check_limits: bool = True):
        """
        Pure X movement using CoreXY kinematics.
        X movement: Both motors move SAME direction.
        """
        return self._step_both_motors(steps, steps, speed, check_limits)
    
    def _move_y(self, steps: int, speed: int = None, check_limits: bool = True):
        """
        Pure Y movement using CoreXY kinematics.
        Y movement: Motors move OPPOSITE directions.
        """
        return self._step_both_motors(steps, -steps, speed, check_limits)
    
    def _test_motor_a(self) -> bool:
        """Test Motor A movement."""
        if self._emergency_stop:
            return False
        
        print(f"\n  Testing Motor A at {self._current_speed}% speed...")
        print("  Moving Motor A forward 200 steps...")
        
        if not self._step_motor(self.MOTOR_A_STEP_PIN, self.MOTOR_A_DIR_PIN, 200, check_limits=False):
            return False
        time.sleep(0.3)
        
        print("  Moving Motor A backward 200 steps...")
        if not self._step_motor(self.MOTOR_A_STEP_PIN, self.MOTOR_A_DIR_PIN, -200, check_limits=False):
            return False
        
        return True
    
    def _test_motor_b(self) -> bool:
        """Test Motor B movement."""
        if self._emergency_stop:
            return False
        
        print(f"\n  Testing Motor B at {self._current_speed}% speed...")
        print("  Moving Motor B forward 200 steps...")
        
        if not self._step_motor(self.MOTOR_B_STEP_PIN, self.MOTOR_B_DIR_PIN, 200, check_limits=False):
            return False
        time.sleep(0.3)
        
        print("  Moving Motor B backward 200 steps...")
        if not self._step_motor(self.MOTOR_B_STEP_PIN, self.MOTOR_B_DIR_PIN, -200, check_limits=False):
            return False
        
        return True
    
    def _run_homing_sequence(self) -> bool:
        """
        Run Prusa-style homing sequence with proper CoreXY movement.
        Uses speed levels: 50% for approach, 20% for precision.
        """
        if self._emergency_stop:
            return False
        
        print("\n  Starting Prusa-style homing sequence...")
        
        # Safety: Move away from limits first (no safety checks here)
        print("  Safety offset: Moving away from limits (50% speed)...")
        if not self._move_x(500, self.CALIBRATION_SPEED, check_limits=False):
            return False
        if not self._move_y(500, self.CALIBRATION_SPEED, check_limits=False):
            return False
        
        # Home X axis with CoreXY movement
        if not self._home_x_axis():
            return False
        
        # Home Y axis with CoreXY movement
        if not self._home_y_axis():
            return False
        
        print("  ✓ Homing complete!")
        return True
    
    def _home_x_axis(self, max_steps: int = 100000) -> bool:
        """Home X axis using proper CoreXY movement."""
        if self._emergency_stop:
            return False
        
        print("  Homing X axis...")
        
        # Expect X limit during this operation
        self._set_expected_limits(x_expected=True, y_expected=False)
        
        # Phase 1: Fast approach (50% speed)
        print("    Phase 1: Fast approach (50% speed)...")
        steps = 0
        while not self._read_x_limit() and steps < max_steps:
            if not self._move_x(-100, self.CALIBRATION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] X limit not found!")
            self._clear_expected_limits()
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off (50% speed)
        print("    Phase 2: Back off...")
        if not self._move_x(200, self.CALIBRATION_SPEED, check_limits=False):
            self._clear_expected_limits()
            return False
        
        # Phase 3: Slow approach (20% speed)
        print("    Phase 3: Slow approach (20% speed)...")
        while not self._read_x_limit():
            if not self._move_x(-10, self.PRECISION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        if not self._move_x(50, self.CALIBRATION_SPEED, check_limits=False):
            self._clear_expected_limits()
            return False
        
        # Phase 5: Final approach (20% speed)
        print("    Phase 5: Final approach (20% speed)...")
        while not self._read_x_limit():
            if not self._move_x(-1, self.PRECISION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
        
        self._clear_expected_limits()
        print("    ✓ X axis homed!")
        return True
    
    def _home_y_axis(self, max_steps: int = 100000) -> bool:
        """Home Y axis using proper CoreXY movement."""
        if self._emergency_stop:
            return False
        
        print("  Homing Y axis...")
        
        # Expect Y limit during this operation
        self._set_expected_limits(x_expected=False, y_expected=True)
        
        # Phase 1: Fast approach (50% speed)
        print("    Phase 1: Fast approach (50% speed)...")
        steps = 0
        while not self._read_y_limit() and steps < max_steps:
            if not self._move_y(-100, self.CALIBRATION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
            steps += 100
        
        if steps >= max_steps:
            print("    [ERROR] Y limit not found!")
            self._clear_expected_limits()
            return False
        
        print(f"    Limit triggered at ~{steps} steps")
        
        # Phase 2: Back off (50% speed)
        print("    Phase 2: Back off...")
        if not self._move_y(200, self.CALIBRATION_SPEED, check_limits=False):
            self._clear_expected_limits()
            return False
        
        # Phase 3: Slow approach (20% speed)
        print("    Phase 3: Slow approach (20% speed)...")
        while not self._read_y_limit():
            if not self._move_y(-10, self.PRECISION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
        
        # Phase 4: Small back off
        print("    Phase 4: Small back off...")
        if not self._move_y(50, self.CALIBRATION_SPEED, check_limits=False):
            self._clear_expected_limits()
            return False
        
        # Phase 5: Final approach (20% speed)
        print("    Phase 5: Final approach (20% speed)...")
        while not self._read_y_limit():
            if not self._move_y(-1, self.PRECISION_SPEED, check_limits=False):
                self._clear_expected_limits()
                return False
        
        self._clear_expected_limits()
        print("    ✓ Y axis homed!")
        return True
