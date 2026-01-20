#!/usr/bin/env python3
"""
Servo Test Suite.

Tests:
1. Z-axis servo (gantry magnet lift)
2. Clock servo (hits clock button after computer move)
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep


class ServoTest(HardwareTest):
    """
    Test both servos: Z-axis (gantry) and clock button.
    """
    
    # Servo pins (BCM)
    Z_SERVO_PIN = 12      # Gantry Z-axis
    CLOCK_SERVO_PIN = 16  # Clock button hitter
    
    # PWM settings
    PWM_FREQUENCY = 50  # 50Hz for servos
    
    # Z-axis servo positions (duty cycle %)
    Z_UP_PWM = 2.5      # Magnet raised
    Z_DOWN_PWM = 7.5    # Magnet lowered
    
    # Clock servo positions (duty cycle %)
    # USER_ATTENTION: Calibrate these values for your clock button
    CLOCK_REST_PWM = 2.5   # Away from button
    CLOCK_HIT_PWM = 7.5    # Pressing button
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._z_pwm = None
        self._clock_pwm = None
    
    @property
    def name(self) -> str:
        return "Servo"
    
    @property
    def description(self) -> str:
        return "Test Z-axis and clock button servos"
    
    def setup(self) -> bool:
        """Setup PWM for servos."""
        if self.gpio is None or self.gpio.mock:
            return True  # Mock mode
        
        try:
            import RPi.GPIO as GPIO
            
            GPIO.setup(self.Z_SERVO_PIN, GPIO.OUT)
            GPIO.setup(self.CLOCK_SERVO_PIN, GPIO.OUT)
            
            self._z_pwm = GPIO.PWM(self.Z_SERVO_PIN, self.PWM_FREQUENCY)
            self._clock_pwm = GPIO.PWM(self.CLOCK_SERVO_PIN, self.PWM_FREQUENCY)
            
            self._z_pwm.start(0)
            self._clock_pwm.start(0)
            
            return True
        except Exception as e:
            print(f"[ERROR] Servo setup failed: {e}")
            return False
    
    def teardown(self):
        """Stop PWM."""
        if self._z_pwm:
            self._z_pwm.stop()
        if self._clock_pwm:
            self._clock_pwm.stop()
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Z-axis servo test
            TestStep(
                name="Z-Axis Servo Test",
                display_text="Z SERVO",
                action=self._test_z_servo,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="Z OK?",
                failure_message="Z FAIL"
            ),
            
            # Confirm Z-axis
            TestStep(
                name="Confirm Z-Axis",
                display_text="Z OK?",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="Z CONF",
                failure_message="Z BAD"
            ),
            
            # Clock servo test
            TestStep(
                name="Clock Servo Test",
                display_text="CLK SRV",
                action=self._test_clock_servo,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="CLK OK?",
                failure_message="CLK FAIL"
            ),
            
            # Confirm clock servo
            TestStep(
                name="Confirm Clock Servo",
                display_text="CLK OK?",
                action=lambda: True,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="CLK CONF",
                failure_message="CLK BAD"
            ),
        ]
    
    def _set_servo(self, pwm, duty_cycle: float):
        """Set servo position via PWM duty cycle."""
        if pwm is None:
            print(f"  [MOCK] Set servo to {duty_cycle}% duty cycle")
            time.sleep(0.5)
            return
        
        pwm.ChangeDutyCycle(duty_cycle)
        time.sleep(0.5)  # Wait for servo to reach position
        pwm.ChangeDutyCycle(0)  # Stop sending signal
    
    def _test_z_servo(self) -> bool:
        """Test Z-axis servo movement."""
        print("  Moving Z-servo to UP position...")
        self._set_servo(self._z_pwm, self.Z_UP_PWM)
        time.sleep(0.5)
        
        print("  Moving Z-servo to DOWN position...")
        self._set_servo(self._z_pwm, self.Z_DOWN_PWM)
        time.sleep(0.5)
        
        print("  Moving Z-servo back to UP position...")
        self._set_servo(self._z_pwm, self.Z_UP_PWM)
        
        return True
    
    def _test_clock_servo(self) -> bool:
        """Test clock button servo."""
        print("  Moving clock servo to REST position...")
        self._set_servo(self._clock_pwm, self.CLOCK_REST_PWM)
        time.sleep(0.5)
        
        print("  Moving clock servo to HIT position (pressing button)...")
        self._set_servo(self._clock_pwm, self.CLOCK_HIT_PWM)
        time.sleep(0.3)  # Brief press
        
        print("  Moving clock servo back to REST...")
        self._set_servo(self._clock_pwm, self.CLOCK_REST_PWM)
        
        return True
