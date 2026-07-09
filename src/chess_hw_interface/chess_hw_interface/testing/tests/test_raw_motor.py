#!/usr/bin/env python3
"""
Raw Motor Diagnostic Test — Bare-Metal Motor Movement.

Tests individual motor stepping without CoreXY kinematics.
This is the most fundamental diagnostic for motor issues:
  - Tests Motor A alone (both directions)
  - Tests Motor B alone (both directions)
  - Tests both motors simultaneously
  - Reports ENABLE pin state, DIR pin state, and step counts

Uses pigpio DMA waves directly (no ROS, no kinematics).

Requires:
  sudo pigpiod running
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep
from ..pigpio_stepper import PigpioStepper
from ...gpio_lock import GantryPinLock


class RawMotorTest(HardwareTest):
    """
    Bare-metal motor diagnostic.

    Steps each motor individually to isolate motor/driver issues.
    If Motor A works but Motor B doesn't, the problem is in
    Motor B's wiring or A4988 driver.
    """

    @property
    def name(self) -> str:
        return "Raw Motor"

    @property
    def description(self) -> str:
        return "Test individual motors directly (no CoreXY kinematics)"

    def setup(self) -> bool:
        # This test opens its own pigpio connection straight to the same
        # BCM pins stepper_driver_node/homing_node use — refuse to start if
        # either of those production nodes is currently live, since two
        # independent pigpio clients racing step pulses on the same pins
        # can corrupt motion or damage hardware.
        self._pin_lock = GantryPinLock()
        if not self._pin_lock.acquire_exclusive():
            print("  Setup failed: gantry GPIO pins are in use by a running "
                  "production node (stepper_driver_node/homing_node). "
                  "Stop the live ROS stack before running this test.")
            return False
        try:
            self.stepper = PigpioStepper()
            self.stepper.motor_enable()
            time.sleep(0.1)
            print(f"\n  Motor pins loaded from config:")
            print(f"    Motor A: DIR={self.stepper.motor_a_dir_pin}, STEP={self.stepper.motor_a_step_pin}")
            print(f"    Motor B: DIR={self.stepper.motor_b_dir_pin}, STEP={self.stepper.motor_b_step_pin}")
            print(f"    ENABLE:  {self.stepper.motor_enable_pin}")
            print(f"    Timing:  dir_setup={self.stepper.dir_setup_us}µs, step_pulse={self.stepper.step_pulse_us}µs")
            print(f"    Speed:   min={self.stepper.min_speed}, max={self.stepper.max_speed} steps/sec")
            return True
        except Exception as e:
            print(f"  Setup failed: {e}")
            self._pin_lock.release()
            return False

    def teardown(self):
        if hasattr(self, 'stepper'):
            self.stepper.stop()
            self.stepper.cleanup()
        self._pin_lock.release()

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name="Enable motors",
                display_text="ENABLE",
                action=self._step_enable,
                success_message="ENABLED",
                failure_message="EN FAIL",
            ),
            TestStep(
                name="Motor A — 50 steps forward (slow)",
                display_text="A FWD",
                action=lambda: self._step_single_motor('A', 50, forward=True, speed=20),
                success_message="A OK",
                failure_message="A FAIL",
            ),
            TestStep(
                name="Motor A — 50 steps reverse (slow)",
                display_text="A REV",
                action=lambda: self._step_single_motor('A', 50, forward=False, speed=20),
                success_message="A OK",
                failure_message="A FAIL",
            ),
            TestStep(
                name="Motor B — 50 steps forward (slow)",
                display_text="B FWD",
                action=lambda: self._step_single_motor('B', 50, forward=True, speed=20),
                success_message="B OK",
                failure_message="B FAIL",
            ),
            TestStep(
                name="Motor B — 50 steps reverse (slow)",
                display_text="B REV",
                action=lambda: self._step_single_motor('B', 50, forward=False, speed=20),
                success_message="B OK",
                failure_message="B FAIL",
            ),
            TestStep(
                name="Both motors — 100 steps forward (medium)",
                display_text="BOTH",
                action=lambda: self._step_both_motors(100, speed=40),
                success_message="BOTH OK",
                failure_message="BOTHFAIL",
            ),
            TestStep(
                name="Both motors — 100 steps reverse (medium)",
                display_text="BOTH R",
                action=lambda: self._step_both_motors(-100, speed=40),
                success_message="BOTH OK",
                failure_message="BOTHFAIL",
            ),
            TestStep(
                name="Disable motors",
                display_text="DISABLE",
                action=self._step_disable,
                success_message="DONE",
                failure_message="ERR",
            ),
        ]

    def _step_enable(self) -> bool:
        """Enable motors and verify ENABLE pin is LOW."""
        self.stepper.motor_enable()
        time.sleep(0.05)
        # Read back ENABLE pin — should be LOW (0) when enabled
        en_state = self.stepper.pi.read(self.stepper.motor_enable_pin)
        print(f"    ENABLE pin state: {'LOW (enabled)' if en_state == 0 else 'HIGH (DISABLED!)'}")
        if en_state != 0:
            print("    ERROR: Motors not enabled! Check ENABLE wiring to GPIO 17.")
            return False
        return True

    def _step_single_motor(self, motor: str, steps: int, forward: bool, speed: int) -> bool:
        """
        Step a single motor using DMA waves.

        This bypasses CoreXY kinematics — we step Motor A or B directly.
        """
        import pigpio

        dir_pin = self.stepper.motor_a_dir_pin if motor == 'A' else self.stepper.motor_b_dir_pin
        step_pin = self.stepper.motor_a_step_pin if motor == 'A' else self.stepper.motor_b_step_pin

        # Set direction (LOW = forward for this hardware)
        dir_state = 0 if forward else 1
        self.stepper.pi.write(dir_pin, dir_state)
        time.sleep(self.stepper.dir_setup_us / 1_000_000)

        direction_str = "FWD" if forward else "REV"
        print(f"    Motor {motor}: {steps} steps {direction_str}, DIR pin {dir_pin}={'LOW' if dir_state == 0 else 'HIGH'}")

        # Calculate delay from speed percentage
        target_speed = self.stepper._speed_to_steps_per_sec(speed)
        delay_us = int(1_000_000 / target_speed)
        wait_us = max(1, delay_us - self.stepper.step_pulse_us)

        print(f"    Speed: {target_speed} steps/sec, delay: {delay_us}µs")

        # Generate DMA wave for this motor only
        try:
            self.stepper.pi.wave_clear()

            set_mask = 1 << step_pin
            clear_mask = set_mask

            self.stepper.pi.wave_add_generic([
                pigpio.pulse(set_mask, 0, self.stepper.step_pulse_us),
                pigpio.pulse(0, clear_mask, wait_us),
            ])
            wid = self.stepper.pi.wave_create()
            if wid < 0:
                print(f"    ERROR: wave_create failed ({wid})")
                return False

            # Create wave chain: repeat the wave 'steps' times
            if steps == 1:
                chain = [wid]
            else:
                chain = [255, 0, wid, 255, 1, steps & 0xFF, (steps >> 8) & 0xFF]

            self.stepper.pi.wave_chain(chain)

            # Wait for completion
            timeout = time.time() + 5.0
            while self.stepper.pi.wave_tx_busy():
                if time.time() > timeout:
                    print("    TIMEOUT: Wave still running after 5 seconds")
                    self.stepper.pi.wave_tx_stop()
                    return False
                time.sleep(0.001)

            self.stepper.pi.wave_delete(wid)
            print(f"    Completed {steps} steps successfully")
            time.sleep(0.3)  # Pause between steps so tester can see/hear each motor
            return True

        except Exception as e:
            print(f"    ERROR: {e}")
            try:
                self.stepper.pi.wave_tx_stop()
            except Exception:
                pass
            return False

    def _step_both_motors(self, steps: int, speed: int) -> bool:
        """Step both motors simultaneously at the given speed."""
        abs_steps = abs(steps)
        forward = steps >= 0
        direction_str = "FWD" if forward else "REV"
        print(f"    Both motors: {abs_steps} steps {direction_str}")

        # Use CoreXY same-direction = Y movement (both motors same dir)
        motor_steps = abs_steps if forward else -abs_steps
        try:
            self.stepper.move_xy(0, motor_steps, speed_percent=speed)
            print(f"    Completed {abs_steps} steps on both motors")
            return True
        except Exception as e:
            print(f"    ERROR: {e}")
            return False

    def _step_disable(self) -> bool:
        """Disable motors and verify ENABLE pin is HIGH."""
        self.stepper.motor_disable()
        time.sleep(0.05)
        en_state = self.stepper.pi.read(self.stepper.motor_enable_pin)
        print(f"    ENABLE pin state: {'HIGH (disabled)' if en_state == 1 else 'LOW (still enabled!)'}")
        return en_state == 1
