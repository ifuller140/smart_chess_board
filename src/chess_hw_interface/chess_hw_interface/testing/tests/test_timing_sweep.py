#!/usr/bin/env python3
"""
Motor Timing Sweep Test — Find Working Speed Range.

Sweeps step rate from very slow to fast to find the speed range
where the motors actually move. This helps diagnose:
  - Resonance frequencies (speeds where motor stalls)
  - Maximum practical speed before step loss
  - Minimum speed that produces movement

Uses pigpio DMA waves directly (no ROS).

Requires:
  sudo pigpiod running
"""

import time
from typing import List

from ..base_test import HardwareTest, TestStep
from ..pigpio_stepper import PigpioStepper


class TimingSweepTest(HardwareTest):
    """
    Sweep motor speeds to find the workable range.

    Tests both motors at increasing speeds and reports
    success/failure at each speed level.
    """

    # Speed levels to test (steps per second)
    SPEED_LEVELS = [50, 100, 150, 200, 300, 400, 600, 800, 1000, 1200]
    STEPS_PER_LEVEL = 100  # Steps at each speed level

    @property
    def name(self) -> str:
        return "Timing Sweep"

    @property
    def description(self) -> str:
        return "Sweep motor speeds to find working range and resonance"

    def setup(self) -> bool:
        try:
            self.stepper = PigpioStepper()
            self.stepper.motor_enable()
            time.sleep(0.1)
            print(f"\n  Config: dir_setup={self.stepper.dir_setup_us}µs, step_pulse={self.stepper.step_pulse_us}µs")
            print(f"  Testing speeds: {self.SPEED_LEVELS} steps/sec")
            print(f"  Steps per level: {self.STEPS_PER_LEVEL}")
            self._results = {}
            return True
        except Exception as e:
            print(f"  Setup failed: {e}")
            return False

    def teardown(self):
        if hasattr(self, 'stepper'):
            self.stepper.stop()
            self.stepper.cleanup()

        # Print summary table
        if hasattr(self, '_results') and self._results:
            print(f"\n{'='*50}")
            print("  TIMING SWEEP RESULTS")
            print(f"{'='*50}")
            print(f"  {'Speed (sps)':<15} {'Status':<12} {'Notes'}")
            print(f"  {'-'*45}")
            for speed, (ok, note) in sorted(self._results.items()):
                status = "✓ OK" if ok else "✗ FAIL"
                print(f"  {speed:<15} {status:<12} {note}")
            print(f"{'='*50}")

    def get_steps(self) -> List[TestStep]:
        steps = [
            TestStep(
                name="Enable motors",
                display_text="ENABLE",
                action=self._enable,
                success_message="ENABLED",
                failure_message="EN FAIL",
            ),
        ]

        for speed in self.SPEED_LEVELS:
            steps.append(TestStep(
                name=f"Speed test: {speed} steps/sec",
                display_text=f"S {speed:4d}",
                action=lambda s=speed: self._test_speed(s),
                success_message=f"{speed} OK",
                failure_message=f"{speed}FAIL",
            ))

        steps.append(TestStep(
            name="Disable motors",
            display_text="DISABLE",
            action=self._disable,
            success_message="DONE",
            failure_message="ERR",
        ))

        return steps

    def _enable(self) -> bool:
        self.stepper.motor_enable()
        time.sleep(0.05)
        return self.stepper.pi.read(self.stepper.motor_enable_pin) == 0

    def _disable(self) -> bool:
        self.stepper.motor_disable()
        return True

    def _test_speed(self, speed_sps: int) -> bool:
        """
        Test motor at a specific speed (steps per second).

        Steps both motors simultaneously for STEPS_PER_LEVEL steps,
        then reverses to return to starting position.
        """
        import pigpio

        delay_us = int(1_000_000 / speed_sps)
        wait_us = max(1, delay_us - self.stepper.step_pulse_us)
        total_steps = self.STEPS_PER_LEVEL

        print(f"    Testing {speed_sps} sps (delay={delay_us}µs, {total_steps} steps)...")

        try:
            # Set direction: forward (both motors same direction = Y movement)
            self.stepper.pi.write(self.stepper.motor_a_dir_pin, 0)
            self.stepper.pi.write(self.stepper.motor_b_dir_pin, 0)
            time.sleep(self.stepper.dir_setup_us / 1_000_000)

            # Build wave for both motors simultaneously
            self.stepper.pi.wave_clear()

            set_mask = (1 << self.stepper.motor_a_step_pin) | (1 << self.stepper.motor_b_step_pin)
            clear_mask = set_mask

            self.stepper.pi.wave_add_generic([
                pigpio.pulse(set_mask, 0, self.stepper.step_pulse_us),
                pigpio.pulse(0, clear_mask, wait_us),
            ])
            wid = self.stepper.pi.wave_create()
            if wid < 0:
                self._results[speed_sps] = (False, f"wave_create failed ({wid})")
                return False  # Non-fatal: continue sweep

            # Chain: repeat wave total_steps times
            chain = [255, 0, wid, 255, 1, total_steps & 0xFF, (total_steps >> 8) & 0xFF]

            expected_time = total_steps * delay_us / 1_000_000
            start = time.time()
            self.stepper.pi.wave_chain(chain)

            # Wait for completion
            timeout = time.time() + max(expected_time * 3, 2.0)
            while self.stepper.pi.wave_tx_busy():
                if time.time() > timeout:
                    self.stepper.pi.wave_tx_stop()
                    self._results[speed_sps] = (False, "timeout (too slow or stalled)")
                    print(f"    TIMEOUT at {speed_sps} sps")
                    self.stepper.pi.wave_delete(wid)
                    return True  # Don't stop the sweep

                time.sleep(0.001)

            elapsed = time.time() - start
            actual_sps = total_steps / elapsed if elapsed > 0 else 0

            self.stepper.pi.wave_delete(wid)

            # Now reverse to return to start
            self.stepper.pi.write(self.stepper.motor_a_dir_pin, 1)
            self.stepper.pi.write(self.stepper.motor_b_dir_pin, 1)
            time.sleep(self.stepper.dir_setup_us / 1_000_000)

            self.stepper.pi.wave_clear()
            self.stepper.pi.wave_add_generic([
                pigpio.pulse(set_mask, 0, self.stepper.step_pulse_us),
                pigpio.pulse(0, clear_mask, wait_us),
            ])
            wid = self.stepper.pi.wave_create()
            if wid >= 0:
                chain = [255, 0, wid, 255, 1, total_steps & 0xFF, (total_steps >> 8) & 0xFF]
                self.stepper.pi.wave_chain(chain)
                while self.stepper.pi.wave_tx_busy():
                    time.sleep(0.001)
                self.stepper.pi.wave_delete(wid)

            self._results[speed_sps] = (True, f"actual={actual_sps:.0f}sps, {elapsed:.3f}s")
            print(f"    OK: {speed_sps} sps target, {actual_sps:.0f} sps actual ({elapsed:.3f}s)")
            time.sleep(0.2)  # Brief pause between speed levels
            return True

        except Exception as e:
            self._results[speed_sps] = (False, str(e))
            print(f"    ERROR at {speed_sps} sps: {e}")
            return True  # Don't stop the sweep
