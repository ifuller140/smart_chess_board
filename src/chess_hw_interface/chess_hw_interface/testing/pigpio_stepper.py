#!/usr/bin/env python3
"""
Pigpio-based stepper motor control for hardware tests.

Uses DMA wave chains for jitter-free, hardware-timed step pulses.
Both motors always step in a SINGLE wave so CoreXY motion is truly
simultaneous — no inter-motor lag.

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

# Wave chain limits — pigpio allows ~250 waves simultaneously.
# We keep headroom for safety.
MAX_UNIQUE_WAVES = 180


class PigpioStepper:
    """
    Pigpio-based stepper motor controller with DMA wave chain generation.

    Both motors always step in the SAME wave pulse, guaranteeing truly
    simultaneous CoreXY motion.  The full speed profile (accel → cruise →
    decel) is pre-computed and uploaded as a wave chain that the DMA engine
    executes in hardware — Python is not in the loop during motion.
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

        # Limit switches: active HIGH with pull-down
        # (when pressed, 5V flows to GPIO → reads HIGH)
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
        """Read X limit switch (active HIGH — pressed = 5V = True)."""
        return self.pi.read(LIMIT_X_PIN) == 1

    def read_y_limit(self) -> bool:
        """Read Y limit switch (active HIGH — pressed = 5V = True)."""
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

    # ------------------------------------------------------------------
    # Wave chain construction — the core fix
    # ------------------------------------------------------------------

    def _build_and_send_wave_chain(
        self,
        abs_a: int,
        abs_b: int,
        profile: List[Tuple[int, int]],
    ):
        """
        Build a wave chain for the entire move and execute it via DMA.

        Uses Bresenham interpolation so both motors step in a single wave
        pulse.  Steps that share the same delay are grouped into a single
        wave ID + loop count, keeping total unique waves well within
        pigpio's ~250 limit.

        Args:
            abs_a: Absolute step count for motor A.
            abs_b: Absolute step count for motor B.
            profile: Speed profile from _calculate_speed_profile().
        """
        max_steps = max(abs_a, abs_b)
        if max_steps == 0:
            return

        # --- Flatten the profile into per-step delays ----------------
        per_step_delays: List[int] = []
        for step_count, delay_us in profile:
            per_step_delays.extend([delay_us] * step_count)

        # --- Bresenham: decide which motors step at each tick ---------
        step_plan: List[Tuple[bool, bool, int]] = []
        err_a = 0
        err_b = 0
        for idx in range(max_steps):
            do_a = False
            do_b = False

            err_a += abs_a
            if err_a >= max_steps:
                err_a -= max_steps
                do_a = True

            err_b += abs_b
            if err_b >= max_steps:
                err_b -= max_steps
                do_b = True

            if do_a or do_b:
                step_plan.append((do_a, do_b, per_step_delays[idx]))

        if not step_plan:
            return

        # --- Group consecutive identical patterns into (pattern, count) --
        groups: List[Tuple[Tuple[bool, bool, int], int]] = []
        current_pattern = (step_plan[0][0], step_plan[0][1], step_plan[0][2])
        count = 1
        for entry in step_plan[1:]:
            pattern = (entry[0], entry[1], entry[2])
            if pattern == current_pattern:
                count += 1
            else:
                groups.append((current_pattern, count))
                current_pattern = pattern
                count = 1
        groups.append((current_pattern, count))

        # --- Create unique waves for each unique pattern ---------------
        wave_ids: List[int] = []
        pattern_to_wid: dict = {}

        try:
            # Clear any leftover waves
            self.pi.wave_clear()

            for pattern, _ in groups:
                if pattern in pattern_to_wid:
                    continue

                do_a, do_b, delay_us = pattern
                wait_us = max(1, delay_us - STEP_PULSE_US)

                set_mask = 0
                clear_mask = 0

                if do_a:
                    set_mask |= (1 << MOTOR_A_STEP_PIN)
                    clear_mask |= (1 << MOTOR_A_STEP_PIN)
                if do_b:
                    set_mask |= (1 << MOTOR_B_STEP_PIN)
                    clear_mask |= (1 << MOTOR_B_STEP_PIN)

                pulses = [
                    pigpio.pulse(set_mask, 0, STEP_PULSE_US),
                    pigpio.pulse(0, clear_mask, wait_us),
                ]
                self.pi.wave_add_generic(pulses)
                wid = self.pi.wave_create()

                if wid < 0:
                    raise RuntimeError(
                        f"pigpio wave_create failed (code {wid}). "
                        "Too many waves or pigpiod resource issue."
                    )

                pattern_to_wid[pattern] = wid
                wave_ids.append(wid)

            # --- Build the chain list ----------------------------------
            chain: List[int] = []
            for pattern, count in groups:
                wid = pattern_to_wid[pattern]
                if count == 1:
                    chain.append(wid)
                else:
                    # pigpio chain loop: 255 0  wid  255 1  loop_lo loop_hi
                    # Loop count is 16-bit little-endian.
                    loop_lo = count & 0xFF
                    loop_hi = (count >> 8) & 0xFF
                    chain.extend([255, 0, wid, 255, 1, loop_lo, loop_hi])

            # --- Send the chain and wait for completion ----------------
            self.pi.wave_chain(chain)

            while self.pi.wave_tx_busy():
                time.sleep(0.001)

        finally:
            # Clean up all waves we created
            for wid in wave_ids:
                try:
                    self.pi.wave_delete(wid)
                except Exception:
                    pass

    def step_both_motors(self, steps_a: int, steps_b: int, speed_percent: Optional[int] = None):
        """
        Step both motors simultaneously using a DMA wave chain.

        The entire motion (acceleration, cruise, deceleration) is built
        as a single wave chain and executed in hardware.  Both motors'
        step edges are always in the same pulse, guaranteeing synchronous
        CoreXY motion.

        Args:
            steps_a: Steps for motor A (negative = reverse)
            steps_b: Steps for motor B (negative = reverse)
            speed_percent: Speed (0-100), or None to use current speed
        """
        if speed_percent is None:
            speed_percent = self._current_speed

        if steps_a == 0 and steps_b == 0:
            return

        # INVERTED DIR pins — positive steps → DIR LOW for this hardware
        self.pi.write(MOTOR_A_DIR_PIN, 0 if steps_a >= 0 else 1)
        self.pi.write(MOTOR_B_DIR_PIN, 0 if steps_b >= 0 else 1)
        time.sleep(DIR_SETUP_US / 1_000_000)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)

        profile = self._calculate_speed_profile(max_steps, speed_percent)

        self.motor_enable()

        try:
            self._build_and_send_wave_chain(abs_a, abs_b, profile)
        finally:
            self.motor_disable()

    # ------------------------------------------------------------------
    # High-level motion methods
    # ------------------------------------------------------------------

    def move_xy(self, dx_steps: int, dy_steps: int, speed_percent: Optional[int] = None):
        """
        Move along both X and Y axes simultaneously (CoreXY).

        This is the primary motion method.  Diagonal, cardinal, and any
        arbitrary vector are all handled by a single wave chain with both
        motors stepping in lock-step.

        CoreXY mapping (Motor A bottom-left, Motor B top-right):
            +X (right): A+, B-
            +Y (up):    A+, B+

        Args:
            dx_steps: Cartesian X motion in steps (positive = right)
            dy_steps: Cartesian Y motion in steps (positive = up)
            speed_percent: Speed (0-100), or None to use current speed
        """
        steps_a = dx_steps + dy_steps
        steps_b = dy_steps - dx_steps
        self.step_both_motors(steps_a, steps_b, speed_percent)
        self._pos_x += dx_steps
        self._pos_y += dy_steps

    def move_x(self, steps: int, speed_percent: Optional[int] = None):
        """
        Move along X axis (CoreXY).

        +X (right): A+, B-
        """
        self.move_xy(steps, 0, speed_percent)

    def move_y(self, steps: int, speed_percent: Optional[int] = None):
        """
        Move along Y axis (CoreXY).

        +Y (up): A+, B+
        """
        self.move_xy(0, steps, speed_percent)

    def step_single_motor(self, motor: str, steps: int, speed_percent: Optional[int] = None):
        """Step a single motor (A or B) directly — bypasses CoreXY mapping."""
        if motor.lower() == 'a':
            self.step_both_motors(steps, 0, speed_percent)
        else:
            self.step_both_motors(0, steps, speed_percent)

    # ------------------------------------------------------------------
    # Homing helpers (small-batch moves with limit switch checks)
    # ------------------------------------------------------------------

    def move_x_until_limit(
        self,
        direction: int,
        speed_percent: int = 25,
        batch_size: int = 25,
        max_steps: int = 100_000,
    ) -> bool:
        """
        Move in X until X limit switch triggers or max_steps reached.

        Args:
            direction: +1 for +X, -1 for -X
            speed_percent: Speed for each batch
            batch_size: Steps per DMA batch (checked between batches)
            max_steps: Safety limit

        Returns:
            True if limit triggered, False if max_steps exhausted.
        """
        moved = 0
        sign = 1 if direction > 0 else -1
        while moved < max_steps:
            if self.read_x_limit():
                return True
            chunk = min(batch_size, max_steps - moved)
            self.move_x(sign * chunk, speed_percent)
            moved += chunk
        return self.read_x_limit()

    def move_y_until_limit(
        self,
        direction: int,
        speed_percent: int = 25,
        batch_size: int = 25,
        max_steps: int = 100_000,
    ) -> bool:
        """
        Move in Y until Y limit switch triggers or max_steps reached.

        Returns:
            True if limit triggered, False if max_steps exhausted.
        """
        moved = 0
        sign = 1 if direction > 0 else -1
        while moved < max_steps:
            if self.read_y_limit():
                return True
            chunk = min(batch_size, max_steps - moved)
            self.move_y(sign * chunk, speed_percent)
            moved += chunk
        return self.read_y_limit()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def stop(self):
        """Stop motors and clean up step pins."""
        try:
            self.pi.wave_tx_stop()
        except Exception:
            pass
        self.pi.write(MOTOR_A_STEP_PIN, 0)
        self.pi.write(MOTOR_B_STEP_PIN, 0)
        self.motor_disable()

    def cleanup(self):
        """Clean up resources."""
        self.stop()
        try:
            self.pi.wave_clear()
        except Exception:
            pass
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
