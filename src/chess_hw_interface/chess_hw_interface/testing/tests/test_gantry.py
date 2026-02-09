#!/usr/bin/env python3
"""
Gantry hardware tests for CoreXY (A4988 + NEMA 11).

The suite is ordered from basic wiring/timing validation to full motion stress.
"""

import time
from statistics import mean
from typing import List

from ..base_test import HardwareTest, TestStep


class GantryTestBase(HardwareTest):
    """Shared pin map and movement helpers for gantry tests."""

    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    MOTOR_ENABLE_PIN = 17  # A4988 ENABLE, active LOW

    LIMIT_X_PIN = 10
    LIMIT_Y_PIN = 9

    DIR_SETUP_US = 50
    STEP_PULSE_US = 100

    MIN_DELAY_MS = 5.0
    MAX_DELAY_MS = 50.0
    DEFAULT_RAMP_STEPS = 30

    def setup(self) -> bool:
        if self.gpio is None:
            raise RuntimeError('GPIO interface required - hardware must be connected')

        try:
            for pin in [
                self.MOTOR_A_DIR_PIN,
                self.MOTOR_A_STEP_PIN,
                self.MOTOR_B_DIR_PIN,
                self.MOTOR_B_STEP_PIN,
                self.MOTOR_ENABLE_PIN,
            ]:
                self.gpio.setup_output(pin)

            self.gpio.setup_input(self.LIMIT_X_PIN, pull_up=True)
            self.gpio.setup_input(self.LIMIT_Y_PIN, pull_up=True)

            self._disable_motors()
            self.gpio.write(self.MOTOR_A_STEP_PIN, False)
            self.gpio.write(self.MOTOR_B_STEP_PIN, False)
            self._pos_x_steps = 0
            self._pos_y_steps = 0
            return True
        except Exception as exc:
            print(f'[ERROR] Gantry setup failed: {exc}')
            return False

    def teardown(self):
        try:
            self.gpio.write(self.MOTOR_A_STEP_PIN, False)
            self.gpio.write(self.MOTOR_B_STEP_PIN, False)
            self._disable_motors()
        except Exception:
            pass

    def _enable_motors(self):
        self.gpio.write(self.MOTOR_ENABLE_PIN, False)
        time.sleep(0.001)

    def _disable_motors(self):
        self.gpio.write(self.MOTOR_ENABLE_PIN, True)

    def _read_x_limit(self) -> bool:
        return self.gpio.read_x_limit()

    def _read_y_limit(self) -> bool:
        return self.gpio.read_y_limit()

    def _speed_to_delay_sec(self, speed_percent: int) -> float:
        speed = max(0, min(100, speed_percent))
        delay_ms = self.MAX_DELAY_MS - (speed / 100.0) * (self.MAX_DELAY_MS - self.MIN_DELAY_MS)
        return delay_ms / 1000.0

    def _ramped_delay(self, step_idx: int, total_steps: int, cruise_delay: float, ramp_steps: int) -> float:
        if total_steps < 3 or ramp_steps <= 0:
            return cruise_delay

        ramp = min(ramp_steps, total_steps // 2)
        if ramp <= 0:
            return cruise_delay

        if step_idx < ramp:
            ratio = (ramp - step_idx) / ramp
        elif step_idx >= (total_steps - ramp):
            ratio = (step_idx - (total_steps - ramp - 1)) / ramp
        else:
            ratio = 0.0

        slowest_delay = self.MAX_DELAY_MS / 1000.0
        return cruise_delay + (slowest_delay - cruise_delay) * ratio * 0.8

    def _step_both(self, steps_a: int, steps_b: int, speed_percent: int = 30, ramp_steps: int = DEFAULT_RAMP_STEPS):
        if steps_a == 0 and steps_b == 0:
            return

        self._enable_motors()
        self.gpio.write(self.MOTOR_A_DIR_PIN, steps_a >= 0)
        self.gpio.write(self.MOTOR_B_DIR_PIN, steps_b >= 0)
        time.sleep(self.DIR_SETUP_US / 1_000_000.0)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        cruise_delay = self._speed_to_delay_sec(speed_percent)

        err_a = 0
        err_b = 0

        for idx in range(max_steps):
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

            if step_a:
                self.gpio.write(self.MOTOR_A_STEP_PIN, True)
            if step_b:
                self.gpio.write(self.MOTOR_B_STEP_PIN, True)

            time.sleep(self.STEP_PULSE_US / 1_000_000.0)
            self.gpio.write(self.MOTOR_A_STEP_PIN, False)
            self.gpio.write(self.MOTOR_B_STEP_PIN, False)

            delay = self._ramped_delay(idx, max_steps, cruise_delay, ramp_steps)
            if delay > (self.STEP_PULSE_US / 1_000_000.0):
                time.sleep(delay - (self.STEP_PULSE_US / 1_000_000.0))

    def _step_single(self, motor: str, steps: int, speed_percent: int = 30):
        if motor == 'a':
            self._step_both(steps, 0, speed_percent)
        else:
            self._step_both(0, steps, speed_percent)

    def _move_x(self, steps: int, speed_percent: int = 30):
        # +X => A+, B-
        self._step_both(steps, -steps, speed_percent)
        self._pos_x_steps += steps

    def _move_y(self, steps: int, speed_percent: int = 30):
        # +Y => A+, B+
        self._step_both(steps, steps, speed_percent)
        self._pos_y_steps += steps

    def _timed_pulse_train(self, step_pin: int, pulses: int, delay_us: int) -> List[float]:
        self._enable_motors()
        intervals_ms: List[float] = []
        last = time.perf_counter()

        for _ in range(pulses):
            self.gpio.write(step_pin, True)
            time.sleep(self.STEP_PULSE_US / 1_000_000.0)
            self.gpio.write(step_pin, False)

            now = time.perf_counter()
            intervals_ms.append((now - last) * 1000.0)
            last = now
            time.sleep(max(0.0, delay_us / 1_000_000.0))

        return intervals_ms


class GantryLimitSwitchTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Limits'

    @property
    def description(self) -> str:
        return 'Validate X/Y limit wiring and active-low polarity'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name='X limit press',
                display_text='PRESS X',
                action=lambda: True,
                wait_for_input=True,
                input_type='x_limit',
                timeout_seconds=45.0,
                success_message='X HIT',
                failure_message='X FAIL',
            ),
            TestStep(
                name='Y limit press',
                display_text='PRESS Y',
                action=lambda: True,
                wait_for_input=True,
                input_type='y_limit',
                timeout_seconds=45.0,
                success_message='Y HIT',
                failure_message='Y FAIL',
            ),
        ]


class GantryPulseIntegrityTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Pulse Integrity'

    @property
    def description(self) -> str:
        return 'Measure pulse train timing jitter from Python loop execution'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Pulse train A', 'PULS A', self._pulse_a, False, success_message='A DONE', failure_message='A ERR'),
            TestStep('Pulse train B', 'PULS B', self._pulse_b, False, success_message='B DONE', failure_message='B ERR'),
            TestStep('Confirm sound', 'SMOOTH?', lambda: True, True, 'clock', 45.0, 'OK', 'BAD'),
        ]

    def _report(self, label: str, intervals_ms: List[float]):
        if not intervals_ms:
            print(f'  [ERROR] {label}: no intervals captured')
            return

        avg = mean(intervals_ms)
        jitter = max(intervals_ms) - min(intervals_ms)
        print(
            f'  {label}: avg={avg:.3f}ms '
            f'min={min(intervals_ms):.3f}ms '
            f'max={max(intervals_ms):.3f}ms '
            f'pkpk_jitter={jitter:.3f}ms'
        )
        if jitter > 2.0:
            print('  [WARN] High software timing jitter detected. Prefer pigpio DMA pulse generation for production.')

    def _pulse_a(self) -> bool:
        self.gpio.write(self.MOTOR_A_DIR_PIN, True)
        intervals = self._timed_pulse_train(self.MOTOR_A_STEP_PIN, pulses=300, delay_us=3000)
        self._report('Motor A pulse train', intervals)
        return True

    def _pulse_b(self) -> bool:
        self.gpio.write(self.MOTOR_B_DIR_PIN, True)
        intervals = self._timed_pulse_train(self.MOTOR_B_STEP_PIN, pulses=300, delay_us=3000)
        self._report('Motor B pulse train', intervals)
        return True


class GantryMotorATest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Motor A'

    @property
    def description(self) -> str:
        return 'Single-motor direction and pulse test for Motor A'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Move A slow', 'A SLOW', self._slow, False, success_message='A1 OK', failure_message='A1 ER'),
            TestStep('Move A mid', 'A MID', self._mid, False, success_message='A2 OK', failure_message='A2 ER'),
            TestStep('Confirm A', 'A OK?', lambda: True, True, 'clock', 30.0, 'A OK', 'A BAD'),
        ]

    def _slow(self) -> bool:
        self._step_single('a', 300, 25)
        time.sleep(0.2)
        self._step_single('a', -300, 25)
        return True

    def _mid(self) -> bool:
        self._step_single('a', 300, 45)
        time.sleep(0.2)
        self._step_single('a', -300, 45)
        return True


class GantryMotorBTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Motor B'

    @property
    def description(self) -> str:
        return 'Single-motor direction and pulse test for Motor B'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Move B slow', 'B SLOW', self._slow, False, success_message='B1 OK', failure_message='B1 ER'),
            TestStep('Move B mid', 'B MID', self._mid, False, success_message='B2 OK', failure_message='B2 ER'),
            TestStep('Confirm B', 'B OK?', lambda: True, True, 'clock', 30.0, 'B OK', 'B BAD'),
        ]

    def _slow(self) -> bool:
        self._step_single('b', 300, 25)
        time.sleep(0.2)
        self._step_single('b', -300, 25)
        return True

    def _mid(self) -> bool:
        self._step_single('b', 300, 45)
        time.sleep(0.2)
        self._step_single('b', -300, 45)
        return True


class GantryCoreXYDirectionTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry CoreXY'

    @property
    def description(self) -> str:
        return 'Verify +X/-X/+Y/-Y direction mapping for both motors'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Move +X', '+X', self._plus_x, False, success_message='X+', failure_message='X+ER'),
            TestStep('Confirm +X', 'X+ OK?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
            TestStep('Move -X', '-X', self._minus_x, False, success_message='X-', failure_message='X-ER'),
            TestStep('Confirm -X', 'X- OK?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
            TestStep('Move +Y', '+Y', self._plus_y, False, success_message='Y+', failure_message='Y+ER'),
            TestStep('Confirm +Y', 'Y+ OK?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
            TestStep('Move -Y', '-Y', self._minus_y, False, success_message='Y-', failure_message='Y-ER'),
            TestStep('Confirm -Y', 'Y- OK?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
        ]

    def _plus_x(self) -> bool:
        self._move_x(300, 35)
        return True

    def _minus_x(self) -> bool:
        self._move_x(-300, 35)
        return True

    def _plus_y(self) -> bool:
        self._move_y(300, 35)
        return True

    def _minus_y(self) -> bool:
        self._move_y(-300, 35)
        return True


class GantrySpeedSweepTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Speed Sweep'

    @property
    def description(self) -> str:
        return 'Run same motion at multiple speeds to detect stall zones'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Sweep X', 'SWP X', self._run_x, False, success_message='X DONE', failure_message='X ERR'),
            TestStep('Sweep Y', 'SWP Y', self._run_y, False, success_message='Y DONE', failure_message='Y ERR'),
            TestStep('Confirm sweep', 'SMTH?', lambda: True, True, 'clock', 45.0, 'OK', 'BAD'),
        ]

    def _run_x(self) -> bool:
        for speed in [20, 30, 40, 50, 60, 70]:
            print(f'  X sweep speed {speed}%: +220/-220')
            self._move_x(220, speed)
            time.sleep(0.2)
            self._move_x(-220, speed)
            time.sleep(0.3)
        return True

    def _run_y(self) -> bool:
        for speed in [20, 30, 40, 50, 60, 70]:
            print(f'  Y sweep speed {speed}%: +220/-220')
            self._move_y(220, speed)
            time.sleep(0.2)
            self._move_y(-220, speed)
            time.sleep(0.3)
        return True


class GantryEnableHoldTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Enable/Hold'

    @property
    def description(self) -> str:
        return 'Verify A4988 ENABLE pin and holding torque behavior'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Disable hold', 'FREE', self._disable_phase, False, success_message='FREE', failure_message='ERR'),
            TestStep('Confirm free', 'FREE?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
            TestStep('Enable hold', 'HOLD', self._enable_phase, False, success_message='HOLD', failure_message='ERR'),
            TestStep('Confirm hold', 'HOLD?', lambda: True, True, 'clock', 30.0, 'OK', 'BAD'),
        ]

    def _disable_phase(self) -> bool:
        print('  Motors disabled for 4s. Gantry should move freely by hand.')
        self._disable_motors()
        time.sleep(4.0)
        return True

    def _enable_phase(self) -> bool:
        print('  Motors enabled for 4s. Gantry should resist manual movement.')
        self._enable_motors()
        time.sleep(4.0)
        return True


class GantryHomingTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Homing'

    @property
    def description(self) -> str:
        return 'Manual homing sequence with staged approach/backoff'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Start homing', 'HOME', self._run_homing, False, success_message='HOME', failure_message='FAIL'),
            TestStep('Confirm home', 'HOME?', lambda: True, True, 'clock', 45.0, 'OK', 'BAD'),
        ]

    def _run_homing(self) -> bool:
        self._move_x(300, 35)
        self._move_y(300, 35)

        x_steps = 0
        while not self._read_x_limit() and x_steps < 100000:
            self._move_x(-25, 35)
            x_steps += 25
        if not self._read_x_limit():
            print('  [ERROR] X limit not reached')
            return False

        self._move_x(120, 25)
        while not self._read_x_limit():
            self._move_x(-2, 20)

        y_steps = 0
        while not self._read_y_limit() and y_steps < 100000:
            self._move_y(-25, 35)
            y_steps += 25
        if not self._read_y_limit():
            print('  [ERROR] Y limit not reached')
            return False

        self._move_y(120, 25)
        while not self._read_y_limit():
            self._move_y(-2, 20)

        self._pos_x_steps = 0
        self._pos_y_steps = 0
        return True


class GantryRepeatabilityTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Repeatability'

    @property
    def description(self) -> str:
        return 'Repeat square pattern to expose intermittent stalls/skips'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Square loops', 'LOOPS', self._run_loops, False, success_message='DONE', failure_message='ERR'),
            TestStep('Confirm return', 'BACK?', lambda: True, True, 'clock', 45.0, 'OK', 'BAD'),
        ]

    def _run_loops(self) -> bool:
        loops = 6
        side = 180
        for idx in range(loops):
            print(f'  Loop {idx + 1}/{loops}')
            self._move_x(side, 45)
            self._move_y(side, 45)
            self._move_x(-side, 45)
            self._move_y(-side, 45)
            time.sleep(0.2)
        return True


class GantryFullTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Gantry Full'

    @property
    def description(self) -> str:
        return 'Run full gantry diagnostics: wiring, timing, motion, and stress'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Limits ready', 'LIMITS', self._limits_prompt, False, success_message='NEXT', failure_message='FAIL'),
            TestStep('Press X', 'PRESS X', lambda: True, True, 'x_limit', 45.0, 'X HIT', 'X FAIL'),
            TestStep('Press Y', 'PRESS Y', lambda: True, True, 'y_limit', 45.0, 'Y HIT', 'Y FAIL'),
            TestStep('Pulse diag', 'PULSE', self._pulse_diag, False, success_message='P OK', failure_message='P ER'),
            TestStep('Motor A', 'MOT A', self._run_motor_a, False, success_message='A OK', failure_message='A ER'),
            TestStep('Motor B', 'MOT B', self._run_motor_b, False, success_message='B OK', failure_message='B ER'),
            TestStep('CoreXY', 'COREXY', self._run_corexy, False, success_message='D OK', failure_message='D ER'),
            TestStep('Speed sweep', 'SPEED', self._run_sweep, False, success_message='S OK', failure_message='S ER'),
            TestStep('Repeat loops', 'LOOPS', self._run_loops, False, success_message='R OK', failure_message='R ER'),
            TestStep('Hold test', 'HOLD', self._run_hold, False, success_message='H OK', failure_message='H ER'),
            TestStep('Final confirm', 'GOOD?', lambda: True, True, 'clock', 45.0, 'PASS', 'FAIL'),
        ]

    def _limits_prompt(self) -> bool:
        print('  Limit test: press X limit then Y limit when prompted.')
        return True

    def _pulse_diag(self) -> bool:
        intervals = self._timed_pulse_train(self.MOTOR_A_STEP_PIN, pulses=180, delay_us=3500)
        avg = mean(intervals) if intervals else 0.0
        jitter = (max(intervals) - min(intervals)) if intervals else 0.0
        print(f'  Pulse diag: avg={avg:.3f}ms, jitter={jitter:.3f}ms')
        return True

    def _run_motor_a(self) -> bool:
        self._step_single('a', 250, 35)
        self._step_single('a', -250, 35)
        return True

    def _run_motor_b(self) -> bool:
        self._step_single('b', 250, 35)
        self._step_single('b', -250, 35)
        return True

    def _run_corexy(self) -> bool:
        self._move_x(200, 35)
        self._move_x(-200, 35)
        self._move_y(200, 35)
        self._move_y(-200, 35)
        return True

    def _run_sweep(self) -> bool:
        for speed in [20, 35, 50, 65]:
            self._move_x(150, speed)
            self._move_x(-150, speed)
            time.sleep(0.1)
        return True

    def _run_loops(self) -> bool:
        for _ in range(3):
            self._move_x(120, 40)
            self._move_y(120, 40)
            self._move_x(-120, 40)
            self._move_y(-120, 40)
        return True

    def _run_hold(self) -> bool:
        self._disable_motors()
        time.sleep(2.0)
        self._enable_motors()
        time.sleep(2.0)
        return True
