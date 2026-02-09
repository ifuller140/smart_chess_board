#!/usr/bin/env python3
"""
Manual Gantry Control Test.

Interactive control with keyboard for CoreXY gantry diagnostics.
"""

import curses
import time
from typing import List

from ..base_test import HardwareTest, TestStep


class ManualGantryTest(HardwareTest):
    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    MOTOR_ENABLE_PIN = 17

    LIMIT_X_PIN = 10
    LIMIT_Y_PIN = 9

    DIR_SETUP_US = 50
    STEP_PULSE_US = 100
    STEP_DELAY_MS = 8.0
    MIN_STEP_DELAY_MS = 2.0
    MAX_STEP_DELAY_MS = 20.0

    @property
    def name(self) -> str:
        return 'Manual Gantry'

    @property
    def description(self) -> str:
        return 'Interactive gantry control with keyboard (manual diagnostics)'

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

            self._pos_x = 0
            self._pos_y = 0
            self._enabled = True
            self._motor_enable()
            return True
        except Exception as exc:
            print(f'[ERROR] Setup failed: {exc}')
            return False

    def teardown(self):
        if self.gpio:
            try:
                self.gpio.write(self.MOTOR_A_STEP_PIN, False)
                self.gpio.write(self.MOTOR_B_STEP_PIN, False)
                self._motor_disable()
            except Exception:
                pass

    def _motor_enable(self):
        self.gpio.write(self.MOTOR_ENABLE_PIN, False)
        self._enabled = True
        time.sleep(0.001)

    def _motor_disable(self):
        self.gpio.write(self.MOTOR_ENABLE_PIN, True)
        self._enabled = False

    def _read_x_limit(self) -> bool:
        return not self.gpio.read(self.LIMIT_X_PIN)

    def _read_y_limit(self) -> bool:
        return not self.gpio.read(self.LIMIT_Y_PIN)

    def _step_both_motors(self, steps_a: int, steps_b: int):
        if steps_a == 0 and steps_b == 0:
            return

        if not self._enabled:
            self._motor_enable()

        self.gpio.write(self.MOTOR_A_DIR_PIN, steps_a >= 0)
        self.gpio.write(self.MOTOR_B_DIR_PIN, steps_b >= 0)
        time.sleep(self.DIR_SETUP_US / 1_000_000.0)

        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)

        err_a = 0
        err_b = 0

        ramp_steps = min(20, max_steps // 2) if max_steps > 2 else 0

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

            delay_ms = self.STEP_DELAY_MS
            if ramp_steps > 0:
                if idx < ramp_steps:
                    ratio = (ramp_steps - idx) / ramp_steps
                elif idx >= (max_steps - ramp_steps):
                    ratio = (idx - (max_steps - ramp_steps - 1)) / ramp_steps
                else:
                    ratio = 0.0
                delay_ms = delay_ms + (self.MAX_STEP_DELAY_MS - delay_ms) * ratio * 0.7
            time.sleep(delay_ms / 1000.0)

    def _move_x(self, steps: int):
        self._step_both_motors(steps, -steps)
        self._pos_x += steps

    def _move_y(self, steps: int):
        self._step_both_motors(steps, steps)
        self._pos_y += steps

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name='Manual Gantry Control',
                display_text='GANTRY',
                action=self._run_manual_control,
                wait_for_input=False,
                success_message='DONE',
                failure_message='FAIL',
            )
        ]

    def _run_manual_control(self) -> bool:
        print('\nStarting manual gantry control in 2 seconds...')
        print("Controls: arrows move, +/- step size, [/] speed, 'e' toggle motor enable, 'q' quit")
        time.sleep(2)

        try:
            curses.wrapper(self._control_loop)
            return True
        except Exception as exc:
            print(f'\n[ERROR] Control loop error: {exc}')
            return False

    def _control_loop(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)

        steps_per_tick = 25
        min_steps = 5
        max_steps = 100

        running = True

        while running:
            stdscr.clear()
            stdscr.addstr(0, 0, '=' * 62)
            stdscr.addstr(1, 0, '  MANUAL GANTRY CONTROL')
            stdscr.addstr(2, 0, '=' * 62)
            stdscr.addstr(4, 0, f'Position: X={self._pos_x:6d}  Y={self._pos_y:6d} steps')
            stdscr.addstr(5, 0, f'Step size: {steps_per_tick} steps/tick')
            stdscr.addstr(6, 0, f'Step delay: {self.STEP_DELAY_MS:4.1f} ms (lower=faster)')
            stdscr.addstr(7, 0, f'Motor enabled: {self._enabled}')
            stdscr.addstr(8, 0, f'Limits: X={self._read_x_limit()}  Y={self._read_y_limit()}')

            stdscr.addstr(10, 0, 'Direction map:')
            stdscr.addstr(11, 0, 'Right(+X): A+, B-')
            stdscr.addstr(12, 0, 'Left(-X):  A-, B+')
            stdscr.addstr(13, 0, 'Up(+Y):    A+, B+')
            stdscr.addstr(14, 0, 'Down(-Y):  A-, B-')

            stdscr.addstr(16, 0, 'Controls:')
            stdscr.addstr(17, 2, 'Arrows: move gantry')
            stdscr.addstr(18, 2, '+/-: adjust step size')
            stdscr.addstr(19, 2, '[ / ]: slower/faster stepping')
            stdscr.addstr(20, 2, 'e: toggle enable/disable motors')
            stdscr.addstr(21, 2, 'q: quit')
            stdscr.refresh()

            key = stdscr.getch()

            if key in (ord('q'), ord('Q')):
                running = False
                continue

            if key in (ord('e'), ord('E')):
                if self._enabled:
                    self._motor_disable()
                else:
                    self._motor_enable()
                continue

            if key in (ord('+'), ord('=')):
                steps_per_tick = min(max_steps, steps_per_tick + 5)
            elif key in (ord('-'), ord('_')):
                steps_per_tick = max(min_steps, steps_per_tick - 5)
            elif key == ord('['):
                self.STEP_DELAY_MS = min(self.MAX_STEP_DELAY_MS, self.STEP_DELAY_MS + 0.5)
            elif key == ord(']'):
                self.STEP_DELAY_MS = max(self.MIN_STEP_DELAY_MS, self.STEP_DELAY_MS - 0.5)
            elif key == curses.KEY_RIGHT:
                self._move_x(steps_per_tick)
            elif key == curses.KEY_LEFT:
                if not self._read_x_limit():
                    self._move_x(-steps_per_tick)
            elif key == curses.KEY_UP:
                self._move_y(steps_per_tick)
            elif key == curses.KEY_DOWN:
                if not self._read_y_limit():
                    self._move_y(-steps_per_tick)

        stdscr.addstr(23, 0, 'Exiting manual control...')
        stdscr.refresh()
        time.sleep(0.5)
