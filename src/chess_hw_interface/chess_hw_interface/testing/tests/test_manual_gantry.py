#!/usr/bin/env python3
"""
Manual Gantry Control Test.

Interactive control with keyboard for CoreXY gantry diagnostics.
Uses pigpio DMA waves for jitter-free stepping.
Supports diagonal motion via move_xy() — both motors step in a
single wave even for combined X+Y movement.
"""

import curses
import time
from typing import List, Optional

from ..base_test import HardwareTest, TestStep
from ..pigpio_stepper import PigpioStepper


class ManualGantryTest(HardwareTest):
    """Interactive manual gantry control using pigpio DMA."""

    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    MOTOR_ENABLE_PIN = 17

    LIMIT_X_PIN = 10
    LIMIT_Y_PIN = 9

    @property
    def name(self) -> str:
        return 'Manual Gantry'

    @property
    def description(self) -> str:
        return 'Interactive gantry control with keyboard (pigpio DMA stepping, diagonal support)'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._stepper: Optional[PigpioStepper] = None
        self._pos_x = 0
        self._pos_y = 0
        self._enabled = False
        self._speed_percent = 40

    def setup(self) -> bool:
        """Initialize pigpio-based stepper controller."""
        try:
            self._stepper = PigpioStepper()
            self._pos_x = 0
            self._pos_y = 0
            self._enabled = True
            self._stepper.motor_enable()
            return True
        except Exception as exc:
            print(f'[ERROR] Setup failed: {exc}')
            print('Make sure pigpiod is running: sudo pigpiod')
            return False

    def teardown(self):
        """Clean up pigpio resources."""
        if self._stepper:
            try:
                self._stepper.cleanup()
            except Exception:
                pass
            self._stepper = None

    def _motor_enable(self):
        if self._stepper:
            self._stepper.motor_enable()
            self._enabled = True

    def _motor_disable(self):
        if self._stepper:
            self._stepper.motor_disable()
            self._enabled = False

    def _read_x_limit(self) -> bool:
        """Read X limit switch (active HIGH — pressed = 5V)."""
        if self._stepper:
            return self._stepper.read_x_limit()
        return False

    def _read_y_limit(self) -> bool:
        """Read Y limit switch (active HIGH — pressed = 5V)."""
        if self._stepper:
            return self._stepper.read_y_limit()
        return False

    def _move_xy(self, dx: int, dy: int):
        """Move along X and Y simultaneously using a single wave chain."""
        if self._stepper:
            self._stepper.move_xy(dx, dy, self._speed_percent)
            self._pos_x += dx
            self._pos_y += dy

    def _move_x(self, steps: int):
        """Move along X axis using pigpio DMA."""
        self._move_xy(steps, 0)

    def _move_y(self, steps: int):
        """Move along Y axis using pigpio DMA."""
        self._move_xy(0, steps)

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
        print("Controls: arrows move, shift+arrows diagonal, +/- step size, [/] speed, 'e' toggle, 'q' quit")
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
            stdscr.addstr(1, 0, '  MANUAL GANTRY CONTROL (pigpio DMA, sync waves)')
            stdscr.addstr(2, 0, '=' * 62)
            stdscr.addstr(4, 0, f'Position: X={self._pos_x:6d}  Y={self._pos_y:6d} steps')
            stdscr.addstr(5, 0, f'Step size: {steps_per_tick} steps/tick')
            stdscr.addstr(6, 0, f'Speed: {self._speed_percent}%')
            stdscr.addstr(7, 0, f'Motor enabled: {self._enabled}')
            stdscr.addstr(8, 0, f'Limits: X={self._read_x_limit()}  Y={self._read_y_limit()}')

            stdscr.addstr(10, 0, 'CoreXY direction map:')
            stdscr.addstr(11, 0, '  Right(+X): A+, B-     Left(-X): A-, B+')
            stdscr.addstr(12, 0, '  Up(+Y):    A+, B+     Down(-Y): A-, B-')
            stdscr.addstr(13, 0, '  Diagonal:  single wave, both motors in lock-step')

            stdscr.addstr(15, 0, 'Controls:')
            stdscr.addstr(16, 2, 'Arrows: cardinal moves (single wave)')
            stdscr.addstr(17, 2, 'w/a/s/d: cardinal moves (alt keys)')
            stdscr.addstr(18, 2, 'Numpad 7/9/1/3: diagonal moves')
            stdscr.addstr(19, 2, '+/-: adjust step size')
            stdscr.addstr(20, 2, '[ / ]: decrease/increase speed')
            stdscr.addstr(21, 2, 'e: toggle enable/disable motors')
            stdscr.addstr(22, 2, 'q: quit')
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
                self._speed_percent = max(10, self._speed_percent - 5)
            elif key == ord(']'):
                self._speed_percent = min(100, self._speed_percent + 5)

            # Cardinal moves
            elif key == curses.KEY_RIGHT or key in (ord('d'), ord('D')):
                self._move_x(steps_per_tick)
            elif key == curses.KEY_LEFT or key in (ord('a'), ord('A')):
                if not self._read_x_limit():
                    self._move_x(-steps_per_tick)
            elif key == curses.KEY_UP or key in (ord('w'), ord('W')):
                self._move_y(steps_per_tick)
            elif key == curses.KEY_DOWN or key in (ord('s'), ord('S')):
                if not self._read_y_limit():
                    self._move_y(-steps_per_tick)

            # Diagonal moves (numpad or number keys)
            # 7 = up-left, 9 = up-right, 1 = down-left, 3 = down-right
            elif key == ord('9'):
                # Up-right diagonal
                self._move_xy(steps_per_tick, steps_per_tick)
            elif key == ord('7'):
                # Up-left diagonal
                if not self._read_x_limit():
                    self._move_xy(-steps_per_tick, steps_per_tick)
            elif key == ord('3'):
                # Down-right diagonal
                if not self._read_y_limit():
                    self._move_xy(steps_per_tick, -steps_per_tick)
            elif key == ord('1'):
                # Down-left diagonal
                if not self._read_x_limit() and not self._read_y_limit():
                    self._move_xy(-steps_per_tick, -steps_per_tick)

        stdscr.addstr(24, 0, 'Exiting manual control...')
        stdscr.refresh()
        time.sleep(0.5)
