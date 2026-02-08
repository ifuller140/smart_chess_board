#!/usr/bin/env python3
"""
Manual Gantry Control Test for Hardware Test Suite.

Interactive curses-based gantry control with:
- Arrow key movement from player perspective (sitting at white's side)
- Real-time position display
- Stepper motor direction chart
- Adjustable step size

CoreXY Motor Layout:
    - Motor A: Bottom-left (BCM 27 dir, 22 step)
    - Motor B: Top-right (BCM 6 dir, 5 step)

Player Perspective Movement:
    - Arrow Right (+X): Motor A CW, Motor B CCW
    - Arrow Left (-X): Motor A CCW, Motor B CW
    - Arrow Up (+Y): Both motors CW
    - Arrow Down (-Y): Both motors CCW
"""

import time
import curses
from typing import List

from ..base_test import HardwareTest, TestStep, TestResult


class ManualGantryTest(HardwareTest):
    """
    Interactive manual gantry control test.
    Uses curses for real-time keyboard input and position display.
    """
    
    # Motor pins (BCM) - per pinout.md
    MOTOR_A_DIR_PIN = 27
    MOTOR_A_STEP_PIN = 22
    MOTOR_B_DIR_PIN = 6
    MOTOR_B_STEP_PIN = 5
    MOTOR_ENABLE_PIN = 17
    
    # Limit switch pins
    LIMIT_X_PIN = 10
    LIMIT_Y_PIN = 9
    
    # Timing constants (tuned for torque)
    DIR_SETUP_US = 5
    STEP_PULSE_US = 10
    
    # Speed configuration
    STEP_DELAY_MS = 4.0  # Delay between steps (ms)
    
    @property
    def name(self) -> str:
        return "Manual Gantry"
    
    @property
    def description(self) -> str:
        return "Interactive gantry control with arrow keys (player perspective)"
    
    def setup(self) -> bool:
        """Setup motor and limit switch pins."""
        if self.gpio is None:
            raise RuntimeError("GPIO interface required - hardware must be connected")
        
        try:
            # Setup motor output pins
            for pin in [self.MOTOR_A_DIR_PIN, self.MOTOR_A_STEP_PIN,
                        self.MOTOR_B_DIR_PIN, self.MOTOR_B_STEP_PIN,
                        self.MOTOR_ENABLE_PIN]:
                self.gpio.setup_output(pin)
            
            # Disable motors initially (ENABLE is active LOW, so HIGH = disabled)
            self.gpio.write(self.MOTOR_ENABLE_PIN, True)
            
            # Setup limit switch inputs with pull-up
            self.gpio.setup_input(self.LIMIT_X_PIN, pull_up=True)
            self.gpio.setup_input(self.LIMIT_Y_PIN, pull_up=True)
            
            # Position tracking
            self._pos_x = 0
            self._pos_y = 0
            
            return True
        except Exception as e:
            print(f"[ERROR] Setup failed: {e}")
            return False
    
    def teardown(self):
        """Disable motors on exit."""
        if self.gpio:
            try:
                self.gpio.write(self.MOTOR_A_STEP_PIN, False)
                self.gpio.write(self.MOTOR_B_STEP_PIN, False)
                self.gpio.write(self.MOTOR_ENABLE_PIN, True)  # Disable
            except:
                pass
    
    def _motor_enable(self):
        """Enable motors (active LOW)."""
        self.gpio.write(self.MOTOR_ENABLE_PIN, False)
        time.sleep(0.001)
    
    def _motor_disable(self):
        """Disable motors."""
        self.gpio.write(self.MOTOR_ENABLE_PIN, True)
    
    def _read_x_limit(self) -> bool:
        """Read X limit switch (active LOW)."""
        return not self.gpio.read(self.LIMIT_X_PIN)
    
    def _read_y_limit(self) -> bool:
        """Read Y limit switch (active LOW)."""
        return not self.gpio.read(self.LIMIT_Y_PIN)
    
    def _step_pulse(self, step_pin: int):
        """Generate a single step pulse."""
        self.gpio.write(step_pin, True)
        time.sleep(self.STEP_PULSE_US / 1_000_000)
        self.gpio.write(step_pin, False)
    
    def _step_both_motors(self, steps_a: int, steps_b: int):
        """
        Step both motors with Bresenham interpolation.
        
        CoreXY kinematics:
        - Motor A at bottom-left, Motor B at top-right
        - +X (right): A+, B- (opposite directions)
        - +Y (up): A+, B+ (same directions)
        """
        if steps_a == 0 and steps_b == 0:
            return
        
        # Set directions
        self.gpio.write(self.MOTOR_A_DIR_PIN, steps_a >= 0)
        self.gpio.write(self.MOTOR_B_DIR_PIN, steps_b >= 0)
        time.sleep(self.DIR_SETUP_US / 1_000_000)
        
        abs_a = abs(steps_a)
        abs_b = abs(steps_b)
        max_steps = max(abs_a, abs_b)
        
        # Enable motors
        self._motor_enable()
        
        try:
            err_a = 0
            err_b = 0
            
            for _ in range(max_steps):
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
                
                # Simultaneous step pulses
                if step_a:
                    self.gpio.write(self.MOTOR_A_STEP_PIN, True)
                if step_b:
                    self.gpio.write(self.MOTOR_B_STEP_PIN, True)
                
                time.sleep(self.STEP_PULSE_US / 1_000_000)
                
                self.gpio.write(self.MOTOR_A_STEP_PIN, False)
                self.gpio.write(self.MOTOR_B_STEP_PIN, False)
                
                time.sleep(self.STEP_DELAY_MS / 1000)
        finally:
            self._motor_disable()
    
    def _move_x(self, steps: int):
        """Move X axis: +X = right (A CW, B CCW)."""
        self._step_both_motors(steps, -steps)
        self._pos_x += steps
    
    def _move_y(self, steps: int):
        """Move Y axis: +Y = up (A CW, B CW)."""
        self._step_both_motors(steps, steps)
        self._pos_y += steps
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            TestStep(
                name="Manual Gantry Control",
                display_text="GANTRY",
                action=self._run_manual_control,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=300.0,
                success_message="DONE",
                failure_message="EXIT"
            ),
        ]
    
    def _run_manual_control(self) -> bool:
        """Run interactive curses-based control."""
        print("\n" + "=" * 60)
        print("  MANUAL GANTRY CONTROL")
        print("=" * 60)
        print("\nStarting interactive control in 2 seconds...")
        print("Press 'q' to exit when done.")
        time.sleep(2)
        
        try:
            curses.wrapper(self._control_loop)
            return True
        except Exception as e:
            print(f"\n[ERROR] Control loop error: {e}")
            return False
    
    def _control_loop(self, stdscr):
        """Curses-based interactive control loop."""
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(50)
        
        steps_per_tick = 25
        min_steps = 5
        max_steps = 100
        
        running = True
        
        while running:
            stdscr.clear()
            
            # Header
            stdscr.addstr(0, 0, "═" * 60)
            stdscr.addstr(1, 0, "  MANUAL GANTRY CONTROL - Player Perspective")
            stdscr.addstr(2, 0, "═" * 60)
            
            # Position display
            stdscr.addstr(4, 0, f"Position: X={self._pos_x:6d}  Y={self._pos_y:6d} steps")
            stdscr.addstr(5, 0, f"Step size: {steps_per_tick} steps/tick")
            stdscr.addstr(6, 0, f"Limits: X={self._read_x_limit()}  Y={self._read_y_limit()}")
            
            # Motor layout diagram
            stdscr.addstr(8, 0, "Motor Layout (from above):")
            stdscr.addstr(9, 0, "                     +Y (UP)")
            stdscr.addstr(10, 0, "                        ↑")
            stdscr.addstr(11, 0, "    Motor B ─────────────┼─────────────")
            stdscr.addstr(12, 0, "   (top-right)      -X ←─┼─→ +X (RIGHT)")
            stdscr.addstr(13, 0, "                         ↓")
            stdscr.addstr(14, 0, "    Motor A ─────────────┴─────────────")
            stdscr.addstr(15, 0, "  (bottom-left)         -Y (DOWN)")
            
            # Direction chart
            stdscr.addstr(17, 0, "Direction Chart:")
            stdscr.addstr(18, 0, "┌─────────┬─────────────────┬─────────────────┐")
            stdscr.addstr(19, 0, "│ Key     │ Motor A (BL)    │ Motor B (TR)    │")
            stdscr.addstr(20, 0, "├─────────┼─────────────────┼─────────────────┤")
            stdscr.addstr(21, 0, "│ → Right │ Clockwise (+)   │ Counter-CW (-)  │")
            stdscr.addstr(22, 0, "│ ← Left  │ Counter-CW (-)  │ Clockwise (+)   │")
            stdscr.addstr(23, 0, "│ ↑ Up    │ Clockwise (+)   │ Clockwise (+)   │")
            stdscr.addstr(24, 0, "│ ↓ Down  │ Counter-CW (-)  │ Counter-CW (-)  │")
            stdscr.addstr(25, 0, "└─────────┴─────────────────┴─────────────────┘")
            
            # Controls
            stdscr.addstr(27, 0, "Controls:")
            stdscr.addstr(28, 2, "↑↓←→ : Move gantry (player perspective)")
            stdscr.addstr(29, 2, "+/-  : Adjust step size")
            stdscr.addstr(30, 2, "q    : Quit")
            
            stdscr.refresh()
            
            key = stdscr.getch()
            
            if key == ord('q') or key == ord('Q'):
                running = False
                continue
            
            if key == ord('+') or key == ord('='):
                steps_per_tick = min(max_steps, steps_per_tick + 5)
            elif key == ord('-') or key == ord('_'):
                steps_per_tick = max(min_steps, steps_per_tick - 5)
            elif key == curses.KEY_RIGHT:
                # +X: Move right
                self._move_x(steps_per_tick)
            elif key == curses.KEY_LEFT:
                # -X: Move left (check limit)
                if not self._read_x_limit():
                    self._move_x(-steps_per_tick)
            elif key == curses.KEY_UP:
                # +Y: Move up
                self._move_y(steps_per_tick)
            elif key == curses.KEY_DOWN:
                # -Y: Move down (check limit)
                if not self._read_y_limit():
                    self._move_y(-steps_per_tick)
        
        stdscr.addstr(32, 0, "Exiting manual control...")
        stdscr.refresh()
        time.sleep(0.5)
