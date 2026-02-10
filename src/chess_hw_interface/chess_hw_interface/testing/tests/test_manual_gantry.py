#!/usr/bin/env python3
"""
Manual Gantry Control Test — Joystick Paradigm via ROS.

Publishes velocity commands to /stepper/velocity (Twist) so the real
stepper_driver_node handles all acceleration, wave generation, and
motor control.  This test exercises the ACTUAL ROS system.

Behavior:
  - Arrow keys / WASD set desired velocity direction
  - Numpad 7/9/1/3 for diagonals
  - Release key → velocity goes to zero → smooth deceleration
  - [ / ] adjust max speed
  - , / . adjust acceleration
  - q to quit

All movement goes through:
  test → /stepper/velocity → stepper_driver_node → pigpio DMA → motors

Requires:
  ROS nodes running: stepper_driver_node, limit_switch_node
"""

import curses
import math
import sys
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from ..base_test import HardwareTest, TestStep


class ManualGantryTestNode(Node):
    """Lightweight ROS node for publishing velocity commands."""

    def __init__(self):
        super().__init__('manual_gantry_test')

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/stepper/velocity', 10)
        self.max_vel_pub = self.create_publisher(
            Float32, '/stepper/set_max_velocity', 10)
        self.accel_pub = self.create_publisher(
            Float32, '/stepper/set_acceleration', 10)

        # Subscribers
        self.status = 'UNKNOWN'
        self.x_limit = False
        self.y_limit = False

        self.create_subscription(
            String, '/stepper/status', self._status_cb, 10)
        self.create_subscription(
            Bool, '/limit_switch/x_min', self._x_limit_cb, 10)
        self.create_subscription(
            Bool, '/limit_switch/y_min', self._y_limit_cb, 10)

    def _status_cb(self, msg):
        self.status = msg.data

    def _x_limit_cb(self, msg):
        self.x_limit = msg.data

    def _y_limit_cb(self, msg):
        self.y_limit = msg.data

    def publish_velocity(self, vx: float, vy: float):
        """Publish Cartesian velocity (steps/sec)."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        self.vel_pub.publish(msg)

    def publish_max_velocity(self, val: float):
        self.max_vel_pub.publish(Float32(data=val))

    def publish_acceleration(self, val: float):
        self.accel_pub.publish(Float32(data=val))

    def stop(self):
        """Publish zero velocity."""
        self.publish_velocity(0.0, 0.0)


class ManualGantryTest(HardwareTest):
    """
    Interactive joystick gantry control via ROS velocity commands.

    All motion goes through the stepper_driver_node — this test exercises
    the real ROS system.
    """

    @property
    def name(self) -> str:
        return 'Manual Gantry (Joystick)'

    @property
    def description(self) -> str:
        return 'Joystick control via ROS /stepper/velocity — smooth accel/decel, instant stop'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._ros_node: Optional[ManualGantryTestNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._max_speed = 400.0       # steps/sec
        self._acceleration = 2000.0   # steps/sec²

    def setup(self) -> bool:
        """Initialize ROS node for velocity publishing."""
        try:
            if not rclpy.ok():
                rclpy.init()
            self._ros_node = ManualGantryTestNode()

            # Start spinning in background thread
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()

            # Give ROS a moment to discover topics
            time.sleep(0.5)

            # Push initial parameters to stepper driver
            self._ros_node.publish_max_velocity(self._max_speed)
            self._ros_node.publish_acceleration(self._acceleration)

            return True
        except Exception as exc:
            print(f'[ERROR] Setup failed: {exc}')
            print('Make sure ROS nodes are running:')
            print('  ros2 launch chess_hw_interface hw_interface_launch.py')
            return False

    def _spin(self):
        """Background ROS spin."""
        try:
            rclpy.spin(self._ros_node)
        except Exception:
            pass

    def teardown(self):
        """Stop velocity and clean up ROS."""
        if self._ros_node:
            try:
                self._ros_node.stop()
            except Exception:
                pass
            try:
                self._ros_node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep(
                name='Joystick Gantry Control',
                display_text='JYSTK',
                action=self._run_joystick,
                wait_for_input=False,
                success_message='DONE',
                failure_message='FAIL',
            )
        ]

    def _run_joystick(self) -> bool:
        print('\nStarting joystick gantry control...')
        print('Controls: arrows/WASD move, numpad diagonals, [/] speed, ,/. accel, q quit')
        print('Requires ROS nodes: stepper_driver_node, limit_switch_node')
        time.sleep(1.5)

        try:
            curses.wrapper(self._control_loop)
            return True
        except Exception as exc:
            print(f'\n[ERROR] Control loop error: {exc}')
            return False

    def _control_loop(self, stdscr):
        curses.curs_set(0)
        stdscr.nodelay(True)
        stdscr.timeout(20)  # 50Hz update rate — matches stepper driver tick

        running = True

        while running:
            # --- Read key ---
            key = stdscr.getch()

            # --- Determine desired velocity direction ---
            vx = 0.0
            vy = 0.0

            if key == curses.KEY_RIGHT or key in (ord('d'), ord('D')):
                vx = self._max_speed
            elif key == curses.KEY_LEFT or key in (ord('a'), ord('A')):
                vx = -self._max_speed
            elif key == curses.KEY_UP or key in (ord('w'), ord('W')):
                vy = self._max_speed
            elif key == curses.KEY_DOWN or key in (ord('s'), ord('S')):
                vy = -self._max_speed
            # Diagonals
            elif key == ord('9'):
                vx = self._max_speed
                vy = self._max_speed
            elif key == ord('7'):
                vx = -self._max_speed
                vy = self._max_speed
            elif key == ord('3'):
                vx = self._max_speed
                vy = -self._max_speed
            elif key == ord('1'):
                vx = -self._max_speed
                vy = -self._max_speed
            # Speed/accel adjustment
            elif key == ord(']'):
                self._max_speed = min(1500.0, self._max_speed + 50.0)
                self._ros_node.publish_max_velocity(self._max_speed)
            elif key == ord('['):
                self._max_speed = max(50.0, self._max_speed - 50.0)
                self._ros_node.publish_max_velocity(self._max_speed)
            elif key == ord('.'):
                self._acceleration = min(8000.0, self._acceleration + 200.0)
                self._ros_node.publish_acceleration(self._acceleration)
            elif key == ord(','):
                self._acceleration = max(200.0, self._acceleration - 200.0)
                self._ros_node.publish_acceleration(self._acceleration)
            elif key in (ord('q'), ord('Q')):
                self._ros_node.stop()
                running = False
                continue

            # Normalize diagonals to same speed magnitude
            if vx != 0.0 and vy != 0.0:
                norm = math.sqrt(vx * vx + vy * vy)
                scale = self._max_speed / norm
                vx *= scale
                vy *= scale

            # Publish velocity (zero if no key pressed → smooth stop)
            self._ros_node.publish_velocity(vx, vy)

            # --- Draw UI ---
            stdscr.clear()
            stdscr.addstr(0, 0, '═' * 60)
            stdscr.addstr(1, 0, '  JOYSTICK GANTRY CONTROL (via ROS /stepper/velocity)')
            stdscr.addstr(2, 0, '═' * 60)

            stdscr.addstr(4, 0, f'Max Speed:    {self._max_speed:7.0f} steps/sec   [ / ] to adjust')
            stdscr.addstr(5, 0, f'Acceleration: {self._acceleration:7.0f} steps/sec²  , / . to adjust')
            stdscr.addstr(6, 0, f'Stepper:      {self._ros_node.status}')

            limit_str = ''
            if self._ros_node.x_limit:
                limit_str += '  X-LIMIT!'
            if self._ros_node.y_limit:
                limit_str += '  Y-LIMIT!'
            stdscr.addstr(7, 0, f'Limits:{limit_str if limit_str else "  clear"}')

            dir_str = 'STOP'
            if vx > 0 and vy > 0:
                dir_str = '↗ NE'
            elif vx < 0 and vy > 0:
                dir_str = '↖ NW'
            elif vx > 0 and vy < 0:
                dir_str = '↘ SE'
            elif vx < 0 and vy < 0:
                dir_str = '↙ SW'
            elif vx > 0:
                dir_str = '→ E'
            elif vx < 0:
                dir_str = '← W'
            elif vy > 0:
                dir_str = '↑ N'
            elif vy < 0:
                dir_str = '↓ S'

            stdscr.addstr(9, 0, f'Direction: {dir_str}')

            stdscr.addstr(11, 0, 'Controls:')
            stdscr.addstr(12, 2, '↑ ↓ ← → / WASD : cardinal movement')
            stdscr.addstr(13, 2, '7 9 1 3          : diagonal movement')
            stdscr.addstr(14, 2, '[ / ]            : decrease/increase speed')
            stdscr.addstr(15, 2, ', / .            : decrease/increase acceleration')
            stdscr.addstr(16, 2, 'q                : quit')

            stdscr.addstr(18, 0, 'CoreXY mapping (from player perspective):')
            stdscr.addstr(19, 2, 'Right(→): Motor A CCW, Motor B CW')
            stdscr.addstr(20, 2, 'Left(←):  Motor A CW,  Motor B CCW')
            stdscr.addstr(21, 2, 'Up(↑):    Motor A CCW, Motor B CCW')
            stdscr.addstr(22, 2, 'Down(↓):  Motor A CW,  Motor B CW')

            stdscr.addstr(24, 0, 'Release key → smooth stop. All motion via ROS.')
            stdscr.refresh()

        stdscr.addstr(26, 0, 'Exiting joystick control...')
        stdscr.refresh()
        time.sleep(0.5)
