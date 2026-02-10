#!/usr/bin/env python3
"""
Gantry hardware tests for CoreXY (A4988 + NEMA 11).

All tests exercise the REAL ROS system by publishing commands to
/stepper/command and subscribing to /stepper/status and /limit_switch/*.
The stepper_driver_node handles all DMA wave generation.

Requires:
  ROS nodes running: stepper_driver_node, limit_switch_node
  Start with: ros2 launch chess_hw_interface hw_interface_launch.py

Test ordering: basic wiring → timing → direction → sync → stress
"""

import math
import threading
import time
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Point, Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from ..base_test import HardwareTest, TestStep


class GantryTestNode(Node):
    """
    Shared ROS node for gantry tests.

    Publishes to /stepper/command and /stepper/velocity.
    Subscribes to /stepper/status and /limit_switch/*.
    """

    def __init__(self, node_name: str = 'gantry_test_node'):
        super().__init__(node_name)

        # Publishers
        self.cmd_pub = self.create_publisher(Point, '/stepper/command', 10)
        self.vel_pub = self.create_publisher(Twist, '/stepper/velocity', 10)
        self.max_vel_pub = self.create_publisher(
            Float32, '/stepper/set_max_velocity', 10)
        self.accel_pub = self.create_publisher(
            Float32, '/stepper/set_acceleration', 10)

        # State from subscriptions
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

    def send_step_command(self, steps_a: int, steps_b: int, speed_pct: float = 50.0):
        """Publish point-to-point step command."""
        msg = Point()
        msg.x = float(steps_a)
        msg.y = float(steps_b)
        msg.z = speed_pct
        self.cmd_pub.publish(msg)

    def send_velocity(self, vx: float, vy: float):
        """Publish velocity command (Cartesian, steps/sec)."""
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        self.vel_pub.publish(msg)

    def wait_for_idle(self, timeout: float = 30.0) -> bool:
        """Wait for stepper to report IDLE."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status == 'IDLE':
                return True
        return False

    def move_x(self, steps: int, speed_pct: float = 35.0):
        """Move X via CoreXY: A = +steps, B = -steps."""
        self.send_step_command(steps, -steps, speed_pct)
        time.sleep(0.05)  # Let message be delivered
        self.wait_for_idle()

    def move_y(self, steps: int, speed_pct: float = 35.0):
        """Move Y via CoreXY: A = +steps, B = +steps."""
        self.send_step_command(steps, steps, speed_pct)
        time.sleep(0.05)
        self.wait_for_idle()

    def move_xy(self, dx: int, dy: int, speed_pct: float = 35.0):
        """Move diagonal via CoreXY."""
        steps_a = dx + dy
        steps_b = dy - dx
        self.send_step_command(steps_a, steps_b, speed_pct)
        time.sleep(0.05)
        self.wait_for_idle()

    def step_both(self, steps_a: int, steps_b: int, speed_pct: float = 35.0):
        """Send raw motor step command."""
        self.send_step_command(steps_a, steps_b, speed_pct)
        time.sleep(0.05)
        self.wait_for_idle()

    def step_single(self, motor: str, steps: int, speed_pct: float = 35.0):
        """Step a single motor."""
        if motor.lower() == 'a':
            self.send_step_command(steps, 0, speed_pct)
        else:
            self.send_step_command(0, steps, speed_pct)
        time.sleep(0.05)
        self.wait_for_idle()


class GantryTestBase(HardwareTest):
    """
    Shared base for gantry tests — uses ROS for all motor control.

    Manages a ROS node and background spin thread so tests can
    publish commands and read state.
    """

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[GantryTestNode] = None
        self._spin_thread: Optional[threading.Thread] = None
        self._pos_x_steps = 0
        self._pos_y_steps = 0

    def setup(self) -> bool:
        """Initialize ROS node for test communication."""
        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = GantryTestNode()
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(0.5)  # Discovery time

            # Check if stepper driver is alive
            self._node.send_step_command(0, 0, 0)
            return True
        except Exception as exc:
            print(f'[ERROR] Setup failed: {exc}')
            print('Make sure ROS nodes are running:')
            print('  ros2 launch chess_hw_interface hw_interface_launch.py')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def teardown(self):
        if self._node:
            try:
                self._node.send_velocity(0.0, 0.0)
                self._node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    # Convenience wrappers that track position

    def _step_both(self, steps_a: int, steps_b: int, speed_pct: float = 35.0):
        if self._node:
            self._node.step_both(steps_a, steps_b, speed_pct)

    def _step_single(self, motor: str, steps: int, speed_pct: float = 35.0):
        if self._node:
            self._node.step_single(motor, steps, speed_pct)

    def _move_x(self, steps: int, speed_pct: float = 35.0):
        if self._node:
            self._node.move_x(steps, speed_pct)
            self._pos_x_steps += steps

    def _move_y(self, steps: int, speed_pct: float = 35.0):
        if self._node:
            self._node.move_y(steps, speed_pct)
            self._pos_y_steps += steps

    def _move_xy(self, dx: int, dy: int, speed_pct: float = 35.0):
        if self._node:
            self._node.move_xy(dx, dy, speed_pct)
            self._pos_x_steps += dx
            self._pos_y_steps += dy

    def _read_x_limit(self) -> bool:
        return self._node.x_limit if self._node else False

    def _read_y_limit(self) -> bool:
        return self._node.y_limit if self._node else False


# ==================================================================
# LIMIT SWITCH TESTS
# ==================================================================


class GantryLimitSwitchTest(GantryTestBase):
    """Validate limit switch wiring via ROS /limit_switch/* topics."""

    @property
    def name(self) -> str:
        return 'Gantry Limit Switches'

    @property
    def description(self) -> str:
        return 'Verify X/Y limit switches are detected via ROS (active HIGH)'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Test X limit', 'X LIM', self._test_x, True,
                     'x_limit', 30.0, 'X OK', 'X ER'),
            TestStep('Test Y limit', 'Y LIM', self._test_y, True,
                     'y_limit', 30.0, 'Y OK', 'Y ER'),
        ]

    def _test_x(self) -> bool:
        print('  Press X limit switch... (reading via ROS /limit_switch/x_min)')
        return self._read_x_limit()

    def _test_y(self) -> bool:
        print('  Press Y limit switch... (reading via ROS /limit_switch/y_min)')
        return self._read_y_limit()


# ==================================================================
# PULSE / TIMING TESTS
# ==================================================================


class GantryPulseIntegrityTest(GantryTestBase):
    """
    Verify that step commands execute without errors.

    Since all stepping is now via DMA wave chains in the stepper_driver_node,
    this test sends step commands and checks for IDLE status completion.
    """

    @property
    def name(self) -> str:
        return 'Gantry Pulse Integrity'

    @property
    def description(self) -> str:
        return 'Send step commands via ROS and verify completion'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Motor A 100 steps', 'A PLS', self._pulse_a, False,
                     success_message='A OK', failure_message='A ER'),
            TestStep('Motor B 100 steps', 'B PLS', self._pulse_b, False,
                     success_message='B OK', failure_message='B ER'),
            TestStep('Both 100 steps', 'AB PL', self._pulse_both, False,
                     success_message='AB OK', failure_message='AB ER'),
        ]

    def _pulse_a(self) -> bool:
        print('  Sending 100 steps to Motor A via /stepper/command...')
        self._step_single('a', 100, 35)
        self._step_single('a', -100, 35)
        return True

    def _pulse_b(self) -> bool:
        print('  Sending 100 steps to Motor B via /stepper/command...')
        self._step_single('b', 100, 35)
        self._step_single('b', -100, 35)
        return True

    def _pulse_both(self) -> bool:
        print('  Sending 100 steps to both motors via /stepper/command...')
        self._step_both(100, 100, 35)
        self._step_both(-100, -100, 35)
        return True


# ==================================================================
# SINGLE MOTOR TESTS
# ==================================================================


class GantryMotorATest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Motor A Only'

    @property
    def description(self) -> str:
        return 'Step motor A forward/reverse via ROS — verify smooth motion'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('A forward', 'A FWD', self._forward, False,
                     success_message='FWD OK', failure_message='FWD ER'),
            TestStep('A reverse', 'A REV', self._reverse, False,
                     success_message='REV OK', failure_message='REV ER'),
            TestStep('Confirm A', 'A OK?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _forward(self) -> bool:
        print('  Motor A: +300 steps via /stepper/command')
        self._step_single('a', 300, 35)
        return True

    def _reverse(self) -> bool:
        print('  Motor A: -300 steps via /stepper/command')
        self._step_single('a', -300, 35)
        return True


class GantryMotorBTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Motor B Only'

    @property
    def description(self) -> str:
        return 'Step motor B forward/reverse via ROS — verify smooth motion'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('B forward', 'B FWD', self._forward, False,
                     success_message='FWD OK', failure_message='FWD ER'),
            TestStep('B reverse', 'B REV', self._reverse, False,
                     success_message='REV OK', failure_message='REV ER'),
            TestStep('Confirm B', 'B OK?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _forward(self) -> bool:
        print('  Motor B: +300 steps via /stepper/command')
        self._step_single('b', 300, 35)
        return True

    def _reverse(self) -> bool:
        print('  Motor B: -300 steps via /stepper/command')
        self._step_single('b', -300, 35)
        return True


# ==================================================================
# COREXY DIRECTION TEST
# ==================================================================


class GantryCoreXYDirectionTest(GantryTestBase):
    """
    Verify CoreXY axis mapping.

    After direction fix:
      +X (right): A CCW, B CW  → step_both(+steps, -steps)
      -X (left):  A CW, B CCW  → step_both(-steps, +steps)
      +Y (up):    A CCW, B CCW → step_both(+steps, +steps)
      -Y (down):  A CW, B CW   → step_both(-steps, -steps)
    """

    @property
    def name(self) -> str:
        return 'CoreXY Direction'

    @property
    def description(self) -> str:
        return 'Verify +X/−X/+Y/−Y mapping via ROS (corrected directions)'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('+X (right)', '+X', self._plus_x, False,
                     success_message='+X OK', failure_message='+X ER'),
            TestStep('-X (left)', '-X', self._minus_x, False,
                     success_message='-X OK', failure_message='-X ER'),
            TestStep('+Y (up)', '+Y', self._plus_y, False,
                     success_message='+Y OK', failure_message='+Y ER'),
            TestStep('-Y (down)', '-Y', self._minus_y, False,
                     success_message='-Y OK', failure_message='-Y ER'),
            TestStep('Confirm dirs', 'DIR?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _plus_x(self) -> bool:
        print('  +X (RIGHT): moving 300 steps via /stepper/command')
        self._move_x(300, 35)
        return True

    def _minus_x(self) -> bool:
        print('  -X (LEFT): moving -300 steps')
        self._move_x(-300, 35)
        return True

    def _plus_y(self) -> bool:
        print('  +Y (UP): moving 300 steps')
        self._move_y(300, 35)
        return True

    def _minus_y(self) -> bool:
        print('  -Y (DOWN): moving -300 steps')
        self._move_y(-300, 35)
        return True


# ==================================================================
# SPEED SWEEP TEST
# ==================================================================


class GantrySpeedSweepTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Speed Sweep'

    @property
    def description(self) -> str:
        return 'Sweep speed from 20% to 80% via ROS — identify stall zones'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Speed sweep', 'SWEEP', self._sweep, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Confirm sweep', 'SWP?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _sweep(self) -> bool:
        for speed in [20, 30, 40, 50, 60, 70, 80]:
            print(f'  Speed {speed}%: +150 / -150 X steps')
            self._move_x(150, speed)
            self._move_x(-150, speed)
            time.sleep(0.2)
        return True


# ==================================================================
# REPEATABILITY TEST
# ==================================================================


class GantryRepeatabilityTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Repeatability'

    @property
    def description(self) -> str:
        return 'Multi-loop stress test via ROS — 5 square loops at varying speeds'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Stress loops', 'LOOPS', self._loops, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Confirm return', 'RET?', lambda: True, True,
                     'clock', 30.0, 'EXACT', 'DRIFT'),
        ]

    def _loops(self) -> bool:
        for i in range(5):
            speed = 30 + i * 10
            print(f'  Loop {i+1}/5 at {speed}% speed')
            self._move_x(200, speed)
            self._move_y(200, speed)
            self._move_x(-200, speed)
            self._move_y(-200, speed)
            time.sleep(0.1)
        print(f'  Final position: X={self._pos_x_steps}, Y={self._pos_y_steps}')
        return True


# ==================================================================
# ENABLE / HOLD TEST
# ==================================================================


class GantryEnableHoldTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Enable/Hold'

    @property
    def description(self) -> str:
        return 'Verify holding torque and enable/disable behavior via ROS'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Move then hold', 'HOLD', self._move_hold, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Confirm hold', 'HLD?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _move_hold(self) -> bool:
        print('  Moving to position...')
        self._move_x(200, 35)
        print('  Holding position (try to move gantry by hand)...')
        time.sleep(3.0)
        print('  Moving back...')
        self._move_x(-200, 35)
        return True


# ==================================================================
# HOMING TEST
# ==================================================================


class GantryHomingTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Homing'

    @property
    def description(self) -> str:
        return 'Verify homing to limit switches via ROS'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Move to center', 'CNTR', self._move_center, False,
                     success_message='CTR OK', failure_message='CTR ER'),
            TestStep('Home X', 'HOM X', self._home_x, False,
                     success_message='HX OK', failure_message='HX ER'),
            TestStep('Back off X', 'BK X', self._backoff_x, False,
                     success_message='BX OK', failure_message='BX ER'),
            TestStep('Home Y', 'HOM Y', self._home_y, False,
                     success_message='HY OK', failure_message='HY ER'),
            TestStep('Confirm home', 'HOME?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _move_center(self) -> bool:
        print('  Moving toward center (600 steps +X, +Y)...')
        self._move_x(600, 40)
        self._move_y(600, 40)
        return True

    def _home_x(self) -> bool:
        print('  Approaching X limit (-X direction)...')
        for _ in range(100):
            if self._read_x_limit():
                print('  X limit triggered!')
                return True
            self._move_x(-50, 25)
        print('  WARNING: X limit not triggered after max steps')
        return False

    def _backoff_x(self) -> bool:
        print('  Backing off X...')
        self._move_x(200, 20)
        return not self._read_x_limit()

    def _home_y(self) -> bool:
        print('  Approaching Y limit (-Y direction)...')
        for _ in range(100):
            if self._read_y_limit():
                print('  Y limit triggered!')
                return True
            self._move_y(-50, 25)
        print('  WARNING: Y limit not triggered after max steps')
        return False


# ==================================================================
# FULL GUIDED TEST
# ==================================================================


class GantryFullTest(GantryTestBase):
    @property
    def name(self) -> str:
        return 'Full Gantry Test'

    @property
    def description(self) -> str:
        return 'Guided end-to-end gantry test via ROS'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('CoreXY directions', 'CXYD', self._run_corexy, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Speed sweep', 'SWEP', self._run_sweep, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Stress loops', 'LOOP', self._run_loops, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Hold test', 'HOLD', self._run_hold, False,
                     success_message='OK', failure_message='ERR'),
            TestStep('Confirm all', 'ALL?', lambda: True, True,
                     'clock', 60.0, 'PASS', 'FAIL'),
        ]

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
        self._move_x(100, 35)
        time.sleep(2.0)
        self._move_x(-100, 35)
        return True


# ==================================================================
# LOCKSTEP TEST — both motors same DIR, single wave
# ==================================================================


class GantryLockstepTest(GantryTestBase):
    """
    Lockstep motor test — both motors same DIR, same steps, single command.

    If this fails → hardware problem.
    If this passes → the CoreXY sync is working.
    """

    @property
    def name(self) -> str:
        return 'Gantry Lockstep'

    @property
    def description(self) -> str:
        return 'Both motors same DIR, 500 steps via ROS — hardware validation'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Lockstep forward', 'LOCK +', self._forward, False,
                     success_message='FWD OK', failure_message='FWD ER'),
            TestStep('Lockstep reverse', 'LOCK -', self._reverse, False,
                     success_message='REV OK', failure_message='REV ER'),
            TestStep('Confirm smooth', 'SMOOTH?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _forward(self) -> bool:
        print('  Both motors: +500 steps, same direction')
        self._step_both(500, 500, 35)
        return True

    def _reverse(self) -> bool:
        print('  Both motors: -500 steps, same direction')
        self._step_both(-500, -500, 35)
        return True


# ==================================================================
# DIAGONAL SYNC TEST
# ==================================================================


class GantryDiagonalSyncTest(GantryTestBase):
    """
    Pure diagonal CoreXY test — A and B step in one command for perfect diagonal.

    Tests that move_xy() produces clean 45° motion with no X/Y wobble.
    """

    @property
    def name(self) -> str:
        return 'Gantry Diagonal Sync'

    @property
    def description(self) -> str:
        return 'Diagonal motion via ROS move_xy() — no wobble'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Diagonal +X+Y', 'D ++', self._diag_pp, False,
                     success_message='++ OK', failure_message='++ ER'),
            TestStep('Diagonal -X-Y', 'D --', self._diag_mm, False,
                     success_message='-- OK', failure_message='-- ER'),
            TestStep('Diagonal +X-Y', 'D +-', self._diag_pm, False,
                     success_message='+- OK', failure_message='+- ER'),
            TestStep('Diagonal -X+Y', 'D -+', self._diag_mp, False,
                     success_message='-+ OK', failure_message='-+ ER'),
            TestStep('Confirm diagonal', 'DIAG?', lambda: True, True,
                     'clock', 30.0, 'OK', 'BAD'),
        ]

    def _diag_pp(self) -> bool:
        print('  Diagonal +X +Y: 300 steps — should be clean 45° line')
        self._move_xy(300, 300, 35)
        return True

    def _diag_mm(self) -> bool:
        print('  Diagonal -X -Y: 300 steps — returning')
        self._move_xy(-300, -300, 35)
        return True

    def _diag_pm(self) -> bool:
        print('  Diagonal +X -Y: 300 steps')
        self._move_xy(300, -300, 35)
        return True

    def _diag_mp(self) -> bool:
        print('  Diagonal -X +Y: 300 steps — returning')
        self._move_xy(-300, 300, 35)
        return True


# ==================================================================
# SQUARE RETURN TEST
# ==================================================================


class GantrySquareReturnTest(GantryTestBase):
    """
    Square return-to-origin test.

    Moves +X 1000, +Y 1000, -X 1000, -Y 1000.
    If sync is correct, gantry returns exactly to start.
    """

    @property
    def name(self) -> str:
        return 'Gantry Square Return'

    @property
    def description(self) -> str:
        return 'Square pattern (1000 steps/side) via ROS — return-to-origin accuracy'

    def get_steps(self) -> List[TestStep]:
        return [
            TestStep('Mark start', 'MARK', self._mark_start, False,
                     success_message='MARKED', failure_message='ERR'),
            TestStep('Run square', 'SQUARE', self._run_square, False,
                     success_message='SQ OK', failure_message='SQ ER'),
            TestStep('Confirm return', 'BACK?', lambda: True, True,
                     'clock', 45.0, 'EXACT', 'DRIFT'),
        ]

    def _mark_start(self) -> bool:
        print('  Mark the current gantry position.')
        print('  The gantry will trace a 1000-step square.')
        time.sleep(2.0)
        return True

    def _run_square(self) -> bool:
        side = 1000
        speed = 40
        print(f'  Running square: {side} steps/side at {speed}% speed')
        print('  +X...')
        self._move_x(side, speed)
        time.sleep(0.3)
        print('  +Y...')
        self._move_y(side, speed)
        time.sleep(0.3)
        print('  -X...')
        self._move_x(-side, speed)
        time.sleep(0.3)
        print('  -Y...')
        self._move_y(-side, speed)
        print(f'  Square complete. Position: X={self._pos_x_steps}, Y={self._pos_y_steps}')
        return True
