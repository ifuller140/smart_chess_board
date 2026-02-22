#!/usr/bin/env python3
"""
Magnet Servo Test Suite.

Tests the Z-axis servo that raises/lowers the permanent magnet
through the underside of the chess board.

NOTE: There is NO electromagnet in this project.
The servo carries a permanent magnet on its arm. Engaging = lowering
magnet to board. Releasing = raising magnet clear of board.

Tests call the /servo/engage and /servo/release ROS services.

Requires:
  ROS nodes running: servo_node
  Start with: ros2 launch chess_hw_interface hw_interface_launch.py
"""

import threading
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ..base_test import HardwareTest, TestStep


class MagnetServoNode(Node):
    """ROS node for calling servo engage/release services."""

    def __init__(self):
        super().__init__('magnet_servo_test_node')
        self.servo_state = 'UNKNOWN'

        self.engage_client = self.create_client(Trigger, '/servo/engage')
        self.release_client = self.create_client(Trigger, '/servo/release')
        self.create_subscription(
            String, '/servo/state', self._state_cb, 10)

    def _state_cb(self, msg):
        self.servo_state = msg.data

    def call_engage(self, timeout_s: float = 5.0) -> bool:
        """Call /servo/engage and return True on success."""
        if not self.engage_client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error('  /servo/engage service not available')
            return False
        req = Trigger.Request()
        future = self.engage_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.result() is not None:
            return future.result().success
        return False

    def call_release(self, timeout_s: float = 5.0) -> bool:
        """Call /servo/release and return True on success."""
        if not self.release_client.wait_for_service(timeout_sec=timeout_s):
            self.get_logger().error('  /servo/release service not available')
            return False
        req = Trigger.Request()
        future = self.release_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_s)
        if future.result() is not None:
            return future.result().success
        return False


class MagnetTest(HardwareTest):
    """
    Test the Z-axis servo that carries the permanent magnet.

    Calls /servo/engage (magnet down) and /servo/release (magnet up)
    via ROS services. All servo control is handled by servo_node.
    """

    @property
    def name(self) -> str:
        return 'Magnet Servo'

    @property
    def description(self) -> str:
        return 'Test servo engage (magnet down) and release (magnet up) via ROS'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[MagnetServoNode] = None
        self._spin_thread: Optional[threading.Thread] = None

    def setup(self) -> bool:
        """Initialize ROS node for servo control."""
        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = MagnetServoNode()
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(0.5)  # Service discovery time
            return True
        except Exception as e:
            print(f'[ERROR] Setup failed: {e}')
            print('Make sure servo_node is running:')
            print('  ros2 launch chess_hw_interface hw_interface_launch.py')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def teardown(self):
        """Ensure magnet is released (servo raised) on cleanup."""
        if self._node:
            print('  Releasing magnet (safety cleanup)...')
            self._node.call_release()
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass

    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Safety reminder
            TestStep(
                name='Safety Check',
                display_text='SAFE CHK',
                action=self._safety_warning,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='READY',
                failure_message='ABORT',
            ),

            # Engage test — servo lowers magnet
            TestStep(
                name='Magnet Engage (servo down)',
                display_text='MAG DOWN',
                action=self._test_engage,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='DOWN OK?',
                failure_message='DOWN FAIL',
            ),

            # Confirm engage
            TestStep(
                name='Confirm Engage',
                display_text='DN OK?',
                action=lambda: True,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='DN CONF',
                failure_message='DN BAD',
            ),

            # Release test — servo raises magnet
            TestStep(
                name='Magnet Release (servo up)',
                display_text='MAG UP',
                action=self._test_release,
                wait_for_input=False,
                success_message='UP OK',
                failure_message='UP FAIL',
            ),

            # Hold strength test (servo down for 5s)
            TestStep(
                name='Hold Strength Test',
                display_text='HOLD TST',
                action=self._test_hold_strength,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=60.0,
                success_message='HOLD OK',
                failure_message='HOLD BAD',
            ),
        ]

    def _safety_warning(self) -> bool:
        """Display safety warning."""
        print('\n  PERMANENT MAGNET SERVO TEST')
        print('  ----------------------------')
        print('  - Servo will lower magnet toward board surface')
        print('  - Keep steel tools away from magnet area')
        print('  - Have a chess piece ready to test attraction')
        print('  - This uses /servo/engage and /servo/release (ROS services)')
        print('\n  Press clock button when ready...\n')
        return True

    def _test_engage(self) -> bool:
        """Test magnet engagement (servo lowers)."""
        print('  Calling /servo/engage (lowering magnet)...')
        ok = self._node.call_engage()
        if ok:
            print('  Servo engaged — magnet lowered to board.')
            print('  Place a chess piece on the board above the magnet.')
        else:
            print('  [ERROR] /servo/engage failed.')
        return ok

    def _test_release(self) -> bool:
        """Test magnet release (servo raises)."""
        print('  Calling /servo/release (raising magnet)...')
        ok = self._node.call_release()
        if ok:
            print('  Servo released — magnet raised clear of board.')
        else:
            print('  [ERROR] /servo/release failed.')
        return ok

    def _test_hold_strength(self) -> bool:
        """Test that the servo can hold magnet for 5 seconds."""
        print('\n  HOLD STRENGTH TEST')
        print('  ------------------')
        print('  1. Move gantry above a square with a chess piece')
        print('  2. Servo will engage (lower magnet) for 5 seconds')
        print('  3. Lift the gantry arm slightly — piece should follow')
        print('\n  Press clock button to start test...\n')

        print('  Calling /servo/engage...')
        if not self._node.call_engage():
            return False

        for i in range(5, 0, -1):
            self.show_display(f'HOLD  {i}')
            time.sleep(1.0)

        print('  Calling /servo/release...')
        self._node.call_release()
        return True
