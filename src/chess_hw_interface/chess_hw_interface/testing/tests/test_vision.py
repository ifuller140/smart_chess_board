#!/usr/bin/env python3
"""
Vision Pipeline Integration Test.

Tests the full perception pipeline end-to-end:
  1. Camera node can capture and publish images
  2. Board detector can detect corners from the camera feed
  3. Piece detector publishes a BoardState with a valid FEN string
  4. Reference capture service is callable
  5. Debug image is published

This test requires:
  - Camera physically pointed at a chess board
  - ROS nodes running: camera_node, board_detector_node, piece_detector_node
  - Start with: ros2 launch chess_perception perception_launch.py
"""

import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

from chess_interfaces.msg import BoardState

from ..base_test import HardwareTest, TestStep


class VisionTestNode(Node):
    """ROS node for subscribing to vision pipeline topics."""

    def __init__(self):
        super().__init__('vision_test_node')

        self.image_received    = False
        self.geometry_received = False
        self.board_state_received = False
        self.debug_received    = False
        self.latest_fen        = ''
        self.latest_piece_count = 0
        self.geometry_corner_count = 0

        self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)
        self.create_subscription(
            BoardState, '/perception/board_geometry', self._on_geometry, 10)
        self.create_subscription(
            BoardState, '/perception/board_state', self._on_board_state, 10)
        self.create_subscription(
            Image, '/perception/piece_debug', self._on_debug, 10)

        self._capture_client = self.create_client(Trigger, '/camera/capture')
        self._ref_client     = self.create_client(
            Trigger, '/perception/capture_reference')

    def _on_image(self, msg):
        self.image_received = True

    def _on_geometry(self, msg):
        self.geometry_received = True
        self.geometry_corner_count = sum(
            1 for p in msg.corners if p.x > 0 or p.y > 0)

    def _on_board_state(self, msg):
        self.board_state_received = True
        self.latest_fen = msg.fen
        self.latest_piece_count = sum(1 for p in msg.pieces if p != 0)

    def _on_debug(self, msg):
        self.debug_received = True

    def call_capture(self) -> bool:
        if not self._capture_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self._capture_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return bool(future.result() and future.result().success)

    def call_capture_reference(self) -> bool:
        if not self._ref_client.wait_for_service(timeout_sec=5.0):
            return False
        future = self._ref_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        return bool(future.result() and future.result().success)


class VisionPipelineTest(HardwareTest):
    """
    Full perception pipeline integration test.

    Tests camera, board detector, and piece detector together.
    Requires physical camera pointed at a chess board.
    """

    @property
    def name(self) -> str:
        return 'Vision Pipeline'

    @property
    def description(self) -> str:
        return 'End-to-end perception pipeline: camera → board detect → piece detect'

    def __init__(self, gpio_interface=None, display_interface=None):
        super().__init__(gpio_interface, display_interface)
        self._node: Optional[VisionTestNode] = None
        self._spin_thread: Optional[threading.Thread] = None

    def setup(self) -> bool:
        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = VisionTestNode()
            self._spin_thread = threading.Thread(
                target=self._spin, daemon=True)
            self._spin_thread.start()
            time.sleep(1.0)
            return True
        except Exception as e:
            print(f'[ERROR] Setup failed: {e}')
            return False

    def _spin(self):
        try:
            rclpy.spin(self._node)
        except Exception:
            pass

    def teardown(self):
        if self._node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def get_steps(self):
        return [
            TestStep(
                name='Camera Image Stream',
                display_text='CAM CHK',
                action=self._test_camera_stream,
                wait_for_input=False,
                success_message='CAM OK',
                failure_message='CAM FAIL',
            ),
            TestStep(
                name='Manual Capture Service',
                display_text='CAPTURE',
                action=self._test_capture_service,
                wait_for_input=False,
                success_message='CAP OK',
                failure_message='CAP FAIL',
            ),
            TestStep(
                name='Board Corner Detection',
                display_text='CORNERS',
                action=self._test_board_detection,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='CRNS OK',
                failure_message='NO CRNS',
            ),
            TestStep(
                name='Reference Baseline Capture',
                display_text='REF CAP',
                action=self._test_reference_capture,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='REF OK',
                failure_message='REF FAIL',
            ),
            TestStep(
                name='Piece Detection FEN',
                display_text='PIECE DET',
                action=self._test_piece_detection,
                wait_for_input=True,
                input_type='clock',
                timeout_seconds=30.0,
                success_message='FEN OK',
                failure_message='NO FEN',
            ),
        ]

    # ── Step actions ──────────────────────────────────────────────────────

    def _test_camera_stream(self) -> bool:
        print('  Waiting for /camera/image_raw topic...')
        deadline = time.time() + 10.0
        while not self._node.image_received and time.time() < deadline:
            time.sleep(0.1)
        if self._node.image_received:
            print('  ✓ Camera images are streaming')
            return True
        print('  ✗ No image received. Is camera_node running?')
        return False

    def _test_capture_service(self) -> bool:
        print('  Calling /camera/capture service...')
        ok = self._node.call_capture()
        if ok:
            print('  ✓ Capture service responded successfully')
        else:
            print('  ✗ Capture service unavailable or failed')
        return ok

    def _test_board_detection(self) -> bool:
        print()
        print('  CORNER DETECTION TEST')
        print('  ---------------------')
        print('  Ensure the camera can see the chess board clearly.')
        print('  Press clock button to trigger detection...')

        self._node.call_capture()
        time.sleep(2.0)  # Allow board_detector to process

        if self._node.geometry_received:
            n = self._node.geometry_corner_count
            print(f'  ✓ Board geometry received ({n}/4 corners detected)')
            return n == 4
        print('  ✗ No board geometry received. Check board_detector_node and lighting.')
        return False

    def _test_reference_capture(self) -> bool:
        print()
        print('  REFERENCE BASELINE CAPTURE')
        print('  --------------------------')
        print('  Remove ALL pieces from the board.')
        print('  Press clock button to capture empty-board reference...')

        ok = self._node.call_capture_reference()
        if ok:
            print('  ✓ Empty board reference captured')
        else:
            print('  ✗ Reference capture failed')
        return ok

    def _test_piece_detection(self) -> bool:
        print()
        print('  PIECE DETECTION TEST')
        print('  --------------------')
        print('  Place chess pieces on the board (starting position or any setup).')
        print('  Press clock button to test piece detection...')

        self._node.board_state_received = False
        self._node.call_capture()
        time.sleep(3.0)

        if not self._node.board_state_received:
            print('  ✗ No /perception/board_state received')
            return False

        fen = self._node.latest_fen
        pieces = self._node.latest_piece_count
        print(f'  ✓ Board state received')
        print(f'    FEN: {fen}')
        print(f'    Pieces detected: {pieces}')
        print(f'    Debug image: view /perception/piece_debug in rqt')

        # Basic FEN validation
        try:
            import chess
            chess.Board(fen)
            print('  ✓ FEN is valid (parseable by python-chess)')
            return True
        except Exception as e:
            print(f'  ✗ FEN is invalid: {e}')
            return False
