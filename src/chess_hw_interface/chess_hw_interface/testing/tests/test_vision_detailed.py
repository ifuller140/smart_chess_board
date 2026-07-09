#!/usr/bin/env python3
"""
Granular Vision Pipeline Tests — individual validation of every CV stage.

Three test subcategories (all under vision/):
  vision/corners   — detect and annotate board corners (colored dots + lines)
  vision/board     — perspective warp + labeled 8×8 grid overlay
  vision/squares   — warp + all 64 square labels + interactive highlight mode

(vision/pieces and vision/fen were removed — they tested a color-threshold
per-square piece classification + full-board FEN reconstruction pipeline
that piece_detector_node no longer implements. The current architecture is
frame-diff based: it flags which squares changed between two captures
rather than classifying piece color/type from vision at all — see
piece_detector_node.py's module docstring. There is no vision-side FEN to
test; the authoritative FEN comes from game_manager_node's own board model.)

All tests:
  - Require camera_node, board_detector_node, piece_detector_node to be running
  - Publish annotated images to /chess_vision/<name>/debug
  - Save latest capture to /tmp/chess_vision/ (3-image rotation, overwrites oldest)
  - Print remote-viewing instructions at startup

Remote viewing over SSH:
  Option A — rqt (requires X11 forwarding):
    ssh -X pi@<IP>
    ros2 run rqt_image_view rqt_image_view
    Select /chess_vision/<name>/debug

  Option B — copy saved file:
    scp pi@<IP>:/tmp/chess_vision/latest_<test>.jpg .

  Option C — stream raw camera:
    ssh -X pi@<IP>
    ros2 run rqt_image_view rqt_image_view /camera/image_raw
"""

import os
import time
import threading
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from chess_interfaces.msg import BoardState

from ..base_test import HardwareTest, TestStep


# ─────────────────────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────────────────────

VISION_SAVE_DIR      = '/tmp/chess_vision'
MAX_SAVED_PER_TEST   = 3
WARP_SIZE            = 400   # px
SQ_PX                = WARP_SIZE // 8

# Corner colors: TL=Cyan, TR=Yellow, BR=Orange, BL=Magenta
CORNER_COLORS = [
    (255, 255,  0),   # 0 TL  Cyan
    ( 0, 255, 255),   # 1 TR  Yellow
    ( 0, 165, 255),   # 2 BR  Orange
    (255,  0, 255),   # 3 BL  Magenta
]
CORNER_LABELS = ['TL', 'TR', 'BR', 'BL']

GRID_COLOR         = (200, 200, 200)   # Light gray
HIGHLIGHT_COLOR    = (  0, 215, 255)   # Gold
EMPTY_SQ_DARK      = ( 90,  60,  30)   # Dark square (BGR)
EMPTY_SQ_LIGHT     = (230, 200, 160)   # Light square (BGR)


# ─────────────────────────────────────────────────────────────────────────────
# Image Saver — rotating 3-file buffer per test
# ─────────────────────────────────────────────────────────────────────────────

class ImageSaver:
    """
    Maintains a rotating buffer of MAX_SAVED_PER_TEST JPEG files per test.
    Files are named <dir>/<test_name>_0.jpg, _1.jpg, _2.jpg.
    Always overwrites the oldest file. Also writes <test_name>_latest.jpg
    as a stable alias (for reliable scp).
    """

    def __init__(self, test_name: str, save_dir: str = VISION_SAVE_DIR):
        self._test_name = test_name
        self._dir       = save_dir
        self._idx       = 0
        os.makedirs(save_dir, exist_ok=True)

    def save(self, image: np.ndarray) -> str:
        """Save image, return saved path."""
        fname   = f'{self._test_name}_{self._idx % MAX_SAVED_PER_TEST}.jpg'
        latest  = f'latest_{self._test_name}.jpg'
        path    = os.path.join(self._dir, fname)
        lpath   = os.path.join(self._dir, latest)

        cv2.imwrite(path, image)
        cv2.imwrite(lpath, image)  # stable alias
        self._idx += 1
        return lpath


# ─────────────────────────────────────────────────────────────────────────────
# Shared ROS Node
# ─────────────────────────────────────────────────────────────────────────────

class VisionDetailNode(Node):
    """
    Single shared ROS node for all detailed vision tests.
    Subscribes to camera + detection topics and publishes annotated debug images.
    """

    def __init__(self):
        super().__init__('vision_detail_test_node')
        self._lock   = threading.Lock()

        # Latest data
        self.latest_raw:     Optional[np.ndarray] = None
        self.latest_corners: Optional[np.ndarray] = None  # shape (4,2)
        self.raw_received    = False
        self.corners_received = False

        # Subscribers
        self.create_subscription(
            Image, '/camera/image_raw', self._on_raw, 10)
        self.create_subscription(
            BoardState, '/perception/board_geometry', self._on_geometry, 10)

        # Debug image publishers (one per test)
        self._pubs = {
            'corners': self.create_publisher(Image, '/chess_vision/corners/debug', 10),
            'board':   self.create_publisher(Image, '/chess_vision/board/debug',   10),
            'squares': self.create_publisher(Image, '/chess_vision/squares/debug', 10),
        }

        # Service clients
        self._cap_client = self.create_client(Trigger, '/camera/capture')

    def _on_raw(self, msg: Image):
        with self._lock:
            try:
                self.latest_raw = np.array(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
                self.raw_received = True
            except Exception as e:
                self.get_logger().error(f'Image decode error: {e}')

    def _on_geometry(self, msg: BoardState):
        if len(msg.corners) == 4:
            pts = np.array([[p.x, p.y] for p in msg.corners], dtype='float32')
            with self._lock:
                self.latest_corners = pts
                self.corners_received = True

    def publish_debug(self, channel: str, image: np.ndarray):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = image.shape[0]
        msg.width = image.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = image.shape[1] * 3
        msg.data = image.tobytes()
        self._pubs[channel].publish(msg)

    def capture(self) -> bool:
        if not self._cap_client.wait_for_service(timeout_sec=4.0):
            return False
        future = self._cap_client.call_async(Trigger.Request())
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.1)
        if future.done() and future.result():
            return bool(future.result().success)
        return False

    def get_snapshot(self):
        """Return a thread-safe snapshot of all latest data."""
        with self._lock:
            return (
                self.latest_raw.copy()     if self.latest_raw is not None else None,
                self.latest_corners.copy() if self.latest_corners is not None else None,
            )


# ─────────────────────────────────────────────────────────────────────────────
# Drawing helpers
# ─────────────────────────────────────────────────────────────────────────────

def _label(img, text, pos, color=(0, 255, 0), scale=0.5, thickness=1, bg=True):
    """Draw a text label with optional black background for visibility."""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (w, h), base = cv2.getTextSize(text, font, scale, thickness)
    x, y = pos
    if bg:
        cv2.rectangle(img, (x - 2, y - h - 3), (x + w + 2, y + base + 1),
                      (0, 0, 0), cv2.FILLED)
    cv2.putText(img, text, (x, y), font, scale, color, thickness, cv2.LINE_AA)


def _warp_perspective(image: np.ndarray, corners: np.ndarray) -> Optional[np.ndarray]:
    """Apply perspective transform from board corners to a square image."""
    dst = np.array([
        [0,          0],
        [WARP_SIZE - 1, 0],
        [WARP_SIZE - 1, WARP_SIZE - 1],
        [0,          WARP_SIZE - 1],
    ], dtype='float32')
    try:
        M = cv2.getPerspectiveTransform(corners, dst)
        return cv2.warpPerspective(image, M, (WARP_SIZE, WARP_SIZE))
    except Exception:
        return None


def _square_name(img_row: int, img_col: int) -> str:
    """Convert warped-image row/col to chess square name (row 0 = rank 8)."""
    return chr(ord('a') + img_col) + str(8 - img_row)


def _draw_grid(img: np.ndarray, color=GRID_COLOR, thickness=1):
    """Draw 8×8 chess grid lines on a warped board image."""
    for i in range(9):
        x = i * SQ_PX
        cv2.line(img, (x, 0), (x, WARP_SIZE), color, thickness)
        cv2.line(img, (0, x), (WARP_SIZE, x), color, thickness)


def _print_remote_viewing_hint(test_name: str):
    """Print local viewing instructions."""
    print()
    print('  ─── Local Viewing ────────────────────────────────────────')
    print(f'  Displaying live on connected monitor: [{test_name}]')
    print(f'  Press Ctrl+C or answer the prompt to advance.')
    print('  ──────────────────────────────────────────────────────────')
    print()


# ─────────────────────────────────────────────────────────────────────────────
# Base class shared by all 5 tests
# ─────────────────────────────────────────────────────────────────────────────

class VisionDetailBase(HardwareTest):
    """Shared setup/teardown for vision detail tests."""

    def __init__(self, gpio_interface=None, display_interface=None, cancel_check=None):
        super().__init__(gpio_interface, display_interface, cancel_check)
        self._node:   Optional[VisionDetailNode] = None
        self._thread: Optional[threading.Thread] = None
        self._saver:  Optional[ImageSaver]       = None

    def setup(self) -> bool:
        try:
            if not rclpy.ok():
                rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self._node = VisionDetailNode()
            self._saver = ImageSaver(self._test_key)
            self._executor = rclpy.executors.SingleThreadedExecutor()
            self._executor.add_node(self._node)

            self._thread = threading.Thread(
                target=self._executor.spin, daemon=True)
            self._thread.start()
            time.sleep(1.5)   # Allow subscriptions to settle
            return True
        except Exception as e:
            print(f'[ERROR] Setup failed: {e}')
            return False

    def teardown(self):
        try:
            cv2.destroyAllWindows()
            cv2.waitKey(1)
        except Exception:
            pass
        try:
            if hasattr(self, '_executor') and self._executor is not None:
                self._executor.shutdown()
        except Exception:
            pass
        try:
            if self._node:
                self._node.destroy_node()
        except Exception:
            pass

    @property
    def _test_key(self) -> str:
        """Short key used for save path and debug topic."""
        raise NotImplementedError

    def _wait_for_raw(self, timeout=8.0) -> bool:
        deadline = time.time() + timeout
        while not self._node.raw_received and time.time() < deadline:
            time.sleep(0.1)
        return self._node.raw_received

    def _wait_for_corners(self, timeout=6.0) -> bool:
        deadline = time.time() + timeout
        while not self._node.corners_received and time.time() < deadline:
            time.sleep(0.1)
        return self._node.corners_received

    def _publish_and_save(self, channel: str, image: np.ndarray) -> str:
        # Display directly on full-screen monitor using OpenCV highgui
        cv2.imshow(f'Vision Debug: {channel}', image)
        cv2.waitKey(200)  # Render frame and process UI events
        return self._saver.save(image)


# ═════════════════════════════════════════════════════════════════════════════
# TEST 1: Corner Detection
# ═════════════════════════════════════════════════════════════════════════════

class CameraCornerDetectionTest(VisionDetailBase):
    """
    Visualize board corner detection.

    Overlay:
      • Large colored circle at each corner (TL=Cyan, TR=Yellow, BR=Orange, BL=Magenta)
      • Corner index label + pixel coordinates
      • Bright green quadrilateral connecting all 4 corners
      • Corner count status in top-left
    """

    @property
    def name(self) -> str:        return 'Corner Detection'
    @property
    def description(self) -> str: return 'Visualize detected board corners with colored markers'
    @property
    def _test_key(self) -> str:   return 'corners'

    def get_steps(self):
        return [
            TestStep(name='Camera alive',       display_text='CAM',     action=self._check_camera,  wait_for_input=False, success_message='CAM OK',  failure_message='NO CAM'),
            TestStep(name='Capture + Annotate', display_text='CORNERS', action=self._run,            wait_for_input=True,  input_type='clock', timeout_seconds=30.0, success_message='CRNS OK', failure_message='CRNS FAIL'),
        ]

    def _check_camera(self) -> bool:
        if not self._wait_for_raw(8.0):
            print('  ✗ No camera feed. Is camera_node running?')
            return False
        print('  ✓ Camera is streaming')
        return True

    def _run(self) -> bool:
        _print_remote_viewing_hint('corners')
        print('  Point camera at chess board. Press clock to capture and annotate...')
        self._node.corners_received = False
        self._node.capture()
        if not self._wait_for_corners(6.0):
            print('  ✗ No board corners detected. Ensure board_detector_node is running and board is visible.')
            return False

        raw, corners = self._node.get_snapshot()
        if raw is None or corners is None:
            print('  ✗ Missing image data')
            return False

        annotated = self._annotate_corners(raw.copy(), corners)
        path = self._publish_and_save('corners', annotated)

        print(f'  ✓ Corners detected and annotated')
        print(f'    TL={corners[0]}  TR={corners[1]}')
        print(f'    BR={corners[2]}  BL={corners[3]}')
        print(f'  → Image saved: {path}')
        return True

    def _annotate_corners(self, img: np.ndarray, corners: np.ndarray) -> np.ndarray:
        n = len(corners)

        # Draw quadrilateral connecting corners
        hull = corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [hull], isClosed=True,
                      color=(0, 255, 0), thickness=3)

        # Draw each corner
        for i, (cx, cy) in enumerate(corners):
            col = CORNER_COLORS[i]
            lbl = CORNER_LABELS[i]
            cv2.circle(img, (int(cx), int(cy)), 18, col, -1)
            cv2.circle(img, (int(cx), int(cy)), 18, (255, 255, 255), 2)
            _label(img, f'{lbl} ({int(cx)},{int(cy)})',
                   (int(cx) + 22, int(cy) + 6), col, scale=0.55, thickness=2)

        # Status badge top-left
        _label(img, f'{n}/4 CORNERS FOUND  (Green=OK)',
               (10, 30), (0, 255, 0) if n == 4 else (0, 0, 255),
               scale=0.7, thickness=2)

        # Legend bottom-left
        for i, (col, lbl) in enumerate(zip(CORNER_COLORS, CORNER_LABELS)):
            _label(img, f'● {lbl}',
                   (10, img.shape[0] - 20 - i * 22),
                   col, scale=0.5)

        return img


# ═════════════════════════════════════════════════════════════════════════════
# TEST 2: Board Detection (Warp + Grid)
# ═════════════════════════════════════════════════════════════════════════════

class BoardDetectionTest(VisionDetailBase):
    """
    Show perspective-warped board with labeled 8×8 grid.

    Produces a side-by-side image:
      Left  — raw camera image with corner markers
      Right — warped bird's-eye view with full grid and file/rank labels
    """

    @property
    def name(self) -> str:        return 'Board Detection'
    @property
    def description(self) -> str: return 'Board warp + labeled 8x8 grid overlay'
    @property
    def _test_key(self) -> str:   return 'board'

    def get_steps(self):
        return [
            TestStep(name='Capture + Warp', display_text='WARP', action=self._run,
                     wait_for_input=True, input_type='clock', timeout_seconds=30.0,
                     success_message='WARP OK', failure_message='WARP FAIL'),
        ]

    def _run(self) -> bool:
        _print_remote_viewing_hint('board')
        print('  Press clock to capture and display warped board with grid...')
        self._node.corners_received = False
        self._node.capture()
        if not self._wait_for_corners(6.0):
            print('  ✗ No corners. Run corner test first.')
            return False

        raw, corners = self._node.get_snapshot()
        if raw is None or corners is None:
            return False

        warped = _warp_perspective(raw, corners)
        if warped is None:
            print('  ✗ Perspective warp failed')
            return False

        annotated_raw    = self._annotate_raw_corners(raw.copy(), corners)
        annotated_warped = self._annotate_warped_grid(warped.copy())

        # Side-by-side: resize raw to same height as warped
        h = WARP_SIZE
        scale = h / annotated_raw.shape[0]
        w_scaled = int(annotated_raw.shape[1] * scale)
        raw_resized = cv2.resize(annotated_raw, (w_scaled, h))

        combined = np.hstack([raw_resized, annotated_warped])

        # Banner at top
        banner = np.zeros((40, combined.shape[1], 3), dtype=np.uint8)
        _label(banner, 'LEFT: Camera + corners    RIGHT: Warped board + grid',
               (10, 28), (255, 255, 255), scale=0.65, thickness=1, bg=False)
        combined = np.vstack([banner, combined])

        path = self._publish_and_save('board', combined)
        print(f'  ✓ Board detected and warped successfully')
        print(f'  → Image saved: {path}')
        return True

    def _annotate_raw_corners(self, img: np.ndarray, corners: np.ndarray) -> np.ndarray:
        hull = corners.astype(np.int32).reshape((-1, 1, 2))
        cv2.polylines(img, [hull], True, (0, 255, 0), 3)
        for i, (cx, cy) in enumerate(corners):
            cv2.circle(img, (int(cx), int(cy)), 12, CORNER_COLORS[i], -1)
            cv2.circle(img, (int(cx), int(cy)), 12, (255, 255, 255), 2)
        _label(img, 'RAW + CORNERS', (10, 30), (0, 255, 0), scale=0.7, thickness=2)
        return img

    def _annotate_warped_grid(self, img: np.ndarray) -> np.ndarray:
        # Draw alternating square colors
        for row in range(8):
            for col in range(8):
                is_light = (row + col) % 2 == 0
                color    = EMPTY_SQ_LIGHT if is_light else EMPTY_SQ_DARK
                x1, y1   = col * SQ_PX, row * SQ_PX
                cv2.rectangle(img, (x1, y1), (x1 + SQ_PX, y1 + SQ_PX), color, -1)

                # Square name in center
                sq = _square_name(row, col)
                tx = x1 + SQ_PX // 2 - 10
                ty = y1 + SQ_PX // 2 + 5
                cv2.putText(img, sq, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.32,
                            (180, 180, 180) if is_light else (220, 220, 220),
                            1, cv2.LINE_AA)

        # Grid lines
        _draw_grid(img, GRID_COLOR, thickness=1)

        # File labels (a–h) at bottom
        for col in range(8):
            lbl = chr(ord('a') + col)
            cx  = col * SQ_PX + SQ_PX // 2 - 5
            _label(img, lbl, (cx, WARP_SIZE - 4), (255, 215, 0), scale=0.45, bg=False)

        # Rank labels (1–8 from bottom to top) on left
        for row in range(8):
            rank = str(8 - row)
            ry   = row * SQ_PX + SQ_PX // 2 + 5
            _label(img, rank, (2, ry), (255, 215, 0), scale=0.45, bg=False)

        _label(img, 'WARPED + GRID', (10, 20), (0, 255, 255), scale=0.65, thickness=2)
        return img


# ═════════════════════════════════════════════════════════════════════════════
# TEST 4: Square Mapping
# ═════════════════════════════════════════════════════════════════════════════

class SquareMappingTest(VisionDetailBase):
    """
    Show all 64 square labels on warped board. Then interactive mode:
    user types a square name and it's highlighted in gold on the debug image.

    Tests correctness of the warp-to-square coordinate mapping.
    """

    @property
    def name(self) -> str:        return 'Square Mapping'
    @property
    def description(self) -> str: return 'Warp + all 64 square labels, interactive highlight mode'
    @property
    def _test_key(self) -> str:   return 'squares'

    def get_steps(self):
        return [
            TestStep(name='Full Grid Map',    display_text='SQMAP',    action=self._run_map,
                     wait_for_input=True, input_type='clock', timeout_seconds=30.0,
                     success_message='MAP OK', failure_message='MAP FAIL'),
            TestStep(name='Interactive Mode', display_text='INTERACT', action=self._run_interactive,
                     wait_for_input=False,
                     success_message='DONE', failure_message='DONE'),
        ]

    def _run_map(self) -> bool:
        _print_remote_viewing_hint('squares')
        print('  Press clock to warp board and label all 64 squares...')
        self._node.capture()
        if not self._wait_for_corners(6.0):
            print('  ✗ No corners detected')
            return False

        raw, corners = self._node.get_snapshot()
        if raw is None or corners is None:
            return False

        warped = _warp_perspective(raw, corners)
        if warped is None:
            print('  ✗ Warp failed')
            return False

        annotated = self._annotate_all_squares(warped.copy())
        path = self._publish_and_save('squares', annotated)
        print(f'  ✓ All 64 squares labeled on warped board')
        print(f'  → Image saved: {path}')
        return True

    def _run_interactive(self) -> bool:
        print()
        print('  INTERACTIVE SQUARE HIGHLIGHT MODE')
        print('  ──────────────────────────────────')
        print('  Type a square name (e.g. e4, h8, a1) and press Enter.')
        print('  Board will re-capture and highlight that square in gold.')
        print('  Type "q" to exit interactive mode.')
        print()

        while True:
            try:
                sq_input = input('  Square (or q): ').strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if sq_input == 'q':
                break

            # Validate
            if (len(sq_input) != 2 or sq_input[0] not in 'abcdefgh'
                    or sq_input[1] not in '12345678'):
                print('  Invalid square. Use format: a1 – h8')
                continue

            self._node.capture()
            time.sleep(0.5)
            raw, corners = self._node.get_snapshot()

            if raw is None or corners is None:
                print('  [WARN] No image/corners')
                continue

            warped = _warp_perspective(raw, corners)
            if warped is None:
                continue

            annotated = self._annotate_all_squares(warped.copy(), highlight=sq_input)
            self._publish_and_save('squares', annotated)
            self._node.publish_debug('squares', annotated)
            print(f'  ✓ {sq_input} highlighted and published')

        return True

    def _annotate_all_squares(
        self, img: np.ndarray, highlight: str = ''
    ) -> np.ndarray:
        for row in range(8):
            for col in range(8):
                is_light = (row + col) % 2 == 0
                bg       = EMPTY_SQ_LIGHT if is_light else EMPTY_SQ_DARK
                sq       = _square_name(row, col)
                x1, y1   = col * SQ_PX, row * SQ_PX
                x2, y2   = x1 + SQ_PX, y1 + SQ_PX

                cv2.rectangle(img, (x1, y1), (x2, y2), bg, -1)

                if sq == highlight:
                    cv2.rectangle(img, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2),
                                  HIGHLIGHT_COLOR, -1)
                    _label(img, sq,
                           (x1 + SQ_PX//2 - 10, y1 + SQ_PX//2 + 5),
                           (0, 0, 0), scale=0.65, thickness=2, bg=False)
                else:
                    txt_col = (80, 50, 20) if is_light else (200, 200, 200)
                    cv2.putText(img, sq,
                                (x1 + 4, y1 + SQ_PX//2 + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.32, txt_col, 1, cv2.LINE_AA)

        _draw_grid(img, GRID_COLOR, 1)

        # File + rank labels
        for col in range(8):
            lbl = chr(ord('a') + col)
            _label(img, lbl, (col * SQ_PX + SQ_PX//2 - 5, WARP_SIZE - 4),
                   (255, 215, 0), scale=0.40, bg=False)
        for row in range(8):
            _label(img, str(8 - row), (2, row * SQ_PX + SQ_PX//2 + 5),
                   (255, 215, 0), scale=0.40, bg=False)

        hl_txt = f'  HIGHLIGHT: {highlight.upper()}' if highlight else ''
        _label(img, f'ALL 64 SQUARES{hl_txt}',
               (10, 20), (0, 255, 255), scale=0.55, thickness=2)

        return img


