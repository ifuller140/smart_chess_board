#!/usr/bin/env python3
"""
Detection Test Script for Chess Vision.

Test different stages of the vision pipeline:
- Board detection
- Piece detection
- Full pipeline

Usage:
    # Test board detection on a captured image:
    python3 scripts/test_detection.py --image /tmp/chess_camera_test/test_capture.jpg

    # Test with live camera:
    python3 scripts/test_detection.py --camera 0

    # Test just board detection:
    python3 scripts/test_detection.py --image test.jpg --step board

    # Test full pipeline with calibration:
    python3 scripts/test_detection.py --image test.jpg --calibration config/calibration.npz
"""

import sys
import os
import argparse
import time

# Add parent to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
import numpy as np
from chess_perception.calibration import CalibrationResult, ImageUndistorter
from chess_perception.board_detection import BoardDetector
from chess_perception.piece_detection import PieceDetector, visualize_board_state
from chess_perception.vision_pipeline import ChessVisionPipeline


def test_board_detection(image: np.ndarray, undistorter=None):
    """Test board detection only."""
    print("\n=== BOARD DETECTION TEST ===")
    
    if undistorter:
        print("Applying lens distortion correction...")
        image = undistorter.undistort(image)
    
    detector = BoardDetector()
    
    start = time.time()
    geometry = detector.detect(image)
    elapsed = time.time() - start
    
    print(f"Detection time: {elapsed*1000:.1f}ms")
    
    if geometry is None:
        print("FAILED: Board not detected")
        debug = detector.draw_debug(image, None)
    else:
        print("SUCCESS: Board detected!")
        print(f"Corners:\n{geometry.corners}")
        
        debug = detector.draw_debug(image, geometry)
        warped = detector.warp_image(image, geometry)
        
        # Draw grid on warped
        for i in range(1, 8):
            cv2.line(warped, (i * 60, 0), (i * 60, 480), (0, 255, 0), 1)
            cv2.line(warped, (0, i * 60), (480, i * 60), (0, 255, 0), 1)
        
        # Show side by side
        h1, w1 = debug.shape[:2]
        scale = 480 / h1
        debug_resized = cv2.resize(debug, (int(w1 * scale), 480))
        combined = np.hstack([debug_resized, warped])
        
        cv2.imshow("Board Detection (Original | Warped)", combined)
        print("\nPress any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def test_piece_detection(image: np.ndarray, undistorter=None, calibrate: bool = False):
    """Test piece detection."""
    print("\n=== PIECE DETECTION TEST ===")
    
    if undistorter:
        print("Applying lens distortion correction...")
        image = undistorter.undistort(image)
    
    board_detector = BoardDetector()
    piece_detector = PieceDetector()
    
    # Detect board
    geometry = board_detector.detect(image)
    if geometry is None:
        print("FAILED: Board not detected")
        return
    
    print("Board detected")
    warped = board_detector.warp_image(image, geometry)
    
    if calibrate:
        print("Calibrating board colors (assumes empty board)...")
        piece_detector.calibrate_board_colors(warped)
    
    # Detect pieces
    start = time.time()
    result = piece_detector.detect(warped, debug=True)
    elapsed = time.time() - start
    
    print(f"Detection time: {elapsed*1000:.1f}ms")
    print("\nDetected board state:")
    print(visualize_board_state(result))
    print("\nPieces found:", result.to_piece_list())
    
    if result.debug_image is not None:
        cv2.imshow("Piece Detection", result.debug_image)
        print("\nPress any key to continue...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def test_full_pipeline(image: np.ndarray, calibration_file: str = None):
    """Test full vision pipeline."""
    print("\n=== FULL PIPELINE TEST ===")
    
    pipeline = ChessVisionPipeline(calibration_file=calibration_file)
    
    start = time.time()
    result = pipeline.process_image(image, debug=True)
    elapsed = time.time() - start
    
    print(f"Total processing time: {elapsed*1000:.1f}ms")
    
    if not result.success:
        print(f"FAILED: {result.error_message}")
        return
    
    print("SUCCESS!")
    print("\nDetected board state:")
    print(visualize_board_state(result.pieces))
    print("\nPieces:", pipeline.get_board_state_dict(result))
    
    if result.pieces.debug_image is not None:
        cv2.imshow("Vision Pipeline Result", result.pieces.debug_image)
        print("\nPress any key to exit...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


def capture_from_camera(camera_id: int) -> np.ndarray:
    """Capture a frame from camera."""
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera {camera_id}")
    
    # Flush buffer
    for _ in range(5):
        cap.read()
    
    ret, frame = cap.read()
    cap.release()
    
    if not ret:
        raise RuntimeError("Failed to capture frame")
    
    return frame


def main():
    parser = argparse.ArgumentParser(
        description="Test chess vision detection",
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--image", "-i", help="Path to test image")
    source.add_argument("--camera", "-c", type=int, help="Camera ID for live capture")
    
    parser.add_argument("--step", choices=["board", "pieces", "full"],
                        default="full", help="Which step to test (default: full)")
    parser.add_argument("--calibration", help="Path to calibration.npz")
    parser.add_argument("--calibrate-colors", action="store_true",
                        help="Calibrate board colors (use empty board image)")
    
    args = parser.parse_args()
    
    # Load image
    if args.image:
        print(f"Loading image: {args.image}")
        image = cv2.imread(args.image)
        if image is None:
            print(f"Could not load: {args.image}")
            return 1
    else:
        print(f"Capturing from camera {args.camera}...")
        try:
            image = capture_from_camera(args.camera)
            print(f"Captured {image.shape[1]}x{image.shape[0]} image")
        except RuntimeError as e:
            print(f"Error: {e}")
            return 1
    
    # Load calibration
    undistorter = None
    if args.calibration and os.path.exists(args.calibration):
        print(f"Loading calibration: {args.calibration}")
        calibration = CalibrationResult.load(args.calibration)
        undistorter = ImageUndistorter(calibration)
    
    # Run test
    if args.step == "board":
        test_board_detection(image, undistorter)
    elif args.step == "pieces":
        test_piece_detection(image, undistorter, calibrate=args.calibrate_colors)
    else:
        test_full_pipeline(image, args.calibration)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
