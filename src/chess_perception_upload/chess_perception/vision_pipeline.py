#!/usr/bin/env python3
"""
Chess Vision Pipeline - Main Orchestrator.

Combines all vision modules into a unified pipeline:
1. Capture image
2. Undistort (if calibrated)
3. Detect board
4. Warp to orthographic view
5. Detect pieces
"""

import os
import numpy as np
import cv2
from typing import Optional, Dict
from dataclasses import dataclass

from .calibration import CalibrationResult, ImageUndistorter
from .board_detection import BoardDetector, BoardGeometry
from .piece_detection import PieceDetector, PieceDetectionResult, SquareState


@dataclass
class VisionResult:
    """Complete result from vision pipeline."""
    # Raw captured image
    raw_image: np.ndarray
    
    # Undistorted image (or same as raw if no calibration)
    undistorted_image: np.ndarray
    
    # Board detection result
    board_geometry: Optional[BoardGeometry]
    
    # Warped orthographic image (or None if no board found)
    warped_image: Optional[np.ndarray]
    
    # Piece detection result
    pieces: Optional[PieceDetectionResult]
    
    # Processing success
    success: bool
    error_message: str = ""


class ChessVisionPipeline:
    """
    Main vision pipeline for chess board analysis.
    
    Usage:
        pipeline = ChessVisionPipeline()
        pipeline.load_calibration("calibration.npz")  # Optional
        
        # Using OpenCV capture
        result = pipeline.process_image(cv2.imread("board.jpg"))
        
        # Or with built-in camera
        result = pipeline.capture_and_process(camera_id=0)
    """
    
    # Default config directories
    CONFIG_DIR = os.path.join(os.path.dirname(__file__), "config")
    DEFAULT_CALIBRATION = os.path.join(CONFIG_DIR, "calibration.npz")
    
    def __init__(self, calibration_file: str = None):
        """
        Initialize pipeline.
        
        Args:
            calibration_file: Path to calibration.npz (optional)
        """
        self.undistorter = None
        self.board_detector = BoardDetector()
        self.piece_detector = PieceDetector()
        
        # Try to load calibration
        if calibration_file and os.path.exists(calibration_file):
            self.load_calibration(calibration_file)
        elif os.path.exists(self.DEFAULT_CALIBRATION):
            self.load_calibration(self.DEFAULT_CALIBRATION)
    
    def load_calibration(self, filepath: str):
        """Load lens distortion calibration."""
        try:
            calibration = CalibrationResult.load(filepath)
            self.undistorter = ImageUndistorter(calibration)
            print(f"Loaded calibration from: {filepath}")
            print(f"  RMS error: {calibration.rms_error:.4f}")
        except Exception as e:
            print(f"Failed to load calibration: {e}")
            self.undistorter = None
    
    def calibrate_piece_colors(self, empty_board_image: np.ndarray):
        """
        Calibrate piece detector colors from empty board image.
        
        Args:
            empty_board_image: Image of empty chess board
        """
        # Process through board detection first
        if self.undistorter:
            empty_board_image = self.undistorter.undistort(empty_board_image)
        
        geometry = self.board_detector.detect(empty_board_image)
        if geometry is None:
            print("Cannot calibrate: board not detected")
            return
        
        warped = self.board_detector.warp_image(empty_board_image, geometry)
        self.piece_detector.calibrate_board_colors(warped)
    
    def process_image(self, image: np.ndarray, debug: bool = False) -> VisionResult:
        """
        Process a single image through the full pipeline.
        
        Args:
            image: BGR image from camera
            debug: If True, generate debug visualizations
            
        Returns:
            VisionResult with all pipeline outputs
        """
        raw_image = image.copy()
        
        # 1. Undistort
        if self.undistorter:
            undistorted = self.undistorter.undistort(image)
        else:
            undistorted = image
        
        # 2. Detect board
        geometry = self.board_detector.detect(undistorted)
        
        if geometry is None:
            return VisionResult(
                raw_image=raw_image,
                undistorted_image=undistorted,
                board_geometry=None,
                warped_image=None,
                pieces=None,
                success=False,
                error_message="Board not detected"
            )
        
        # 3. Warp to orthographic
        warped = self.board_detector.warp_image(undistorted, geometry)
        
        # 4. Detect pieces
        pieces = self.piece_detector.detect(warped, debug=debug)
        
        return VisionResult(
            raw_image=raw_image,
            undistorted_image=undistorted,
            board_geometry=geometry,
            warped_image=warped,
            pieces=pieces,
            success=True
        )
    
    def capture_and_process(self, camera_id: int = 0, debug: bool = False) -> VisionResult:
        """
        Capture from camera and process.
        
        Args:
            camera_id: OpenCV camera ID
            debug: If True, generate debug visualizations
            
        Returns:
            VisionResult
        """
        cap = cv2.VideoCapture(camera_id)
        
        if not cap.isOpened():
            return VisionResult(
                raw_image=np.zeros((480, 640, 3), dtype=np.uint8),
                undistorted_image=np.zeros((480, 640, 3), dtype=np.uint8),
                board_geometry=None,
                warped_image=None,
                pieces=None,
                success=False,
                error_message=f"Cannot open camera {camera_id}"
            )
        
        # Flush buffer and capture
        for _ in range(5):
            cap.read()
        
        ret, frame = cap.read()
        cap.release()
        
        if not ret:
            return VisionResult(
                raw_image=np.zeros((480, 640, 3), dtype=np.uint8),
                undistorted_image=np.zeros((480, 640, 3), dtype=np.uint8),
                board_geometry=None,
                warped_image=None,
                pieces=None,
                success=False,
                error_message="Failed to capture image"
            )
        
        return self.process_image(frame, debug=debug)
    
    def get_board_state_dict(self, result: VisionResult) -> Dict[str, str]:
        """
        Convert detection result to dictionary of square -> piece.
        
        Args:
            result: VisionResult from process_image()
            
        Returns:
            Dict like {'e2': 'W', 'd7': 'B', ...}
        """
        if result.pieces is None:
            return {}
        
        state = {}
        for row in range(8):
            for col in range(8):
                square_state = SquareState(result.pieces.board_state[row, col])
                if square_state != SquareState.EMPTY:
                    file = chr(ord('a') + col)
                    rank = 8 - row
                    square = f"{file}{rank}"
                    state[square] = 'W' if square_state == SquareState.WHITE_PIECE else 'B'
        
        return state


# Convenience function for quick testing
def quick_test(image_path: str, calibration_path: str = None):
    """Quick test of the vision pipeline."""
    from .piece_detection import visualize_board_state
    
    img = cv2.imread(image_path)
    if img is None:
        print(f"Could not load: {image_path}")
        return
    
    pipeline = ChessVisionPipeline(calibration_file=calibration_path)
    result = pipeline.process_image(img, debug=True)
    
    if result.success:
        print("\n=== Vision Pipeline Result ===")
        print(visualize_board_state(result.pieces))
        print(f"\nPieces: {pipeline.get_board_state_dict(result)}")
        
        # Display
        if result.pieces.debug_image is not None:
            cv2.imshow("Detection", result.pieces.debug_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    else:
        print(f"Pipeline failed: {result.error_message}")


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        quick_test(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else None)
    else:
        print("Usage: python -m chess_perception.vision_pipeline <image> [calibration.npz]")
