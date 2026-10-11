#!/usr/bin/env python3
"""
Chess Vision - Package Initialization.

Main modules:
- calibration: Camera calibration and lens distortion correction
- board_detection: Board detection and homography transform
- piece_detection: Piece detection on squares
- vision_pipeline: Main pipeline orchestrator
"""

from .calibration import CameraCalibrator, CalibrationResult, ImageUndistorter
from .board_detection import BoardDetector, BoardGeometry, get_square_roi
from .piece_detection import PieceDetector, PieceDetectionResult, SquareState
from .vision_pipeline import ChessVisionPipeline, VisionResult

__all__ = [
    # Calibration
    'CameraCalibrator',
    'CalibrationResult', 
    'ImageUndistorter',
    
    # Board Detection
    'BoardDetector',
    'BoardGeometry',
    'get_square_roi',
    
    # Piece Detection
    'PieceDetector',
    'PieceDetectionResult',
    'SquareState',
    
    # Pipeline
    'ChessVisionPipeline',
    'VisionResult',
]
