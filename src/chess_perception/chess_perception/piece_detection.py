#!/usr/bin/env python3
"""
Piece Detection Module for Chess Vision.

Detects pieces on the chess board using color-based blob detection.
Works on orthographic (warped) board image from BoardDetector.

Piece colors: Black and white pieces
Board colors: Tan (light squares) and dark brown (dark squares)
"""

import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import IntEnum


class SquareState(IntEnum):
    """State of a square on the board."""
    EMPTY = 0
    WHITE_PIECE = 1
    BLACK_PIECE = 2


@dataclass
class PieceDetectionResult:
    """Result of piece detection for the board."""
    # 8x8 grid of states (row 0 = rank 8, col 0 = file a)
    board_state: np.ndarray
    
    # Confidence scores for each square (0-1)
    confidence: np.ndarray
    
    # Debug image with annotations
    debug_image: Optional[np.ndarray] = None
    
    def get_square(self, file: str, rank: int) -> SquareState:
        """
        Get state of a specific square.
        
        Args:
            file: 'a' through 'h'
            rank: 1 through 8
            
        Returns:
            SquareState
        """
        col = ord(file.lower()) - ord('a')
        row = 8 - rank
        return SquareState(self.board_state[row, col])
    
    def to_piece_list(self) -> List[str]:
        """
        Convert to list of squares with pieces.
        
        Returns:
            List of strings like ['e1:W', 'd8:B', ...]
        """
        pieces = []
        for row in range(8):
            for col in range(8):
                state = self.board_state[row, col]
                if state != SquareState.EMPTY:
                    file = chr(ord('a') + col)
                    rank = 8 - row
                    color = 'W' if state == SquareState.WHITE_PIECE else 'B'
                    pieces.append(f"{file}{rank}:{color}")
        return pieces


class PieceDetector:
    """
    Detects chess pieces on each square using color analysis.
    
    Strategy:
    1. For each square, extract center region (avoiding edges)
    2. Calculate histogram of brightness
    3. Detect anomalies compared to expected board square color
    4. Classify piece as white or black based on brightness
    """
    
    # Image parameters (must match BoardDetector output)
    BOARD_SIZE = 480  # pixels
    SQUARE_SIZE = 60  # pixels per square
    
    # Detection thresholds (tuned for tan/brown board with B/W pieces)
    PIECE_DETECTION_THRESHOLD = 30  # Min std dev to detect piece vs empty
    WHITE_PIECE_THRESHOLD = 150     # Min brightness for white piece
    BLACK_PIECE_THRESHOLD = 80      # Max brightness for black piece
    
    def __init__(self, 
                 board_size: int = 480,
                 piece_threshold: float = 30,
                 white_threshold: float = 150,
                 black_threshold: float = 80):
        """
        Initialize piece detector.
        
        Args:
            board_size: Size of input orthographic image
            piece_threshold: Std dev threshold for piece detection
            white_threshold: Min brightness to classify as white piece
            black_threshold: Max brightness to classify as black piece
        """
        self.board_size = board_size
        self.square_size = board_size // 8
        self.piece_threshold = piece_threshold
        self.white_threshold = white_threshold
        self.black_threshold = black_threshold
        
        # Calibrated board colors (can be updated via calibrate())
        self.light_square_color = np.array([200, 180, 150])  # Tan
        self.dark_square_color = np.array([100, 80, 60])     # Dark brown
    
    def calibrate_board_colors(self, warped_image: np.ndarray):
        """
        Calibrate expected board colors from an empty board image.
        
        Args:
            warped_image: 480x480 orthographic image of empty board
        """
        light_samples = []
        dark_samples = []
        
        for row in range(8):
            for col in range(8):
                roi = self._get_square_center_roi(warped_image, row, col)
                mean_color = np.mean(roi, axis=(0, 1))
                
                # Light squares: (row+col) is even
                if (row + col) % 2 == 0:
                    light_samples.append(mean_color)
                else:
                    dark_samples.append(mean_color)
        
        self.light_square_color = np.mean(light_samples, axis=0)
        self.dark_square_color = np.mean(dark_samples, axis=0)
        
        print(f"Calibrated light square color: {self.light_square_color}")
        print(f"Calibrated dark square color: {self.dark_square_color}")
    
    def detect(self, warped_image: np.ndarray, debug: bool = False) -> PieceDetectionResult:
        """
        Detect pieces on all squares.
        
        Args:
            warped_image: 480x480 orthographic board image
            debug: If True, generate debug visualization
            
        Returns:
            PieceDetectionResult
        """
        board_state = np.zeros((8, 8), dtype=np.int8)
        confidence = np.zeros((8, 8), dtype=np.float32)
        
        debug_image = warped_image.copy() if debug else None
        
        for row in range(8):
            for col in range(8):
                state, conf = self._analyze_square(warped_image, row, col)
                board_state[row, col] = state
                confidence[row, col] = conf
                
                if debug and state != SquareState.EMPTY:
                    self._draw_square_debug(debug_image, row, col, state, conf)
        
        return PieceDetectionResult(
            board_state=board_state,
            confidence=confidence,
            debug_image=debug_image
        )
    
    def _get_square_center_roi(self, image: np.ndarray, row: int, col: int, 
                                margin_pct: float = 0.25) -> np.ndarray:
        """
        Get center region of a square (avoiding edges where piece might overlap).
        
        Args:
            image: Warped board image
            row, col: Square indices
            margin_pct: Percentage of square to exclude from edges
            
        Returns:
            Cropped ROI
        """
        margin = int(self.square_size * margin_pct)
        
        x1 = col * self.square_size + margin
        x2 = (col + 1) * self.square_size - margin
        y1 = row * self.square_size + margin
        y2 = (row + 1) * self.square_size - margin
        
        return image[y1:y2, x1:x2]
    
    def _analyze_square(self, image: np.ndarray, row: int, col: int) -> Tuple[SquareState, float]:
        """
        Analyze a single square for piece presence and color.
        
        Returns:
            (state, confidence)
        """
        roi = self._get_square_center_roi(image, row, col)
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        
        # Calculate statistics
        mean_val = np.mean(gray_roi)
        std_val = np.std(gray_roi)
        
        # Get expected square color (light or dark)
        is_light_square = (row + col) % 2 == 0
        expected_brightness = np.mean(self.light_square_color[:3] if is_light_square 
                                       else self.dark_square_color[:3])
        
        # Detect piece by deviation from expected
        brightness_diff = abs(mean_val - expected_brightness)
        
        # High std dev OR significant brightness difference indicates piece
        has_piece = std_val > self.piece_threshold or brightness_diff > 40
        
        if not has_piece:
            return SquareState.EMPTY, 0.0
        
        # Classify piece color
        if mean_val > self.white_threshold:
            return SquareState.WHITE_PIECE, min(1.0, brightness_diff / 100)
        elif mean_val < self.black_threshold:
            return SquareState.BLACK_PIECE, min(1.0, brightness_diff / 100)
        else:
            # Ambiguous - use comparison to expected
            if mean_val > expected_brightness:
                return SquareState.WHITE_PIECE, 0.5
            else:
                return SquareState.BLACK_PIECE, 0.5
    
    def _draw_square_debug(self, image: np.ndarray, row: int, col: int,
                           state: SquareState, confidence: float):
        """Draw debug annotation on a square."""
        x1 = col * self.square_size
        y1 = row * self.square_size
        x2 = x1 + self.square_size
        y2 = y1 + self.square_size
        
        # Color based on piece
        if state == SquareState.WHITE_PIECE:
            color = (0, 255, 0)  # Green for white
            label = "W"
        else:
            color = (0, 0, 255)  # Red for black
            label = "B"
        
        # Thickness based on confidence
        thickness = 2 if confidence > 0.7 else 1
        
        cv2.rectangle(image, (x1 + 2, y1 + 2), (x2 - 2, y2 - 2), color, thickness)
        cv2.putText(image, label, (x1 + 5, y2 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)


def visualize_board_state(result: PieceDetectionResult) -> str:
    """
    Create ASCII visualization of detected board state.
    
    Returns:
        Multi-line string representing the board
    """
    symbols = {
        SquareState.EMPTY: '.',
        SquareState.WHITE_PIECE: 'W',
        SquareState.BLACK_PIECE: 'B'
    }
    
    lines = ["  a b c d e f g h"]
    for row in range(8):
        rank = 8 - row
        row_str = f"{rank} "
        for col in range(8):
            state = SquareState(result.board_state[row, col])
            row_str += symbols[state] + " "
        lines.append(row_str + str(rank))
    lines.append("  a b c d e f g h")
    
    return "\n".join(lines)


# CLI for standalone testing
if __name__ == "__main__":
    import argparse
    from board_detection import BoardDetector
    
    parser = argparse.ArgumentParser(description="Piece Detection Test")
    parser.add_argument("image", help="Path to test image")
    parser.add_argument("--calibrate", "-c", action="store_true",
                        help="Calibrate board colors (use empty board image)")
    
    args = parser.parse_args()
    
    img = cv2.imread(args.image)
    if img is None:
        print(f"Could not load: {args.image}")
        exit(1)
    
    # Detect board
    board_detector = BoardDetector()
    geometry = board_detector.detect(img)
    
    if geometry is None:
        print("Board detection failed")
        exit(1)
    
    # Warp to orthographic
    warped = board_detector.warp_image(img, geometry)
    
    # Detect pieces
    piece_detector = PieceDetector()
    
    if args.calibrate:
        print("Calibrating board colors...")
        piece_detector.calibrate_board_colors(warped)
    
    result = piece_detector.detect(warped, debug=True)
    
    # Print results
    print("\nDetected board state:")
    print(visualize_board_state(result))
    print("\nPieces found:", result.to_piece_list())
    
    # Show debug image
    if result.debug_image is not None:
        cv2.imshow("Piece Detection", result.debug_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
