#!/usr/bin/env python3
"""
Camera Calibration Module for Chess Vision.

Uses OpenCV camera calibration with a checkerboard pattern.
For chess board: 7x7 internal corners (8x8 squares).

Physical Setup:
- Board: 12x12 inches, 1.5-inch squares
- Camera: Pi Camera v2, 10.5" high, ~30° tilt
"""

import os
import glob
import numpy as np
import cv2
from typing import Optional, Tuple, List
from dataclasses import dataclass


@dataclass
class CalibrationResult:
    """Container for calibration data."""
    camera_matrix: np.ndarray
    dist_coeffs: np.ndarray
    rvecs: List[np.ndarray]
    tvecs: List[np.ndarray]
    rms_error: float
    image_size: Tuple[int, int]
    
    def save(self, filepath: str):
        """Save calibration to OpenCV YAML file (compatible with cv2.FileStorage)."""
        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_WRITE)
        fs.write('camera_matrix', self.camera_matrix)
        fs.write('dist_coeffs', self.dist_coeffs)
        fs.write('image_size', np.array(list(self.image_size), dtype=np.int32))
        fs.write('rms_error', float(self.rms_error))
        fs.release()
        print(f"Calibration saved to: {filepath}")

    @classmethod
    def load(cls, filepath: str) -> 'CalibrationResult':
        """Load calibration from OpenCV YAML file."""
        fs = cv2.FileStorage(filepath, cv2.FILE_STORAGE_READ)
        if not fs.isOpened():
            raise IOError(f"Cannot open calibration file: {filepath}")
        camera_matrix = fs.getNode('camera_matrix').mat()
        dist_coeffs = fs.getNode('dist_coeffs').mat()
        image_size_mat = fs.getNode('image_size').mat()
        rms_error = float(fs.getNode('rms_error').real())
        fs.release()
        return cls(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rvecs=[],
            tvecs=[],
            rms_error=rms_error,
            image_size=tuple(int(x) for x in image_size_mat.flatten())
        )


class CameraCalibrator:
    """
    Camera calibration using chess board pattern.
    
    Uses the chess board itself (7x7 internal corners for 8x8 board)
    with known square size of 1.5 inches = 38.1 mm.
    """
    
    # Chess board pattern: 7x7 internal corners for 8x8 squares
    PATTERN_SIZE = (7, 7)
    
    # Square size in mm (1.5 inches = 38.1 mm)
    SQUARE_SIZE_MM = 38.1
    
    def __init__(self, pattern_size: Tuple[int, int] = None, square_size_mm: float = None):
        """
        Initialize calibrator.
        
        Args:
            pattern_size: (cols, rows) of internal corners. Default (7,7) for chess board.
            square_size_mm: Size of each square in mm. Default 38.1 (1.5 inches).
        """
        self.pattern_size = pattern_size or self.PATTERN_SIZE
        self.square_size = square_size_mm or self.SQUARE_SIZE_MM
        
        # Prepare object points (3D points in real world space)
        # Z=0 since the board is flat
        self.objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:self.pattern_size[0], 
                                     0:self.pattern_size[1]].T.reshape(-1, 2)
        self.objp *= self.square_size
        
        # Storage for calibration points
        self.obj_points = []  # 3D points
        self.img_points = []  # 2D points
        self.image_size = None
        
    def find_corners(self, image: np.ndarray, draw: bool = False) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Find chessboard corners in an image.
        
        Args:
            image: BGR or grayscale image
            draw: If True, return annotated image
            
        Returns:
            (found, corners) where corners is Nx1x2 array of corner points
        """
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        self.image_size = (gray.shape[1], gray.shape[0])
        
        # Find corners with subpixel accuracy
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        
        if found:
            # Refine corners
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        return found, corners
    
    def add_calibration_image(self, image: np.ndarray) -> bool:
        """
        Add an image for calibration.
        
        Args:
            image: Image with visible chess board
            
        Returns:
            True if corners were found and added
        """
        found, corners = self.find_corners(image)
        
        if found:
            self.obj_points.append(self.objp)
            self.img_points.append(corners)
            return True
        return False
    
    def calibrate(self) -> Optional[CalibrationResult]:
        """
        Run camera calibration.
        
        Returns:
            CalibrationResult or None if insufficient data
        """
        if len(self.obj_points) < 3:
            print(f"Need at least 3 images, have {len(self.obj_points)}")
            return None
        
        print(f"Calibrating with {len(self.obj_points)} images...")
        
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            self.obj_points,
            self.img_points,
            self.image_size,
            None,
            None
        )
        
        print(f"Calibration RMS error: {ret:.4f}")
        print(f"Camera matrix:\n{camera_matrix}")
        print(f"Distortion coefficients: {dist_coeffs.ravel()}")
        
        return CalibrationResult(
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            rvecs=rvecs,
            tvecs=tvecs,
            rms_error=ret,
            image_size=self.image_size
        )


class ImageUndistorter:
    """Apply lens distortion correction to images."""
    
    def __init__(self, calibration: CalibrationResult):
        """
        Initialize undistorter.
        
        Args:
            calibration: CalibrationResult from CameraCalibrator
        """
        self.calibration = calibration
        
        # Compute optimal new camera matrix
        self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
            calibration.camera_matrix,
            calibration.dist_coeffs,
            calibration.image_size,
            alpha=1,  # Keep all pixels
            newImgSize=calibration.image_size
        )
        
        # Precompute undistort maps for speed
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            calibration.camera_matrix,
            calibration.dist_coeffs,
            None,
            self.new_camera_matrix,
            calibration.image_size,
            cv2.CV_16SC2
        )
    
    def undistort(self, image: np.ndarray, crop: bool = True) -> np.ndarray:
        """
        Undistort an image.
        
        Args:
            image: Input distorted image
            crop: If True, crop to valid region
            
        Returns:
            Undistorted image
        """
        # Use precomputed maps for speed
        undistorted = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
        
        if crop:
            x, y, w, h = self.roi
            if w > 0 and h > 0:
                undistorted = undistorted[y:y+h, x:x+w]
        
        return undistorted


def calibrate_from_images(image_paths: List[str], output_path: str) -> Optional[CalibrationResult]:
    """
    Convenience function to calibrate from a list of image paths.
    
    Args:
        image_paths: List of paths to calibration images
        output_path: Path to save calibration result
        
    Returns:
        CalibrationResult or None
    """
    calibrator = CameraCalibrator()
    
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"Could not load: {path}")
            continue
        
        if calibrator.add_calibration_image(img):
            print(f"✓ Found corners in: {os.path.basename(path)}")
        else:
            print(f"✗ No corners in: {os.path.basename(path)}")
    
    result = calibrator.calibrate()
    
    if result:
        result.save(output_path)
    
    return result


# CLI for standalone use
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Camera Calibration Tool")
    parser.add_argument("--images", "-i", required=True,
                        help="Directory containing calibration images or glob pattern")
    parser.add_argument("--output", "-o", default="calibration.yaml",
                        help="Output file path (default: calibration.yaml)")
    parser.add_argument("--pattern", "-p", default="7x7",
                        help="Pattern size as WxH (default: 7x7 for chess board)")
    parser.add_argument("--square-size", "-s", type=float, default=38.1,
                        help="Square size in mm (default: 38.1 = 1.5 inches)")
    
    args = parser.parse_args()
    
    # Parse pattern
    pw, ph = map(int, args.pattern.split('x'))
    
    # Find images
    if os.path.isdir(args.images):
        patterns = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        image_paths = []
        for p in patterns:
            image_paths.extend(glob.glob(os.path.join(args.images, p)))
    else:
        image_paths = glob.glob(args.images)
    
    if not image_paths:
        print(f"No images found in: {args.images}")
        exit(1)
    
    print(f"Found {len(image_paths)} images")
    print(f"Pattern: {pw}x{ph}, Square size: {args.square_size}mm")
    print()
    
    calibrator = CameraCalibrator(pattern_size=(pw, ph), square_size_mm=args.square_size)
    
    for path in sorted(image_paths):
        img = cv2.imread(path)
        if img is None:
            print(f"Could not load: {path}")
            continue
        
        if calibrator.add_calibration_image(img):
            print(f"✓ {os.path.basename(path)}")
        else:
            print(f"✗ {os.path.basename(path)} (no corners found)")
    
    print()
    result = calibrator.calibrate()
    
    if result:
        result.save(args.output)
        print(f"\nCalibration complete! RMS error: {result.rms_error:.4f}")
    else:
        print("\nCalibration failed - not enough valid images")
