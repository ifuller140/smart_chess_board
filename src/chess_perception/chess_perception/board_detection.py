#!/usr/bin/env python3
"""
Board Detection Module for Chess Vision.

Detects chess board using line detection and computes homography
to transform the perspective view to an orthographic top-down view.

Physical Setup:
- Board: 12x12 inches (8x8 squares, 1.5 inches each)
- Output: 480x480 orthographic image (60 pixels per square)
"""

import numpy as np
import cv2
from typing import Optional, Tuple, List
from dataclasses import dataclass


@dataclass
class BoardGeometry:
    """Detected board corners and homography matrix."""
    corners: np.ndarray  # 4x2 array of corner points [TL, TR, BR, BL]
    homography: np.ndarray  # 3x3 homography matrix
    inverse_homography: np.ndarray  # For mapping back to original
    output_size: Tuple[int, int]  # (width, height) of warped output
    
    def warp_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """Transform a point from original image to warped space."""
        pt = np.array([[point]], dtype=np.float32)
        warped = cv2.perspectiveTransform(pt, self.homography)
        return tuple(warped[0, 0])
    
    def unwarp_point(self, point: Tuple[float, float]) -> Tuple[float, float]:
        """Transform a point from warped space back to original image."""
        pt = np.array([[point]], dtype=np.float32)
        unwarped = cv2.perspectiveTransform(pt, self.inverse_homography)
        return tuple(unwarped[0, 0])


class BoardDetector:
    """
    Detects chess board and computes perspective transform.
    
    Uses multiple strategies:
    1. Line detection (Hough) - finds grid lines
    2. Corner detection - finds 4 board corners
    3. Contour detection - fallback using largest quadrilateral
    """
    
    # Output size: 480x480 = 60 pixels per square (8 squares)
    OUTPUT_SIZE = (480, 480)
    SQUARE_SIZE = 60  # pixels per square in output
    
    def __init__(self, output_size: Tuple[int, int] = None):
        """
        Initialize board detector.
        
        Args:
            output_size: (width, height) for warped output. Default (480, 480).
        """
        self.output_size = output_size or self.OUTPUT_SIZE
        
        # Destination points for homography (top-down view)
        w, h = self.output_size
        self.dst_points = np.array([
            [0, 0],       # Top-left
            [w - 1, 0],   # Top-right
            [w - 1, h - 1],  # Bottom-right
            [0, h - 1]    # Bottom-left
        ], dtype=np.float32)
    
    def detect(self, image: np.ndarray) -> Optional[BoardGeometry]:
        """
        Detect board in image.
        
        Args:
            image: BGR image
            
        Returns:
            BoardGeometry or None if detection failed
        """
        # Try line-based detection first
        corners = self._detect_by_lines(image)
        
        # Fallback to contour detection
        if corners is None:
            corners = self._detect_by_contour(image)
        
        if corners is None:
            return None
        
        # Compute homography
        H = cv2.getPerspectiveTransform(corners, self.dst_points)
        H_inv = cv2.getPerspectiveTransform(self.dst_points, corners)
        
        return BoardGeometry(
            corners=corners,
            homography=H,
            inverse_homography=H_inv,
            output_size=self.output_size
        )
    
    def warp_image(self, image: np.ndarray, geometry: BoardGeometry) -> np.ndarray:
        """
        Warp image to orthographic view using detected geometry.
        
        Args:
            image: Original image
            geometry: BoardGeometry from detect()
            
        Returns:
            Warped 480x480 image (or configured output_size)
        """
        return cv2.warpPerspective(image, geometry.homography, geometry.output_size)
    
    def _detect_by_lines(self, image: np.ndarray) -> Optional[np.ndarray]:
        """
        Detect board corners using Hough line detection.
        
        Finds horizontal and vertical lines, then computes intersections.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)
        
        # Detect lines
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold=100)
        
        if lines is None or len(lines) < 8:
            return None
        
        # Separate horizontal and vertical lines
        horizontal = []
        vertical = []
        
        for line in lines:
            rho, theta = line[0]
            
            # Horizontal: theta near 0 or pi
            if theta < np.pi / 6 or theta > 5 * np.pi / 6:
                vertical.append((rho, theta))
            # Vertical: theta near pi/2
            elif np.pi / 3 < theta < 2 * np.pi / 3:
                horizontal.append((rho, theta))
        
        if len(horizontal) < 2 or len(vertical) < 2:
            return None
        
        # Find outermost lines
        horizontal.sort(key=lambda x: x[0])
        vertical.sort(key=lambda x: x[0])
        
        # Get first and last of each
        h_lines = [horizontal[0], horizontal[-1]]
        v_lines = [vertical[0], vertical[-1]]
        
        # Compute intersections
        corners = []
        for h in h_lines:
            for v in v_lines:
                pt = self._line_intersection(h, v)
                if pt is not None:
                    corners.append(pt)
        
        if len(corners) != 4:
            return None
        
        # Order corners: TL, TR, BR, BL
        return self._order_corners(np.array(corners, dtype=np.float32))
    
    def _detect_by_contour(self, image: np.ndarray) -> Optional[np.ndarray]:
        """
        Detect board using contour detection (fallback method).
        
        Finds the largest quadrilateral contour.
        """
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 30, 100)
        
        # Dilate to connect edges
        kernel = np.ones((3, 3), np.uint8)
        edges = cv2.dilate(edges, kernel, iterations=2)
        
        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find largest quadrilateral
        best_contour = None
        max_area = 0
        
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 10000:  # Minimum area filter
                continue
            
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            
            if len(approx) == 4 and area > max_area:
                max_area = area
                best_contour = approx
        
        if best_contour is None:
            return None
        
        corners = best_contour.reshape(4, 2).astype(np.float32)
        return self._order_corners(corners)
    
    def _line_intersection(self, line1: Tuple[float, float], 
                           line2: Tuple[float, float]) -> Optional[Tuple[float, float]]:
        """
        Find intersection of two lines in Hough space (rho, theta).
        """
        rho1, theta1 = line1
        rho2, theta2 = line2
        
        A = np.array([
            [np.cos(theta1), np.sin(theta1)],
            [np.cos(theta2), np.sin(theta2)]
        ])
        b = np.array([rho1, rho2])
        
        try:
            x = np.linalg.solve(A, b)
            return tuple(x)
        except np.linalg.LinAlgError:
            return None
    
    def _order_corners(self, corners: np.ndarray) -> np.ndarray:
        """
        Order corners as: top-left, top-right, bottom-right, bottom-left.
        """
        rect = np.zeros((4, 2), dtype=np.float32)
        
        # Sum of coordinates: TL has smallest, BR has largest
        s = corners.sum(axis=1)
        rect[0] = corners[np.argmin(s)]  # TL
        rect[2] = corners[np.argmax(s)]  # BR
        
        # Difference: TR has smallest, BL has largest
        diff = np.diff(corners, axis=1).ravel()
        rect[1] = corners[np.argmin(diff)]  # TR
        rect[3] = corners[np.argmax(diff)]  # BL
        
        return rect
    
    def draw_debug(self, image: np.ndarray, geometry: Optional[BoardGeometry]) -> np.ndarray:
        """
        Draw debug visualization on image.
        
        Args:
            image: Original image
            geometry: Detection result (can be None)
            
        Returns:
            Annotated image copy
        """
        debug = image.copy()
        
        if geometry is None:
            cv2.putText(debug, "No board detected", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return debug
        
        # Draw corners
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # TL, TR, BR, BL
        labels = ['TL', 'TR', 'BR', 'BL']
        
        for i, (corner, color, label) in enumerate(zip(geometry.corners, colors, labels)):
            pt = tuple(corner.astype(int))
            cv2.circle(debug, pt, 10, color, -1)
            cv2.putText(debug, label, (pt[0] + 15, pt[1] + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Draw board outline
        pts = geometry.corners.astype(int).reshape((-1, 1, 2))
        cv2.polylines(debug, [pts], True, (0, 255, 0), 3)
        
        return debug


def get_square_center(row: int, col: int, square_size: int = 60) -> Tuple[int, int]:
    """
    Get center point of a square in the warped image.
    
    Args:
        row: Row index (0-7, 0 = top/rank 8)
        col: Column index (0-7, 0 = left/file a)
        square_size: Pixels per square (default 60)
        
    Returns:
        (x, y) center point
    """
    x = col * square_size + square_size // 2
    y = row * square_size + square_size // 2
    return (x, y)


def get_square_roi(warped_image: np.ndarray, row: int, col: int, 
                   square_size: int = 60, margin: int = 5) -> np.ndarray:
    """
    Extract a square's region of interest from warped image.
    
    Args:
        warped_image: Orthographic board image
        row: Row index (0-7)
        col: Column index (0-7)
        square_size: Pixels per square
        margin: Pixels to exclude from edges
        
    Returns:
        Cropped square image
    """
    x1 = col * square_size + margin
    x2 = (col + 1) * square_size - margin
    y1 = row * square_size + margin
    y2 = (row + 1) * square_size - margin
    
    return warped_image[y1:y2, x1:x2]


# CLI for standalone testing
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Board Detection Test")
    parser.add_argument("image", help="Path to test image")
    parser.add_argument("--output", "-o", help="Save output image")
    
    args = parser.parse_args()
    
    img = cv2.imread(args.image)
    if img is None:
        print(f"Could not load: {args.image}")
        exit(1)
    
    detector = BoardDetector()
    geometry = detector.detect(img)
    
    if geometry is None:
        print("Board detection failed")
        debug = detector.draw_debug(img, None)
    else:
        print("Board detected!")
        print(f"Corners:\n{geometry.corners}")
        
        debug = detector.draw_debug(img, geometry)
        warped = detector.warp_image(img, geometry)
        
        # Draw grid on warped
        for i in range(1, 8):
            cv2.line(warped, (i * 60, 0), (i * 60, 480), (0, 255, 0), 1)
            cv2.line(warped, (0, i * 60), (480, i * 60), (0, 255, 0), 1)
        
        # Show side by side
        h1, w1 = debug.shape[:2]
        h2, w2 = warped.shape[:2]
        
        # Resize warped to match height
        scale = h1 / h2
        warped_resized = cv2.resize(warped, (int(w2 * scale), h1))
        
        combined = np.hstack([debug, warped_resized])
        
        cv2.imshow("Board Detection", combined)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        if args.output:
            cv2.imwrite(args.output, combined)
            print(f"Saved: {args.output}")
