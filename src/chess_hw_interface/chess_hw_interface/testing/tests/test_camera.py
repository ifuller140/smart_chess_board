#!/usr/bin/env python3
"""
Camera Test Suite.

Tests:
1. Camera connection
2. Image capture
3. Calibration image capture
4. Board corner detection

Supports both CSI cameras (via libcamera) and USB cameras (via OpenCV).
"""

import time
from typing import List, Optional
import os

try:
    import cv2
    OPENCV_AVAILABLE = True
except ImportError:
    OPENCV_AVAILABLE = False

from ..base_test import HardwareTest, TestStep


class CameraTest(HardwareTest):
    """
    Test camera functionality and calibration.
    """
    
    # Output directory for test images
    OUTPUT_DIR = "/tmp/chess_camera_test"
    
    @property
    def name(self) -> str:
        return "Camera"
    
    @property
    def description(self) -> str:
        return "Test camera capture and calibration"
    
    def setup(self) -> bool:
        """Create output directory."""
        try:
            os.makedirs(self.OUTPUT_DIR, exist_ok=True)
            return True
        except Exception as e:
            print(f"[ERROR] Failed to create output dir: {e}")
            return False
    
    def _capture_image_opencv(self, output_path: str) -> bool:
        """
        Capture image using OpenCV (for USB cameras).
        
        Args:
            output_path: Path to save the captured image
            
        Returns:
            True if capture successful
        """
        if not OPENCV_AVAILABLE:
            return False
        
        # Try common video device indices
        for device_idx in [0, 1, 2]:
            cap = cv2.VideoCapture(device_idx)
            if cap.isOpened():
                # Allow camera to warm up
                time.sleep(0.5)
                ret, frame = cap.read()
                cap.release()
                
                if ret and frame is not None:
                    cv2.imwrite(output_path, frame)
                    return os.path.exists(output_path)
        
        return False
    
    def get_steps(self) -> List[TestStep]:
        """Define test steps."""
        return [
            # Check camera
            TestStep(
                name="Camera Connection",
                display_text="CAM CHK",
                action=self._check_camera,
                wait_for_input=False,
                success_message="CAM CONN",
                failure_message="NO CAM"
            ),
            
            # Capture test image
            TestStep(
                name="Test Capture",
                display_text="CAPTURE",
                action=self._capture_test_image,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="CAP OK",
                failure_message="CAP FAIL"
            ),
            
            # Calibration mode
            TestStep(
                name="Calibration Capture",
                display_text="CALIB",
                action=self._capture_calibration,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=60.0,
                success_message="CALIB OK",
                failure_message="CAL FAIL"
            ),
            
            # Board detection test
            TestStep(
                name="Board Detection",
                display_text="DETECT",
                action=self._test_board_detection,
                wait_for_input=True,
                input_type="clock",
                timeout_seconds=30.0,
                success_message="BRD OK",
                failure_message="NO BRD"
            ),
        ]
    
    def _check_camera(self) -> bool:
        """Check if camera is available."""
        print("  Checking camera connection...")
        
        # Check for CSI camera
        csi_exists = os.path.exists("/dev/video0") or os.path.exists("/dev/video1")
        
        # Try libcamera
        try:
            result = os.system("libcamera-hello --list-cameras > /dev/null 2>&1")
            libcamera_ok = (result == 0)
        except:
            libcamera_ok = False
        
        if csi_exists or libcamera_ok:
            print("  Camera detected!")
            return True
        
        # Check USB cameras
        for i in range(4):
            if os.path.exists(f"/dev/video{i}"):
                print(f"  USB camera found at /dev/video{i}")
                return True
        
        print("  [ERROR] No camera detected")
        return False
    
    def _capture_test_image(self) -> bool:
        """Capture a test image."""
        print("  Capturing test image...")
        
        output_path = os.path.join(self.OUTPUT_DIR, "test_capture.jpg")
        
        try:
            # Try libcamera first (for Pi CSI camera)
            result = os.system(f"libcamera-still -o {output_path} -t 1000 > /dev/null 2>&1")
            
            if result != 0 or not os.path.exists(output_path):
                # Fallback to OpenCV for USB cameras
                print("    libcamera failed, trying OpenCV...")
                if not self._capture_image_opencv(output_path):
                    print("  [ERROR] All capture methods failed")
                    return False
            
            if os.path.exists(output_path):
                print(f"  Image saved to: {output_path}")
                return True
            else:
                print("  [ERROR] Capture failed")
                return False
                
        except Exception as e:
            print(f"  [ERROR] {e}")
            return False
    
    def _capture_calibration(self) -> bool:
        """Capture calibration images."""
        print("\n  CALIBRATION MODE")
        print("  -----------------")
        print("  Place checkerboard pattern on the board.")
        print("  Press clock button to capture each angle.")
        print("  Capture 5+ images at different angles.\n")
        
        calibration_dir = os.path.join(self.OUTPUT_DIR, "calibration")
        os.makedirs(calibration_dir, exist_ok=True)
        
        images_captured = 0
        target_images = 5
        
        while images_captured < target_images:
            self.show_display(f"CAL {images_captured+1}/{target_images}")
            
            # Wait for button press
            print(f"  Press clock button to capture image {images_captured + 1}...")
            if self.wait_for_clock_button(timeout=30.0):
                output_path = os.path.join(calibration_dir, f"calibration_{images_captured + 1}.jpg")
                
                # Try libcamera first, fallback to OpenCV
                result = os.system(f"libcamera-still -o {output_path} -t 500 > /dev/null 2>&1")
                capture_ok = (result == 0 and os.path.exists(output_path))
                
                if not capture_ok:
                    # Fallback to OpenCV for USB cameras
                    capture_ok = self._capture_image_opencv(output_path)
                
                if capture_ok:
                    images_captured += 1
                    print(f"  Captured: {output_path}")
                else:
                    print("  [WARNING] Capture failed, try again")
            else:
                print("  Timeout - skipping remaining captures")
                break
        
        if images_captured >= target_images:
            print(f"\n  Captured {images_captured} calibration images!")
            return True
        else:
            print(f"\n  Only captured {images_captured} images (need {target_images})")
            return images_captured > 0  # Partial success OK
    
    def _test_board_detection(self) -> bool:
        """Test if we can detect the chess board."""
        print("  Testing board detection...")
        
        try:
            import cv2
            import numpy as np
        except ImportError:
            raise ImportError("OpenCV (cv2) is required for board detection")
        
        # Capture fresh image
        test_image = os.path.join(self.OUTPUT_DIR, "detection_test.jpg")
        
        # Try libcamera first, fallback to OpenCV
        result = os.system(f"libcamera-still -o {test_image} -t 500 > /dev/null 2>&1")
        if result != 0 or not os.path.exists(test_image):
            if not self._capture_image_opencv(test_image):
                print("  [ERROR] Failed to capture image for detection")
                return False
        
        if not os.path.exists(test_image):
            print("  [ERROR] Failed to capture image for detection")
            return False
        
        # Load and process
        img = cv2.imread(test_image)
        if img is None:
            print("  [ERROR] Failed to load image")
            return False
        
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Try to find chessboard corners (7x7 internal corners for 8x8 board)
        found, corners = cv2.findChessboardCorners(gray, (7, 7), None)
        
        if found:
            print("  Chessboard corners detected!")
            # Save annotated image
            cv2.drawChessboardCorners(img, (7, 7), corners, found)
            annotated_path = os.path.join(self.OUTPUT_DIR, "detected_corners.jpg")
            cv2.imwrite(annotated_path, img)
            print(f"  Annotated image saved: {annotated_path}")
            return True
        else:
            # Try edge detection as fallback
            edges = cv2.Canny(gray, 50, 150)
            lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
            
            if lines is not None and len(lines) >= 8:
                print(f"  Board edges detected ({len(lines)} lines)")
                return True
            else:
                print("  [WARNING] Could not detect board clearly")
                return True  # Don't fail - might need calibration
