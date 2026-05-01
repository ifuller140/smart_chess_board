#!/usr/bin/env python3
"""
Camera Calibration Script for Chess Vision.

Run this to calibrate your camera using the chess board.
Uses images captured from the test suite or custom images.

Usage:
    # Using images from test capture:
    python3 scripts/calibrate_camera.py --images /tmp/chess_camera_test/calibration

    # Custom images:
    python3 scripts/calibrate_camera.py --images ./my_calibration_images

    # Specify output location:
    python3 scripts/calibrate_camera.py --images /tmp/... --output config/calibration.npz
"""

import sys
import os
import argparse
import glob

# Add parent to path for imports
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import cv2
from chess_perception.calibration import CameraCalibrator, calibrate_from_images


def main():
    parser = argparse.ArgumentParser(
        description="Calibrate camera for chess vision",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Calibrate from test images:
  python3 calibrate_camera.py -i /tmp/chess_camera_test/calibration

  # Save to specific location:
  python3 calibrate_camera.py -i ./images -o ./config/calibration.npz

  # Use custom pattern (not chess board):
  python3 calibrate_camera.py -i ./images --pattern 9x6 --square-size 25.0
        """
    )
    
    parser.add_argument("--images", "-i", required=True,
                        help="Directory with calibration images")
    parser.add_argument("--output", "-o", 
                        default="chess_perception/config/calibration.npz",
                        help="Output file path")
    parser.add_argument("--pattern", "-p", default="7x7",
                        help="Checkerboard pattern (internal corners). Default: 7x7 for chess board")
    parser.add_argument("--square-size", "-s", type=float, default=38.1,
                        help="Square size in mm. Default: 38.1 (1.5 inches)")
    parser.add_argument("--preview", action="store_true",
                        help="Show preview of detected corners")
    
    args = parser.parse_args()
    
    # Parse pattern
    try:
        pw, ph = map(int, args.pattern.split('x'))
    except ValueError:
        print(f"Invalid pattern format: {args.pattern}")
        print("Use format like '7x7' or '9x6'")
        return 1
    
    # Find images
    if os.path.isdir(args.images):
        patterns = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.JPG', '*.PNG']
        image_paths = []
        for p in patterns:
            image_paths.extend(glob.glob(os.path.join(args.images, p)))
    else:
        image_paths = glob.glob(args.images)
    
    if not image_paths:
        print(f"No images found in: {args.images}")
        return 1
    
    image_paths = sorted(image_paths)
    
    print("=" * 60)
    print("CAMERA CALIBRATION")
    print("=" * 60)
    print(f"Images: {len(image_paths)} files")
    print(f"Pattern: {pw}x{ph} internal corners")
    print(f"Square size: {args.square_size} mm")
    print(f"Output: {args.output}")
    print("=" * 60)
    print()
    
    # Initialize calibrator
    calibrator = CameraCalibrator(pattern_size=(pw, ph), square_size_mm=args.square_size)
    
    successful = 0
    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            print(f"✗ Could not load: {os.path.basename(path)}")
            continue
        
        found, corners = calibrator.find_corners(img)
        
        if found:
            calibrator.obj_points.append(calibrator.objp)
            calibrator.img_points.append(corners)
            successful += 1
            print(f"✓ {os.path.basename(path)}")
            
            if args.preview:
                preview = img.copy()
                cv2.drawChessboardCorners(preview, (pw, ph), corners, found)
                cv2.imshow("Corners", cv2.resize(preview, (800, 600)))
                key = cv2.waitKey(500)
                if key == 27:  # ESC
                    cv2.destroyAllWindows()
                    args.preview = False
        else:
            print(f"✗ {os.path.basename(path)} (no corners found)")
    
    if args.preview:
        cv2.destroyAllWindows()
    
    print()
    print(f"Found corners in {successful}/{len(image_paths)} images")
    
    if successful < 3:
        print("\nNeed at least 3 valid images for calibration")
        return 1
    
    # Run calibration
    print("\nRunning calibration...")
    result = calibrator.calibrate()
    
    if result is None:
        print("Calibration failed")
        return 1
    
    # Ensure output directory exists
    output_dir = os.path.dirname(args.output)
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    
    # Save
    result.save(args.output)
    
    print()
    print("=" * 60)
    print("CALIBRATION COMPLETE")
    print("=" * 60)
    print(f"RMS Error: {result.rms_error:.4f}")
    print(f"Saved to: {args.output}")
    print()
    print("The calibration will be automatically loaded by the vision pipeline.")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
