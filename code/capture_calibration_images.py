#!/usr/bin/env python3
"""
Calibration Image Capture — Smart Chess Board

Interactively captures checkerboard images for camera calibration.
Each press of Enter captures a frame. Press 'q' + Enter to quit.

Usage:
    python3 code/capture_calibration_images.py             # V4L2 /dev/video0
    python3 code/capture_calibration_images.py --device 1
    python3 code/capture_calibration_images.py --picam
    python3 code/capture_calibration_images.py --out /tmp/chess_calib --count 20
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np


SAVE_DIR = "/tmp/chess_calib"
TARGET_COUNT = 20


def open_v4l2(device_id: int, width: int = 1280, height: int = 720):
    dev_path = f"/dev/video{device_id}"
    cap = cv2.VideoCapture(dev_path, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap = cv2.VideoCapture(dev_path)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {dev_path}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 5)
    time.sleep(1.0)
    # flush
    for _ in range(5):
        cap.read()
    return cap


def open_picam(width: int = 1280, height: int = 720):
    from picamera2 import Picamera2
    picam = Picamera2()
    cfg = picam.create_video_configuration(main={"size": (width, height)})
    picam.configure(cfg)
    picam.start()
    time.sleep(2.0)
    return picam


def read_frame(cap=None, picam=None):
    if picam:
        frame = picam.capture_array("main")
        if frame.shape[2] == 3:
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        return frame
    else:
        for _ in range(3):  # flush a few frames first
            cap.read()
        ret, frame = cap.read()
        return frame if ret else None


def check_corners(frame, pattern_size=(7, 7)):
    """Quick corner check - returns True if pattern found."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    return found, corners


def main():
    parser = argparse.ArgumentParser(description="Capture calibration images")
    parser.add_argument("--device", "-d", type=int, default=0, help="V4L2 device index")
    parser.add_argument("--picam", action="store_true", help="Use picamera2")
    parser.add_argument("--out", "-o", default=SAVE_DIR, help="Output directory")
    parser.add_argument("--count", "-n", type=int, default=TARGET_COUNT, help="Target image count")
    parser.add_argument("--pattern", "-p", default="7x7", help="Checkerboard internal corners (WxH)")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    try:
        pw, ph = map(int, args.pattern.split('x'))
    except ValueError:
        print(f"Invalid pattern: {args.pattern}")
        sys.exit(1)

    os.makedirs(args.out, exist_ok=True)

    print("=" * 60)
    print("CHECKERBOARD CALIBRATION IMAGE CAPTURE")
    print("=" * 60)
    print(f"Output dir  : {args.out}")
    print(f"Target count: {args.count}")
    print(f"Pattern     : {pw}x{ph} internal corners")
    print(f"Backend     : {'picamera2' if args.picam else f'V4L2 /dev/video{args.device}'}")
    print()
    print("Instructions:")
    print("  - Hold the checkerboard at different angles/positions")
    print("  - Press ENTER to capture a frame")
    print("  - Type 'q' and ENTER to quit early")
    print("=" * 60)
    print()

    # Open camera
    picam = None
    cap = None
    try:
        if args.picam:
            picam = open_picam(args.width, args.height)
            print("picamera2 ready")
        else:
            cap = open_v4l2(args.device, args.width, args.height)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"Camera ready: {actual_w}x{actual_h}")
    except Exception as e:
        print(f"ERROR opening camera: {e}")
        sys.exit(1)

    captured = 0
    skipped = 0

    while captured < args.count:
        remaining = args.count - captured
        prompt = f"\n[{captured}/{args.count}] Press ENTER to capture (or 'q'+ENTER to quit): "
        
        try:
            user_input = input(prompt).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\nInterrupted.")
            break

        if user_input == 'q':
            print("Quitting early.")
            break

        print("  Capturing...", end=" ", flush=True)
        frame = read_frame(cap=cap, picam=picam)

        if frame is None:
            print("FAILED — could not read frame")
            continue

        # Check for corners
        found, corners = check_corners(frame, pattern_size=(pw, ph))
        
        # Draw corners on a copy for feedback
        preview = frame.copy()
        if found:
            cv2.drawChessboardCorners(preview, (pw, ph), corners, found)
            status_text = f"CORNERS FOUND ({pw}x{ph})"
            color = (0, 255, 0)
        else:
            status_text = "NO CORNERS DETECTED"
            color = (0, 0, 255)

        cv2.putText(preview, status_text, (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)
        cv2.putText(preview, f"Image {captured+1}/{args.count}",
                    (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        # Save the raw frame (not annotated) for calibration
        filename = f"calib_{captured+1:03d}.jpg"
        filepath = os.path.join(args.out, filename)
        cv2.imwrite(filepath, frame)

        # Also save annotated preview
        preview_path = os.path.join(args.out, f"preview_{captured+1:03d}.jpg")
        cv2.imwrite(preview_path, preview)

        if found:
            print(f"✓ Saved: {filename} (corners found)")
            captured += 1
        else:
            print(f"⚠ Saved: {filename} (WARNING: no corners detected - check board visibility)")
            print("  This image will still be saved but may fail calibration.")
            print("  Recommended: re-capture this one.")
            captured += 1  # still count it, calibrate_camera.py will filter bad ones

    print()
    print("=" * 60)
    print(f"CAPTURE COMPLETE: {captured} images saved to {args.out}")
    print()
    if captured >= 3:
        print("Next step - run calibration:")
        print(f"  python3 src/chess_perception/scripts/calibrate_camera.py \\")
        print(f"      --images {args.out} \\")
        print(f"      --output src/chess_perception/chess_perception/config/calibration.npz \\")
        print(f"      --pattern {args.pattern}")
    else:
        print("WARNING: Need at least 3 images for calibration.")
    print("=" * 60)

    if picam:
        picam.stop()
        picam.close()
    elif cap:
        cap.release()


if __name__ == "__main__":
    main()
