#!/usr/bin/env python3
"""
Camera device probe - tests all video devices and reports which give real frames.
Also tries libcamera-based capture.
"""
import cv2
import numpy as np
import subprocess
import os
import sys
import time

print("=" * 60)
print("CAMERA DEVICE PROBE")
print("=" * 60)

# 1. Check libcamera
print("\n--- libcamera availability ---")
try:
    result = subprocess.run(["libcamera-hello", "--version"], capture_output=True, text=True, timeout=5)
    print(result.stdout.strip() or result.stderr.strip() or "no output")
except FileNotFoundError:
    print("  libcamera-hello: not found")
except Exception as e:
    print(f"  libcamera-hello error: {e}")

# 2. Try rpicam / libcamera capture
print("\n--- Testing libcamera-jpeg capture ---")
test_img = "/tmp/libcam_test.jpg"
for cmd_name in ["libcamera-jpeg", "rpicam-jpeg"]:
    try:
        result = subprocess.run(
            [cmd_name, "-o", test_img, "-t", "2000", "--nopreview"],
            capture_output=True, text=True, timeout=20
        )
        if os.path.exists(test_img):
            size = os.path.getsize(test_img)
            print(f"  {cmd_name}: SUCCESS - {size} bytes written to {test_img}")
            break
        else:
            print(f"  {cmd_name}: FAILED - {result.stderr[:200]}")
    except FileNotFoundError:
        print(f"  {cmd_name}: not found")
    except Exception as e:
        print(f"  {cmd_name}: error {e}")

# 3. Probe V4L2 devices
print("\n--- V4L2 device probe ---")
devices_to_try = [0, 1, 2, 10, 11, 12, 13, 14, 15, 16, 18, 20, 21, 22, 23, 31]
working_devices = []

for dev_id in devices_to_try:
    dev_path = f"/dev/video{dev_id}"
    if not os.path.exists(dev_path):
        continue
    
    # Use V4L2 first, then auto
    for backend_name, backend in [("V4L2", cv2.CAP_V4L2), ("AUTO", cv2.CAP_ANY)]:
        cap = cv2.VideoCapture(dev_id, backend)
        if not cap.isOpened():
            continue
        
        # Try to read a frame
        ret = False
        frame = None
        for _ in range(5):  # try a few times
            ret, frame = cap.read()
            if ret and frame is not None:
                break
            time.sleep(0.1)
        
        cap.release()
        
        if ret and frame is not None:
            nonblack = np.mean(frame) > 5  # More than 5/255 average = not black
            print(f"  video{dev_id} ({backend_name}): ✓ ret={ret} shape={frame.shape} mean={np.mean(frame):.1f} nonblack={nonblack}")
            if nonblack:
                working_devices.append((dev_id, backend_name))
            break
        else:
            print(f"  video{dev_id} ({backend_name}): frame read FAILED")
            break

print("\n--- v4l2-ctl info for promising devices ---")
for dev_id in [0, 1]:
    try:
        result = subprocess.run(
            ["v4l2-ctl", f"--device=/dev/video{dev_id}", "--all"],
            capture_output=True, text=True, timeout=5
        )
        if result.returncode == 0:
            for line in result.stdout.split('\n')[:20]:
                if line.strip():
                    print(f"  {line}")
        else:
            print(f"  video{dev_id}: {result.stderr[:100]}")
    except FileNotFoundError:
        print(f"  video{dev_id}: v4l2-ctl not found")
    except Exception as e:
        print(f"  video{dev_id}: {e}")

print()
print("=" * 60)
if working_devices:
    print(f"WORKING DEVICES WITH REAL FRAMES: {working_devices}")
else:
    print("WARNING: No V4L2 device returned non-black frames")
    if os.path.exists("/tmp/libcam_test.jpg"):
        print("  → libcamera capture worked — use picamera2 instead of V4L2")
    else:
        print("  → Check camera connection and driver")
print("=" * 60)
