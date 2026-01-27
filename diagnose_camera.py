#!/usr/bin/env python3
"""
Camera Diagnostic Helper.
Run this to identify why camera tests are failing.
"""

import os
import sys
import subprocess

def check_command(cmd):
    try:
        subprocess.check_call(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except:
        return False

print("="*60)
print("CAMERA DIAGNOSTIC TOOL")
print("="*60)

# 1. Check for devices
print("\n1. Checking /dev/video* devices...")
devices = [f for f in os.listdir("/dev") if f.startswith("video")]
if devices:
    print(f"   FOUND: {', '.join(devices)}")
else:
    print("   [!] NO VIDEO DEVICES FOUND in /dev")

# 2. Check libcamera
print("\n2. Checking libcamera support...")
if check_command("libcamera-hello --list-cameras"):
    print("   [OK] libcamera detects a camera")
else:
    print("   [!] libcamera did NOT detect any cameras (normal for USB webcams)")

# 3. Check fswebcam
print("\n3. Checking fswebcam (for USB)...")
if check_command("which fswebcam"):
    print("   [OK] fswebcam is installed")
else:
    print("   [!] fswebcam NOT installed. Install with: sudo apt install fswebcam")

# 4. Check OpenCV
print("\n4. Checking OpenCV...")
try:
    import cv2
    print(f"   [OK] OpenCV {cv2.__version__} imported successfully")
except ImportError:
    print("   [!] OpenCV NOT installed. Install with: pip3 install opencv-python")

print("\n" + "="*60)
print("DIAGNOSIS:")

if not devices and not check_command("libcamera-hello --list-cameras"):
    print("CRITICAL: No camera detected at all.")
    print(" - Check ribbon cable (CSI) or USB cable.")
    print(" - For CSI: Run 'vcgencmd get_camera'")
elif devices and not check_command("libcamera-hello --list-cameras"):
    print("Legacy/USB Camera detected.")
    print(" - The test suite might be failing because it relies on libcamera.")
    print(" - I will patch the test suite to support USB cameras better.")
else:
    print("Camera detected. If tests fail, it might be a permissions or timeout issue.")
    print(" - Try increasing timeouts in test_camera.py")
