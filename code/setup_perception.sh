#!/bin/bash
# setup_perception.sh - Run on the Pi to set up directories and verify camera

set -e

echo "=== Setting up Smart Chess Perception ==="

# Ensure directories
mkdir -p ~/dev/smart_chess_board/code
mkdir -p ~/dev/smart_chess_board/src/chess_perception/chess_perception/config

# Copy from upload area
UPLOAD=~/dev/smart_chess_board/src/chess_perception_upload

if [ -d "$UPLOAD" ]; then
    cp "$UPLOAD/live_camera_preview.py" ~/dev/smart_chess_board/code/ 2>/dev/null && echo "✓ live_camera_preview.py" || echo "✗ live_camera_preview.py"
    cp "$UPLOAD/capture_calibration_images.py" ~/dev/smart_chess_board/code/ 2>/dev/null && echo "✓ capture_calibration_images.py" || echo "✗ capture_calibration_images.py"
    cp -r "$UPLOAD/chess_perception/." ~/dev/smart_chess_board/src/chess_perception/chess_perception/ && echo "✓ chess_perception package" || echo "✗ chess_perception package"
    cp -r "$UPLOAD/scripts/." ~/dev/smart_chess_board/src/chess_perception/scripts/ && echo "✓ scripts" || echo "✗ scripts"
else
    echo "WARN: Upload dir $UPLOAD not found - files may already be in place"
fi

echo ""
echo "=== Installed Files ==="
ls ~/dev/smart_chess_board/code/ 2>/dev/null
ls ~/dev/smart_chess_board/src/chess_perception/scripts/ 2>/dev/null

echo ""
echo "=== Camera Devices ==="
for dev in /dev/video0 /dev/video1; do
    if [ -e "$dev" ]; then
        echo "$dev: exists"
        v4l2-ctl --device=$dev --info 2>/dev/null | grep -E "Driver|Card|Bus" || true
    fi
done

echo ""
echo "=== Python Packages ==="
python3 -c "import cv2; print('cv2:', cv2.__version__)" 2>/dev/null || echo "cv2: NOT FOUND"
python3 -c "import numpy; print('numpy:', numpy.__version__)" 2>/dev/null || echo "numpy: NOT FOUND"
python3 -c "import picamera2; print('picamera2:', picamera2.__version__)" 2>/dev/null || echo "picamera2: NOT INSTALLED (will use V4L2)"

echo ""
echo "SETUP_DONE"
