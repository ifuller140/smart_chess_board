#!/bin/bash
# install_libcamera_python.sh
# Installs python3-libcamera and python3-picamera2 on Ubuntu 22.04 (Jammy) for Raspberry Pi.
# Run on the Pi as: sudo bash code/install_libcamera_python.sh
#
# These packages enable the Pi Camera Module v2 CSI interface via libcamera.
# Without them, picamera2 fails with "No module named 'libcamera'".

set -e

if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: This script must be run as root."
    echo "  Usage: sudo bash code/install_libcamera_python.sh"
    exit 1
fi

CODENAME=$(. /etc/os-release && echo "$UBUNTU_CODENAME" 2>/dev/null || echo "jammy")
echo "=== Detected Ubuntu codename: $CODENAME ==="

# 1. Check if python3-libcamera is already in the standard repos
echo ""
echo "=== Checking standard Ubuntu repos ==="
apt-get update -qq

if apt-cache show python3-libcamera &>/dev/null; then
    echo "python3-libcamera found in standard repos — installing..."
    apt-get install -y python3-libcamera python3-picamera2 python3-kms++
else
    echo "python3-libcamera not in standard repos — adding Raspberry Pi Foundation repo..."

    # 2. Add RPi Foundation apt repository (provides python3-libcamera for Ubuntu 22.04)
    RPi_REPO_FILE="/etc/apt/sources.list.d/raspi.list"
    RPi_GPG="/etc/apt/trusted.gpg.d/raspberrypi.gpg"

    cat > "$RPi_REPO_FILE" << EOF
deb http://archive.raspberrypi.com/debian/ jammy main
EOF
    echo "  Wrote $RPi_REPO_FILE"

    # Add GPG key
    if command -v curl &>/dev/null; then
        curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
            | gpg --dearmor -o "$RPi_GPG"
    else
        wget -qO- https://archive.raspberrypi.com/debian/raspberrypi.gpg.key \
            | gpg --dearmor | tee "$RPi_GPG" > /dev/null
    fi
    echo "  Wrote $RPi_GPG"

    apt-get update -qq
    apt-get install -y python3-libcamera python3-picamera2 python3-kms++
fi

echo ""
echo "=== Installation complete — testing ==="
python3 -c "
import libcamera
print('libcamera OK:', libcamera.__version__ if hasattr(libcamera, '__version__') else 'imported')
" || echo "WARN: libcamera import test failed"

python3 -c "
from picamera2 import Picamera2
print('picamera2 OK')
p = Picamera2()
cfg = p.create_video_configuration(main={'size': (640, 480)})
p.configure(cfg)
p.start()
import time; time.sleep(0.5)
frame = p.capture_array('main')
p.stop(); p.close()
print(f'Camera frame shape: {frame.shape}, mean: {frame.mean():.1f}')
print('SUCCESS — camera is working!')
" || echo "WARN: picamera2 test failed (check camera connection and /boot/firmware/config.txt)"

echo ""
echo "=== DONE ==="
echo "Now rebuild and restart the ROS node:"
echo "  cd ~/dev/smart_chess_board"
echo "  source /opt/ros/humble/setup.bash"
echo "  colcon build --symlink-install --packages-select chess_perception"
echo "  source install/setup.bash"
echo "  ros2 launch chess_perception perception_test_launch.py use_picamera2:=True"
