#!/bin/bash
# install_libcamera_python.sh
# Adds the Raspberry Pi foundation repo and installs python3-libcamera

set -e
echo "=== Adding Raspberry Pi apt repository ==="
echo "fuller" | sudo -S bash -c 'cat > /etc/apt/sources.list.d/raspi.list << EOF
deb http://archive.raspberrypi.com/debian/ jammy main
EOF'

echo "=== Adding RPi GPG key ==="
echo "fuller" | sudo -S bash -c '
curl -fsSL https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor -o /etc/apt/trusted.gpg.d/raspberrypi.gpg 2>/dev/null || \
wget -qO- https://archive.raspberrypi.com/debian/raspberrypi.gpg.key | gpg --dearmor | tee /etc/apt/trusted.gpg.d/raspberrypi.gpg > /dev/null
'

echo "=== Updating apt ==="
echo "fuller" | sudo -S apt-get update 2>&1 | tail -5

echo "=== Installing python3-libcamera and python3-picamera2 ==="
echo "fuller" | sudo -S apt-get install -y python3-libcamera python3-picamera2 python3-kms++ 2>&1 | tail -20

echo "=== Testing ==="
python3 -c "from picamera2 import Picamera2; print('picamera2 OK'); p = Picamera2(); print('Camera created:', p); p.close()" 2>&1

echo "DONE"
