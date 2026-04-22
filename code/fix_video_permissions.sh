#!/bin/bash
# fix_video_permissions.sh — run on Pi to make video device permanent

# Add ian to video group (needs re-login to take effect, but sudo sets it now)
echo fuller | sudo -S usermod -aG video ian 2>/dev/null

# Create udev rule
echo fuller | sudo -S bash -c 'cat > /etc/udev/rules.d/99-chess-camera.rules << UDEV
KERNEL=="video0", GROUP="video", MODE="0666"
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
UDEV'

echo fuller | sudo -S udevadm control --reload-rules
echo fuller | sudo -S udevadm trigger
echo fuller | sudo -S chmod a+rw /dev/video0 /dev/video1 2>/dev/null

echo "Groups: $(groups ian)"
ls -la /dev/video0
echo "DONE"
