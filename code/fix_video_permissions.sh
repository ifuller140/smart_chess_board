#!/bin/bash
# fix_video_permissions.sh — run on Pi to make video device permanent
# Run this manually (not from a script) so sudo can prompt for your password interactively.

# Add ian to video group (needs re-login to take effect, but sudo sets it now)
sudo usermod -aG video ian

# Create udev rule
sudo bash -c 'cat > /etc/udev/rules.d/99-chess-camera.rules << UDEV
KERNEL=="video0", GROUP="video", MODE="0666"
KERNEL=="video[0-9]*", GROUP="video", MODE="0660"
UDEV'

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo chmod a+rw /dev/video0 /dev/video1 2>/dev/null

echo "Groups: $(groups ian)"
ls -la /dev/video0
echo "DONE"
