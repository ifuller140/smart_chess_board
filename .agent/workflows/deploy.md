---
description: How to deploy the system to Raspberry Pi
---

# Deployment Workflow

Steps for deploying the Smart Chess Board software to a Raspberry Pi.

## Prerequisites

> **IMPORTANT**: This project requires real Raspberry Pi hardware with GPIO. The code will NOT run on development machines without RPi.GPIO.

- [ ] Raspberry Pi 4B with Ubuntu 22.04 or Raspberry Pi OS
- [ ] Network connectivity (SSH access)
- [ ] ROS 2 Humble installed on Pi
- [ ] All physical hardware components connected (steppers, servos, limit switches, camera)

## 1. Install ROS 2 Humble

```bash
# On Raspberry Pi
# Follow official docs: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Quick version:
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

## 2. Install System Dependencies

```bash
sudo apt install -y \
    python3-pip \
    python3-opencv \
    libcamera-apps \
    stockfish

pip3 install \
    RPi.GPIO \
    python-chess \
    opencv-python \
    numpy
```

## 3. Clone Repository

```bash
mkdir -p ~/smart_chess_ws/src
cd ~/smart_chess_ws/src
git clone https://github.com/ifuller140/smart_chess_board.git
# Or use scp/rsync to copy files
```

## 4. Build Workspace

```bash
cd ~/smart_chess_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## 5. Configure GPIO

```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Logout and login again for group changes
```

## 6. Update Configuration

```bash
# Edit pin configuration
nano ~/smart_chess_ws/src/smart_chess_board/src/chess_hw_interface/config/pins.yaml
```

Update pin numbers to match your wiring.

## 7. Test Hardware Components

```bash
# Follow hardware-test.md workflow
```

## 8. Run System

```bash
cd ~/smart_chess_ws
source install/setup.bash
ros2 launch src/smart_chess_board/src/launch/full_system_launch.py
```

## Auto-Start on Boot (Optional)

```bash
# Create systemd service
sudo nano /etc/systemd/system/smart-chess.service
```

Content:
```ini
[Unit]
Description=Smart Chess Board
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/smart_chess_ws
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch src/smart_chess_board/src/launch/full_system_launch.py"
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

Enable:
```bash
sudo systemctl enable smart-chess.service
sudo systemctl start smart-chess.service
```

## Updating Deployment

```bash
cd ~/smart_chess_ws/src/smart_chess_board
git pull
cd ~/smart_chess_ws
colcon build
# Restart service or re-launch
```