---
description: How to deploy the system to Raspberry Pi
---

# Deployment Workflow

Steps for deploying the Smart Chess Board software to a Raspberry Pi.

## Prerequisites

> **IMPORTANT**: This project requires real Raspberry Pi hardware with GPIO. The code will NOT run on development machines without RPi.GPIO.

- [ ] Raspberry Pi 4B with Ubuntu 24.04 (or Raspberry Pi OS)
- [ ] Network connectivity (SSH access via `ssh smart-chess-pi`)
- [ ] ROS 2 Jazzy installed on Pi
- [ ] pigpiod daemon running
- [ ] All physical hardware components connected (steppers, servos, limit switches, camera)

## SSH Access

The Pi is configured in `~/.ssh/config` as `smart-chess-pi` (192.168.1.150):

```bash
ssh smart-chess-pi
```

## 1. Install ROS 2 Jazzy

```bash
# On Raspberry Pi
# Follow official docs: https://docs.ros.org/en/jazzy/Installation.html

# Quick version:
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions
```

## 2. Install System Dependencies

```bash
sudo apt install -y \
    python3-pip \
    python3-opencv \
    libcamera-apps \
    stockfish \
    pigpio

pip3 install \
    RPi.GPIO \
    pigpio \
    python-chess \
    opencv-python \
    numpy
```

### Enable pigpiod daemon (required for motor control)

```bash
# Enable pigpiod to start automatically on boot
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

## 3. Clone Repository

```bash
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/ifuller140/smart_chess_board.git
```

## 4. Build Workspace

```bash
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **NOTE**: The `--symlink-install` flag creates symlinks instead of copying files,
> so you can edit source files without rebuilding (only need to rebuild if
> you change setup.py, package.xml, or add/remove files).

### Clean Rebuild (if you get errors)

```bash
cd ~/dev/smart_chess_board
rm -rf build/ install/ log/
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Package Build Order

colcon handles dependency ordering automatically. The packages are:

1. `gantry_control_interfaces` — CMake package for MoveGantry.action (built first)
2. `chess_hw_interface` — stepper, servo, limit switch nodes + hardware tests
3. `chess_perception` — camera and board detection
4. `gantry_control` — kinematics, motion planning, homing (depends on 1 and 2)
5. `chess_logic` — chess engine and game manager

## 5. Configure GPIO

```bash
# Add user to gpio group
sudo usermod -a -G gpio $USER

# Logout and login again for group changes
```

## 6. Update Configuration

```bash
# Edit pin configuration
nano ~/dev/smart_chess_board/src/chess_hw_interface/config/pins.yaml
```

Update pin numbers to match your wiring.

## 7. Test Hardware Components

```bash
# Source ROS first
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Launch hardware interface nodes (required for gantry tests)
ros2 launch chess_hw_interface hw_interface_launch.py

# In another terminal — run tests
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 -m chess_hw_interface.testing.test_runner --list
python3 -m chess_hw_interface.testing.test_runner --category gantry --subtest manual
```

## 8. Run System

```bash
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Hardware interface only:
ros2 launch chess_hw_interface hw_interface_launch.py

# Full system:
ros2 launch src/launch/full_system_launch.py
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
After=network.target pigpiod.service
Requires=pigpiod.service

[Service]
Type=simple
User=ian
WorkingDirectory=/home/ian/dev/smart_chess_board
ExecStart=/bin/bash -c "source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch chess_hw_interface hw_interface_launch.py"
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
cd ~/dev/smart_chess_board
git pull
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
# Restart service or re-launch
```