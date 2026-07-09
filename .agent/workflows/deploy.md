---
description: How to deploy the system to Raspberry Pi
---

# Deployment Workflow

Steps for deploying the Smart Chess Board software to a Raspberry Pi.

> **IMPORTANT**: This project requires Raspberry Pi hardware with GPIO. Code will NOT run on development machines without RPi.GPIO.

---

## Quick Reference

| Sub-deployment | What it runs | Web port(s) |
|---------------|-------------|-------------|
| [Hardware Only](#a-hardware-only) | Motors, servos, limit switches | None |
| [Perception Only](#b-perception-only) | Camera + board detection | `5000` (FEN visualizer), `8080` (camera stream) |
| [Full System](#c-full-system) | All nodes + game loop | `5000`, `8080` |

---

## One-Time Pi Setup

### SSH Access

The Pi is configured in `~/.ssh/config` as `smart-chess-pi` (192.168.1.149):

```bash
ssh smart-chess-pi
```

### 1. Install ROS 2 Humble

```bash
# On Raspberry Pi
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

### 2. Install System Dependencies

```bash
sudo apt install -y \
    python3-pip \
    python3-opencv \
    python3-picamera2 \
    libcamera-apps \
    libcamera-ipa \
    stockfish \
    pigpio

pip3 install \
    RPi.GPIO \
    pigpio \
    python-chess \
    opencv-python \
    numpy \
    flask
```

### 3. Enable pigpiod (required for motor control)

```bash
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```

### 4. GPIO permissions

```bash
sudo usermod -a -G gpio $USER
# Logout and login again for group changes
```

### 5. Install sudoers rule (hardware tests need /dev/mem)

```bash
sudo cp ~/dev/smart_chess_board/setup/smart-chess-hw-tests.sudoers \
    /etc/sudoers.d/smart-chess-hw-tests
sudo chmod 440 /etc/sudoers.d/smart-chess-hw-tests
```

### 6. Clone Repository

```bash
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/ifuller140/smart_chess_board.git
```

### 7. Build Workspace

```bash
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `--symlink-install` lets you edit source files without rebuilding. Only rebuild after changing `setup.py`, `package.xml`, or adding/removing files.

**Clean rebuild (if errors):**

```bash
cd ~/dev/smart_chess_board
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**Package build order** (colcon handles this automatically):

1. `chess_interfaces` — CMake: `MoveGantry.action`, `RequestMove.srv`, `BoardState.msg`
2. `chess_hw_interface` — Stepper, servo, limit switch + test runner
3. `chess_perception` — Camera and board detection
4. `gantry_control` — Kinematics and motion planning
5. `chess_logic` — Chess engine and game manager

### 8. Update Pin Configuration

```bash
nano ~/dev/smart_chess_board/src/chess_hw_interface/config/pins.yaml
```

Verify GPIO BCM pin numbers match your physical wiring. See `docs/hardware/pinout.md`.

---

## Sub-Deployments

### A. Hardware Only

Runs the GPIO drivers (stepper motors, servos, limit switches). No vision or game logic.

```bash
ssh smart-chess-pi
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch chess_hw_interface hw_interface_launch.py
```

**Verify hardware is responding:**

```bash
# In a second terminal on the Pi
cd ~/dev/smart_chess_board && source install/setup.bash

# Run hardware tests
./run_hw_test.sh --category gantry --subtest limits
./run_hw_test.sh --category gantry --subtest manual    # interactive arrow-key control
./run_hw_test.sh --category gantry --subtest full
```

See `.agent/workflows/hardware-test.md` for the full test suite reference.

---

### B. Perception Only

Runs the camera + board detection pipeline and exposes two browser-accessible interfaces.

**Open 3 SSH terminals to the Pi:**

**Terminal 1 — Launch perception stack:**

```bash
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch chess_perception perception_launch.py use_picamera2:=True
```

Wait for: `Camera node ready (backend=picamera2, 1280x720 @ 5.0fps)`

**Terminal 2 — Start FEN visualizer (port 5000):**

```bash
cd ~/dev/smart_chess_board
source install/setup.bash
python3 src/chess_perception/scripts/fen_visualizer.py
```

Output:
```
✓ ROS subscriber started
  Network URL: http://192.168.1.149:5000
```

Open `http://192.168.1.149:5000` on your Mac to see the live board state.

**Terminal 3 (optional) — Start raw camera stream (port 8080):**

```bash
python3 ~/dev/smart_chess_board/code/camera_stream_server.py
```

Open `http://smart-chess-pi:8080/` to see the raw MJPEG camera feed.

**Calibration:**

Before a game, capture the current board as the pre-move reference (game_manager_node also does this automatically each turn):

```bash
ros2 service call /perception/capture_premove std_srvs/srv/Trigger {}
```

See `docs/features/vision-system.md` for the full calibration procedure.

---

### C. Full System

Runs all nodes: hardware interface, perception, gantry control, and game logic.

```bash
ssh smart-chess-pi
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch src/launch/full_system_launch.py
```

In a separate terminal, optionally start the FEN visualizer to monitor board state:

```bash
cd ~/dev/smart_chess_board && source install/setup.bash
python3 src/chess_perception/scripts/fen_visualizer.py
```

---

## Updating the Deployment

```bash
ssh smart-chess-pi
cd ~/dev/smart_chess_board
git pull
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
# Restart whichever launch / service is running
```

---

## Auto-Start on Boot (Optional)

A ready-to-install unit file is tracked at `setup/smart-chess.service` (same
convention as `setup/smart-chess-hw-tests.sudoers` — a git-tracked template,
never installed automatically):

```bash
sudo cp setup/smart-chess.service /etc/systemd/system/smart-chess.service
sudo systemctl daemon-reload
sudo systemctl enable smart-chess.service
sudo systemctl start smart-chess.service
```

Check status/logs:

```bash
systemctl status smart-chess.service
journalctl -u smart-chess.service -f
```

If it doesn't come up after a boot, check for `start-limit-hit` in
`systemctl status` — the unit caps automatic restarts (5 within 60s) rather
than crash-looping forever if something is persistently broken (e.g.
`pigpiod` never coming up).

For finer-grained recovery of just the perception+UI layer (camera_node,
board_detector_node, piece_detector_node, chess_ui) without restarting the
whole stack — e.g. to mitigate the documented `camera_ros` stale-subscriber-
after-~2h bug — pass `respawn:=True` to `full_system_launch.py` (off by
default for interactive/manual launches). This is deliberately **not**
applied to hardware/gantry nodes: respawning those individually without a
full re-home risks operating on stale position assumptions after a crash
mid-motion, so a hardware-layer crash is only ever recovered by the whole-
unit systemd restart above (which correctly re-homes on every restart).
