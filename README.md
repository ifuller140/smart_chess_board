# Smart Chess Board

An automated chess board that plays physical chess against a human opponent using computer vision, a CoreXY gantry system with a permanent magnet, and the Stockfish chess engine.

> **Agents / Claude Code**: Start with [`CLAUDE.md`](CLAUDE.md) for project context and guidelines.

## Chess OS — Main Interface

**Chess OS** is the primary control interface for the board. It's a ROS 2 package (`chess_ui`) that runs a Flask web app on the Pi and exposes everything through a browser UI:

```bash
# Start Chess OS (from project root on the Pi, after colcon build + source install/setup.bash)
ros2 run chess_ui chess_ui

# Then open in any browser on the same network:
# http://<pi-ip>:5000
```

| Tab | What it does |
|-----|-------------|
| **Game** | Live FEN board, game start/stop, Stockfish moves, chess clock |
| **Gantry** | Jog controls, homing, board calibration, square navigation, canvas visualizer |
| **Hardware** | Servo (magnet), limit switch indicators, stepper commands, emergency stop |
| **Perception** | Live camera stream, piece-detector diff heatmap, per-square scores, detection threshold tuning |
| **Tests** | Run any hardware test category directly from the browser (via `test_runner_node`) |

> **Always launch Chess OS when working with the board.** It is the single pane of glass for gantry control, calibration, vision, hardware state, and test execution.

## Features

- **Automated piece movement** via CoreXY gantry with permanent magnet
- **Computer vision** for board state detection (FEN output)
- **Stockfish integration** for move calculation
- **ROS 2 architecture** for modular, reliable operation
- **Chess OS** web dashboard (port 5000) — unified control for everything

## Documentation

| Document | Description |
|----------|-------------|
| [CLAUDE.md](CLAUDE.md) | Agent entry point — start here |
| [docs/CONTEXT.md](docs/CONTEXT.md) | Full project context |
| [docs/hardware/](docs/hardware/) | Hardware specifications & wiring |
| [docs/software/](docs/software/) | Software architecture |
| [docs/features/](docs/features/) | Feature deep-dives |
| [.agent/workflows/deploy.md](.agent/workflows/deploy.md) | Deployment guide |

## Hardware Requirements

| Component | Model | Quantity |
|-----------|-------|----------|
| Controller | Raspberry Pi 4B (4GB+) | 1 |
| Stepper Motors | NEMA 11 + A4988 Driver | 2 |
| Servo Motor | SG90 | 2 (Z-axis + clock) |
| Permanent Magnet | — | 1 |
| Camera | RPi Camera Module v2 or USB | 1 |
| Limit Switches | Micro switch | 3 |

See [docs/hardware/components.md](docs/hardware/components.md) for full specifications.

## Quick Start

### 1. Install Dependencies (on Raspberry Pi)

```bash
# ROS 2 Humble — see .agent/workflows/deploy.md for full instructions
sudo apt install ros-humble-ros-base python3-colcon-common-extensions

# System libraries
sudo apt install python3-picamera2 libcamera-apps stockfish pigpio

# Python packages
pip3 install RPi.GPIO pigpio python-chess opencv-python numpy flask
```

### 2. Clone and Build

```bash
cd ~/dev
git clone https://github.com/ifuller140/smart_chess_board.git
cd smart_chess_board
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch

```bash
# Hardware only (motors, servos, limit switches)
ros2 launch chess_hw_interface hw_interface_launch.py

# Perception only (camera + board detection)
ros2 launch chess_perception perception_launch.py use_picamera2:=True

# Full system (all ROS nodes)
ros2 launch src/launch/full_system_launch.py
```

### 4. Open Chess OS

Already running if you used the full-system launch above. Otherwise, start it separately:

```bash
# In a second terminal — start the web UI
ros2 run chess_ui chess_ui

# Open http://<pi-ip>:5000 in your browser
```

Chess OS connects to ROS automatically if nodes are running. It also works standalone (`--no-ros`, falling back to a local OpenCV camera capture) for UI development without a running ROS graph.

### 5. Test Hardware

Use the **Tests tab** in Chess OS to run any hardware test from the browser, or from the command line:

```bash
./run_hw_test.sh --category gantry --subtest manual
```

See [.agent/workflows/hardware-test.md](.agent/workflows/hardware-test.md) for the full test suite.

## Project Structure

```
smart_chess_board/
├── src/                   # ROS 2 packages
│   ├── chess_hw_interface/   # GPIO drivers + hardware tests + test_runner_node
│   ├── chess_perception/     # Camera + board detection nodes
│   ├── chess_logic/          # Stockfish + game state machine
│   ├── gantry_control/       # CoreXY kinematics + motion planning
│   ├── chess_interfaces/     # ROS 2 message/service/action definitions
│   ├── chess_ui/             # Chess OS — web UI/control surface (main UI)
│   └── launch/               # full_system_launch.py
├── docs/                  # All documentation
├── code/                  # Standalone bench-test scripts
├── setup/                 # System configuration (sudoers)
├── cad/                   # CAD files
└── .agent/workflows/      # Deployment and testing guides
```

## Configuration

Edit GPIO pins in `src/chess_hw_interface/config/pins.yaml` to match your wiring.
See [docs/hardware/pinout.md](docs/hardware/pinout.md) for pin assignments.

## License

MIT License — see [LICENSE](LICENSE)

---

*Built with ROS 2 Humble on Raspberry Pi 4B*
