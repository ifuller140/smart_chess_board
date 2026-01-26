# Smart Chess Board

An automated chess board that plays physical chess against a human opponent using computer vision, a CoreXY gantry system with electromagnet, and the Stockfish chess engine.

> **🤖 Agents**: Start with [`AGENTS.md`](AGENTS.md) for project context and guidelines.

## Features

- 🎯 **Automated piece movement** via CoreXY gantry with electromagnet
- 📷 **Computer vision** for board state detection
- 🧠 **Stockfish integration** for move calculation
- ⏱️ **Chess clock** support (planned)
- 🔄 **ROS 2 architecture** for modular, reliable operation

## Documentation

| Document | Description |
|----------|-------------|
| [AGENTS.md](AGENTS.md) | Agent entry point |
| [docs/CONTEXT.md](docs/CONTEXT.md) | Full project context |
| [docs/hardware/](docs/hardware/) | Hardware specifications & wiring |
| [docs/software/](docs/software/) | Software architecture |
| [docs/features/](docs/features/) | Feature deep-dives |

## Hardware Requirements

| Component | Model | Quantity |
|-----------|-------|----------|
| Controller | Raspberry Pi 4B (4GB+) | 1 |
| Stepper Motors | 28BYJ-48 + ULN2003 | 2 |
| Servo Motor | SG90 | 1 |
| Electromagnet | 5V DC (~2.5kg hold) | 1 |
| Camera | RPi Camera Module v2 or USB | 1 |
| Limit Switches | Micro switch | 3 |

See [docs/hardware/components.md](docs/hardware/components.md) for full specifications.

## Quick Start

### 1. Install Dependencies

```bash
# ROS 2 Humble (see docs.ros.org for full instructions)
sudo apt install ros-humble-ros-base

# Python packages
pip3 install RPi.GPIO python-chess opencv-python
```

### 2. Build & Run

```bash
mkdir -p ~/smart_chess_ws/src
cd ~/smart_chess_ws/src
git clone <this-repo> smart_chess_board
cd ~/smart_chess_ws
colcon build
source install/setup.bash
ros2 launch src/smart_chess_board/src/launch/full_system_launch.py
```

### 3. Test Hardware

Use the `/hardware-test` workflow to validate components before full operation.

## Project Structure

```
smart_chess_board/
├── docs/                  # All documentation
├── src/                   # ROS 2 packages
│   ├── chess_hw_interface/
│   ├── chess_perception/
│   ├── chess_logic/
│   └── gantry_control/
├── code/                  # Standalone test scripts
├── cad/                   # CAD files
└── .agent/workflows/      # Agent task workflows
```

## Configuration

Edit GPIO pins in `src/chess_hw_interface/config/pins.yaml` to match your wiring.

See [docs/hardware/pinout.md](docs/hardware/pinout.md) for pin assignments.

## Contributing

1. Read [AGENTS.md](AGENTS.md) for project context
2. Follow the `/code-review` workflow checklist
3. Update documentation for any changes
4. Test on hardware before submitting

## License

MIT License - see [LICENSE](LICENSE)

---

*Built with ROS 2 Humble on Raspberry Pi*
