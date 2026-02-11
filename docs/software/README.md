# Software Documentation

> **Complete software architecture reference for the Smart Chess Board.**

## Overview

The system is built on ROS 2 Jazzy, using Python (`rclpy`) for all nodes. The architecture follows a modular design with separate packages for hardware interface, perception, logic, and motion control.

## Quick Links

| Document | Description |
|----------|-------------|
| [architecture.md](architecture.md) | High-level system design and data flow |
| [nodes.md](nodes.md) | Complete ROS 2 node reference |
| [interfaces.md](interfaces.md) | Custom messages, services, and actions |
| [configuration.md](configuration.md) | Configuration files and parameters |

## Package Overview

```
src/
├── chess_hw_interface/    # GPIO drivers
├── chess_perception/      # Computer vision
├── chess_logic/          # Game management
├── gantry_control/       # Motion control
└── launch/               # Launch files
```

| Package | Responsibility | Key Nodes |
|---------|---------------|-----------|
| `chess_hw_interface` | Low-level hardware control (GPIO) | `stepper_driver_node`, `servo_node`, `limit_switch_node` |
| `chess_perception` | Computer vision pipeline | `camera_node`, `board_detector_node`, `piece_detector_node` |
| `chess_logic` | Game rules and AI | `game_manager_node`, `chess_engine_node` |
| `gantry_control` | Motion planning and kinematics | `gantry_kinematics_node`, `motion_planner_node`, `homing_node` |

## Software Stack

| Layer | Technology |
|-------|------------|
| OS | Ubuntu 22.04 (Jammy) |
| Middleware | ROS 2 Jazzy |
| Language | Python 3.10+ |
| Vision | OpenCV 4.x |
| Chess Engine | python-chess + Stockfish |
| GPIO | RPi.GPIO |

## Quick Commands

### Build Workspace
```bash
cd ~/smart_chess_ws
colcon build
source install/setup.bash
```

### Run Full System
```bash
ros2 launch src/launch/full_system_launch.py
```

### Run Individual Nodes (for testing)
```bash
# Stepper driver
ros2 run chess_hw_interface stepper_driver_node

# Camera
ros2 run chess_perception camera_node

# Game manager
ros2 run chess_logic game_manager_node
```

### List Active Nodes
```bash
ros2 node list
```

### View Topics
```bash
ros2 topic list
ros2 topic echo /perception/board_state
```

## Development Workflow

1. **Edit code** in `src/<package>/<package>/nodes/`
2. **Rebuild** with `colcon build --packages-select <package>`
3. **Re-source** with `source install/setup.bash`
4. **Test** with `ros2 run <package> <node>`

## Testing

### Unit Tests
```bash
colcon test --packages-select <package>
```

### Component Tests
Standalone test scripts in `/code/`:
- `square.py` - Test stepper motion
- `ServoTestController.py` - Test servo
- `rotate.py` - Test motor rotation

---

*See [architecture.md](architecture.md) for system design details.*
