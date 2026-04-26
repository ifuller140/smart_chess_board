# Smart Chess Board - ROS 2 Software Architecture

This document outlines the software architecture for the Smart Chess Board project. The system is built on ROS 2 (humble) and uses Python (`rclpy`) for all nodes.

## High-Level Overview

The system is composed of modular nodes communicating via topics, services, and actions.
- **Hardware Interface**: Controls steppers, servos, limit switches, and displays.
- **Perception**: Captures images, detects the board grid, and identifies piece configurations.
- **Logic**: Manages game state, validates moves, and interfaces with a chess engine.
- **Motion Control**: Handles CoreXY kinematics, path planning, and magnet actuation.

## Repository Structure

```text
smart_chessboard/                 # workspace root
├─ src/
│  ├─ chess_hw_interface/         # Hardware drivers (GPIO)
│  │  ├─ package.xml
│  │  ├─ setup.py
│  │  ├─ chess_hw_interface/
│  │  │  ├─ nodes/
│  │  │  │  ├─ stepper_driver_node.py
│  │  │  │  ├─ servo_node.py
│  │  │  │  ├─ limit_switch_node.py
│  │  │  │  ├─ clock_display_node.py
│  │  │  │  └─ gpio_watchdog_node.py
│  │  │  ├─ launch/
│  │  │  │  └─ hw_interface_launch.py
│  │  │  ├─ config/
│  │  │  │  └─ pins.yaml
│  │  │  └─ resource/...
│  ├─ chess_perception/           # Computer Vision
│  │  ├─ package.xml
│  │  ├─ chess_perception/
│  │  │  ├─ nodes/
│  │  │  │  ├─ camera_node.py
│  │  │  │  ├─ board_detector_node.py
│  │  │  │  └─ piece_detector_node.py
│  │  │  ├─ msg/
│  │  │  │  └─ BoardState.msg
│  │  │  ├─ config/
│  │  │  │  └─ cv_params.yaml
│  ├─ chess_logic/                # Game Rules & Engine
│  │  ├─ package.xml
│  │  ├─ chess_logic/
│  │  │  ├─ nodes/
│  │  │  │  ├─ chess_engine_node.py
│  │  │  │  └─ game_manager_node.py
│  │  │  ├─ srv/
│  │  │  │  └─ RequestMove.srv
│  │  │  └─ config/
│  ├─ gantry_control/             # Motion Planning & Kinematics
│  │  ├─ package.xml
│  │  ├─ gantry_control/
│  │  │  ├─ nodes/
│  │  │  │  ├─ gantry_kinematics_node.py
│  │  │  │  ├─ motion_planner_node.py
│  │  │  │  └─ homing_node.py
│  │  │  ├─ action/
│  │  │  │  └─ MoveGantry.action
│  │  │  └─ config/
│  ├─ sim_bridge/                 # Simulation Adapters
│  └─ launch/
│     └─ full_system_launch.py
└─ README.md
```

## Custom Interfaces

### Message: `BoardState.msg`
```text
std_msgs/Header header
int8[64] pieces          # 8x8 flattened array (0=empty, 1=white pawn, etc.)
string fen               # FEN string representation
geometry_msgs/Point[4] corners # Board corners in image coordinates
```

### Service: `RequestMove.srv`
```text
string fen               # Current board state
---
string best_move_uci     # e.g., "e2e4"
float32 think_time
bool success
```

### Action: `MoveGantry.action`
```text
# Goal
float32 x_mm
float32 y_mm
float32 speed_mm_s
bool engage_magnet
bool release_magnet
---
# Result
bool success
string message
---
# Feedback
float32 current_x_mm
float32 current_y_mm
float32 percent_complete
```

## Node Details

### 1. `stepper_driver_node.py` (chess_hw_interface)
- **Purpose**: Low-level control of 28BYJ-48 steppers via ULN2003.
- **Topics**:
    - Sub: `/stepper/command`
    - Pub: `/stepper/status`
- **Config**: `pins.yaml` (Motor pins, sequence type).
- **Logic**: Serializes step requests, handles GPIO pulsing.

### 2. `gantry_kinematics_node.py` (gantry_control)
- **Purpose**: CoreXY kinematics (Step <-> MM conversion).
- **Actions**:
    - Server: `/gantry/move` (`MoveGantry`)
- **Topics**:
    - Pub: `/gantry/pose`
    - Sub: `/limit_switch/state`
- **Logic**: Calculates `dA = dX + dY`, `dB = dX - dY`. Handles acceleration.

### 3. `motion_planner_node.py` (gantry_control)
- **Purpose**: High-level pick-and-place sequencing.
- **Logic**:
    1. Move to Source XY.
    2. Engage Magnet (Servo).
    3. Move to Target XY.
    4. Release Magnet.
- **Config**: `board_map.yaml` (Square -> MM coordinates).

### 4. `servo_node.py` (chess_hw_interface)
- **Purpose**: Z-axis/Magnet control.
- **Services**: `/servo/engage`, `/servo/release`.
- **Config**: PWM values for Up/Down positions.

### 5. `limit_switch_node.py` (chess_hw_interface)
- **Purpose**: Read X-min, Y-min, and Clock-hit switches.
- **Topics**: Publishes `/limit_switch/state`.
- **Logic**: Debouncing, triggers emergency stop if limits hit unexpectedly.

### 6. `clock_display_node.py` (chess_hw_interface)
- **Purpose**: Control 7-segment displays.
- **Services**: `/clock/start_player`, `/clock/stop`.
- **Topics**: Sub `/clock/set_time`.

### 7. `camera_node.py` (chess_perception)
- **Purpose**: Capture images from RPi Camera.
- **Services**: `/camera/capture`.
- **Topics**: Pub `/camera/image_raw`.

### 8. `board_detector_node.py` (chess_perception)
- **Purpose**: Detect board grid and corners.
- **Logic**: Canny edge detection, Hough lines, perspective transform.
- **Topics**: Pub `/perception/board_geometry`.

### 9. `piece_detector_node.py` (chess_perception)
- **Purpose**: Identify pieces on the board.
- **Logic**: Color histograms, blob detection (No ML).
- **Topics**: Pub `/perception/board_state` (FEN).

### 10. `chess_engine_node.py` (chess_logic)
- **Purpose**: Interface with `python-chess` / Stockfish.
- **Services**: `/chess_engine/request_move`.

### 11. `game_manager_node.py` (chess_logic)
- **Purpose**: Main state machine (IDLE, WAITING_MOVE, MOVING, etc.).
- **Logic**:
    - Wait for Clock Hit -> Trigger Camera.
    - Compare Board State -> Validate Move.
    - Request Engine Move -> Trigger Gantry.

### 12. `homing_node.py` (gantry_control)
- **Purpose**: Homing sequence (Move to limits, zero coordinates).

### 13. `gpio_watchdog_node.py` (chess_hw_interface)
- **Purpose**: Safety monitoring. Triggers E-Stop on faults.

## Launch Strategy
- **`hw_interface_launch.py`**: Drivers.
- **`perception_launch.py`**: Vision.
- **`gantry_launch.py`**: Motion.
- **`logic_launch.py`**: Game loop.
- **`full_system_launch.py`**: All-in-one.

## Next Steps
1. Create package structure.
2. Populate `pins.yaml` with user-provided pinouts.
3. Implement `gantry_kinematics_node.py` or `camera_node.py` as a starting point.
