# ROS 2 Interfaces

> **Custom message, service, and action definitions.**

---

## Custom Messages

### BoardState.msg
**Package**: `chess_perception`
**File**: `msg/BoardState.msg`

Represents the current state of the chess board.

```
std_msgs/Header header
int8[64] pieces          # 8x8 flattened array (see encoding below)
string fen               # FEN string representation
geometry_msgs/Point[4] corners  # Board corners in image coordinates
```

**Piece Encoding**:
| Value | Piece | Value | Piece |
|-------|-------|-------|-------|
| 0 | Empty | | |
| 1 | White Pawn | -1 | Black Pawn |
| 2 | White Knight | -2 | Black Knight |
| 3 | White Bishop | -3 | Black Bishop |
| 4 | White Rook | -4 | Black Rook |
| 5 | White Queen | -5 | Black Queen |
| 6 | White King | -6 | Black King |

**Array Index to Square Mapping**:
```
Index 0  = a1,  Index 1  = b1,  ... Index 7  = h1
Index 8  = a2,  Index 9  = b2,  ... Index 15 = h2
...
Index 56 = a8,  Index 57 = b8,  ... Index 63 = h8
```

---

### StepperCommand.msg
**Package**: `chess_hw_interface`
**File**: `msg/StepperCommand.msg` (to be created)

Commands for stepper motors.

```
int32 steps_a           # Steps for motor A (+ = forward)
int32 steps_b           # Steps for motor B (+ = forward)
float32 step_delay      # Delay between steps (seconds)
bool sync               # Move motors simultaneously
```

---

### StepperStatus.msg
**Package**: `chess_hw_interface`
**File**: `msg/StepperStatus.msg` (to be created)

Status feedback from stepper motors.

```
std_msgs/Header header
int32 position_a        # Current position motor A (steps from home)
int32 position_b        # Current position motor B (steps from home)
bool moving             # True if currently moving
bool error              # True if error occurred
string error_message    # Error description
```

---

### LimitSwitchState.msg
**Package**: `chess_hw_interface`
**File**: `msg/LimitSwitchState.msg` (to be created)

State of all limit switches.

```
std_msgs/Header header
bool x_min_triggered    # X-axis limit switch
bool y_min_triggered    # Y-axis limit switch
bool clock_hit          # Clock button pressed
```

---

### BoardGeometry.msg
**Package**: `chess_perception`
**File**: `msg/BoardGeometry.msg` (to be created)

Detected board geometry in image coordinates.

```
std_msgs/Header header
geometry_msgs/Point[4] corners    # [top-left, top-right, bottom-right, bottom-left]
float32[9] homography_matrix      # 3x3 perspective transform (flattened)
bool valid                        # True if board was detected
```

---

## Custom Services

### RequestMove.srv
**Package**: `chess_logic`
**File**: `srv/RequestMove.srv`

Request the best move from the chess engine.

```
# Request
string fen               # Current board state in FEN notation
float32 time_limit      # Maximum thinking time (seconds)
---
# Response
string best_move_uci     # Best move in UCI format (e.g., "e2e4")
float32 think_time      # Actual time spent thinking
int32 evaluation        # Position evaluation (centipawns)
bool success            # True if move was found
string message          # Error message if failed
```

---

### StartPlayer.srv
**Package**: `chess_hw_interface`
**File**: `srv/StartPlayer.srv` (to be created)

Start the clock for a specific player.

```
# Request
int8 player             # 1 = white, 2 = black
---
# Response
bool success
float32 remaining_time  # Time remaining for that player
```

---

## Custom Actions

### MoveGantry.action
**Package**: `gantry_control`
**File**: `action/MoveGantry.action`

Move the gantry to a specified position with optional magnet control.

```
# Goal
float32 x_mm            # Target X position (mm from home)
float32 y_mm            # Target Y position (mm from home)
float32 speed_mm_s      # Movement speed (mm/s)
bool engage_magnet      # Engage magnet at destination
bool release_magnet     # Release magnet at destination
---
# Result
bool success            # True if move completed
string message          # Error message if failed
float32 final_x_mm      # Actual final X position
float32 final_y_mm      # Actual final Y position
---
# Feedback
float32 current_x_mm    # Current X position
float32 current_y_mm    # Current Y position
float32 percent_complete  # 0.0 to 1.0
```

---

### ExecuteChessMove.action
**Package**: `gantry_control`
**File**: `action/ExecuteChessMove.action` (to be created)

Execute a complete chess move (pick, move, place, optional capture).

```
# Goal
string move_uci         # Move in UCI format (e.g., "e2e4")
bool is_capture         # True if capturing a piece
string captured_piece   # Square of captured piece if is_capture
---
# Result
bool success
string message
---
# Feedback
string current_phase    # "approaching", "picking", "moving", "placing"
float32 percent_complete
```

---

## Standard Messages Used

| Message Type | Package | Usage |
|--------------|---------|-------|
| `std_msgs/Header` | std_msgs | Timestamp and frame ID |
| `std_msgs/Bool` | std_msgs | Boolean states |
| `std_msgs/String` | std_msgs | Text messages |
| `geometry_msgs/Point` | geometry_msgs | X/Y/Z coordinates |
| `geometry_msgs/Pose` | geometry_msgs | Position + orientation |
| `sensor_msgs/Image` | sensor_msgs | Camera images |
| `std_srvs/Trigger` | std_srvs | Simple trigger services |

---

## Building Custom Interfaces

To add new message/service/action definitions:

1. Create the `.msg`, `.srv`, or `.action` file in the appropriate `msg/`, `srv/`, or `action/` directory

2. Update `package.xml`:
```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

3. Update `CMakeLists.txt` (or equivalent for ament_python):
```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YourMessage.msg"
  "srv/YourService.srv"
  "action/YourAction.action"
)
```

4. Build and source:
```bash
colcon build --packages-select <package>
source install/setup.bash
```

---

*See [configuration.md](configuration.md) for parameter references.*
