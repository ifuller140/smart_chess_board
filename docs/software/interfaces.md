# ROS 2 Interfaces

> **Custom message, service, and action definitions.**

All custom `.msg`/`.srv`/`.action` definitions live in one consolidated package: **`src/chess_interfaces/`** (`ament_cmake` build type). Every other package (`chess_hw_interface`, `chess_perception`, `chess_logic`, `gantry_control`) depends on it and imports from `chess_interfaces.msg`/`.srv`/`.action` — there is no other package that owns interface definitions.

---

## Custom Messages

### BoardState.msg
**Package**: `chess_interfaces`
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

## Custom Services

### RequestMove.srv
**Package**: `chess_interfaces`
**File**: `srv/RequestMove.srv`

Request the best move from the chess engine.

```
# Request
string fen               # Current board state in FEN notation
float32 think_time_s     # How long the engine should think (seconds)
---
# Response
string best_move_uci     # Best move in UCI format (e.g., "e2e4")
float32 think_time       # Actual time spent thinking
bool success             # True if move was found
```

---

## Custom Actions

### MoveGantry.action
**Package**: `chess_interfaces`
**File**: `action/MoveGantry.action`

Move the gantry to a specified position with optional magnet engagement at the destination.

```
# Goal
float32 target_x_mm      # Target X position (mm from home)
float32 target_y_mm      # Target Y position (mm from home)
float32 speed_mm_s       # Movement speed (mm/s)
bool engage_magnet       # Engage magnet at destination
---
# Result
bool success             # True if move completed
string message           # Error message if failed
---
# Feedback
float32 current_x_mm     # Current X position
float32 current_y_mm     # Current Y position
float32 percent_complete # 0.0 to 1.0
```

Note the goal fields are `target_x_mm`/`target_y_mm` (not `x_mm`/`y_mm`) — this was a real field-name mismatch bug between `gantry_kinematics_node` and this action definition, fixed in commit history; keep both in sync if you ever change one.

---

## Standard Messages Used

| Message Type | Package | Usage |
|--------------|---------|-------|
| `std_msgs/Header` | std_msgs | Timestamp and frame ID |
| `std_msgs/Bool` | std_msgs | Boolean states (e.g. `/emergency_stop`, limit switches) |
| `std_msgs/String` | std_msgs | Text messages (e.g. FEN strings, servo state) |
| `std_msgs/Float32` | std_msgs | Scalar values (e.g. clock times) |
| `geometry_msgs/Point` | geometry_msgs | X/Y/Z coordinates (board corners) |
| `sensor_msgs/Image` | sensor_msgs | Camera images |
| `std_srvs/Trigger` | std_srvs | Simple trigger services (servo engage/release, clock pause/resume, capture reference, etc.) |

Most hardware state (limit switches, clock times, servo state, node health) is published as plain `std_msgs` types on topic names that describe their purpose, rather than as custom message types — see `docs/software/nodes.md` for the full topic/service/action reference per node.

---

## Building Custom Interfaces

To add new message/service/action definitions:

1. Create the `.msg`, `.srv`, or `.action` file under `src/chess_interfaces/msg/`, `srv/`, or `action/`.

2. Register it in `src/chess_interfaces/CMakeLists.txt`:
```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoardState.msg"
  "srv/RequestMove.srv"
  "action/MoveGantry.action"
  "msg/YourNewMessage.msg"
)
```

3. Make sure the consuming package's `package.xml` has `<depend>chess_interfaces</depend>` (all four consumer packages already do).

4. Build and source:
```bash
colcon build --packages-select chess_interfaces
source install/setup.bash
```

5. Import in Python as `from chess_interfaces.msg import YourNewMessage` (or `.srv` / `.action`).

---

*See [configuration.md](configuration.md) for parameter references, and [nodes.md](nodes.md) for the full per-node topic/service/action listing.*
