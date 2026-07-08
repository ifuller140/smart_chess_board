# ROS 2 Node Reference

> **Detailed specifications for all ROS 2 nodes.**

---

## Chess OS — Web Interface (`code/chess_os.py`)

Chess OS is not a ROS node — it's a Flask web application that acts as the **master control UI**. It runs as a standalone Python process and connects to the live ROS graph via rclpy.

**Start**: `python3 code/chess_os.py` → open `http://<pi-ip>:5000`

**ROS subscriptions** (reads live state):

| Topic | Type | Used for |
|-------|------|----------|
| `/gantry/pose` | `geometry_msgs/Point` | Live X/Y position on canvas |
| `/stepper/status` | `std_msgs/String` | Stepper health badge |
| `/gantry/status` | `std_msgs/String` | Gantry state badge |
| `/servo/state` | `std_msgs/String` | Magnet engaged/released |
| `/limit_switch/x_min` | `std_msgs/Bool` | X limit indicator |
| `/limit_switch/y_min` | `std_msgs/Bool` | Y limit indicator |
| `/limit_switch/clock_hit` | `std_msgs/Bool` | Clock button indicator |
| `/clock/white_time` | `std_msgs/Float32` | White clock MM:SS |
| `/clock/black_time` | `std_msgs/Float32` | Black clock MM:SS |
| `/camera/image_raw` | `sensor_msgs/Image` | Live MJPEG stream |

**ROS publishers**:

| Topic | Type | Used for |
|-------|------|----------|
| `/stepper/velocity` | `geometry_msgs/Twist` | Jog (20 Hz while key held) |
| `/stepper/command` | `geometry_msgs/Point` | Direct step command |
| `/emergency_stop` | `std_msgs/Bool` | E-stop button |

**ROS service clients**:

| Service | Used for |
|---------|---------|
| `/gantry/home` | Home button |
| `/servo/engage` | Magnet engage |
| `/servo/release` | Magnet release |
| `/clock/reset`, `/clock/pause`, `/clock/resume` | Clock controls |

**ROS action client**:

| Action | Used for |
|--------|---------|
| `/gantry/move` (`MoveGantry`) | Square goto / calibration moves (fire-and-forget) |

**Key API endpoints** (all JSON):

| Endpoint | Description |
|----------|-------------|
| `GET /api/status` | Full system state snapshot |
| `GET /api/stream/raw` | Raw camera MJPEG stream |
| `GET /api/stream/warp` | Warped board MJPEG stream |
| `POST /api/gantry/jog/start` | Start continuous jog `{dir, speed}` |
| `POST /api/gantry/jog/stop` | Stop jog |
| `POST /api/gantry/goto` | Move to square `{square}` or `{x_mm, y_mm}` |
| `POST /api/gantry/calibration/save_a1` | Save current pos as a1 |
| `POST /api/gantry/calibration/save_h8` | Save current pos as h8 |
| `POST /api/gantry/calibration/apply` | Compute sq_x/sq_y, write `board_calibration.json` |
| `POST /api/hw/estop` | Publish emergency stop |
| `POST /api/tests/run` | Launch a test `{category, subtest}` |
| `GET /api/tests/stream` | SSE stream of test stdout |

**Flags**:

| Flag | Effect |
|------|--------|
| `--no-ros` | Disable ROS; UI-only mode |
| `--port N` | Change listen port (default 5000) |
| `--camera N` | OpenCV camera device index |
| `--mode showcase` | Showcase display mode |

---

## chess_hw_interface Package

### stepper_driver_node

**Purpose**: Low-level control of NEMA 11 stepper motors via A4988 drivers.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `stepper_driver_node` |
| File | `nodes/stepper_driver_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/stepper/command` | `geometry_msgs/Point` | Step commands for motors A (x) and B (y), speed (z) |
| `/emergency_stop` | `std_msgs/Bool` | Emergency stop signal |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/stepper/status` | `std_msgs/String` | Current motor status |
| `/limit_switch/x_min` | `std_msgs/Bool` | X-axis limit switch state |
| `/limit_switch/y_min` | `std_msgs/Bool` | Y-axis limit switch state |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motorA_dir_pin` | int | 27 | BCM pin for Motor A direction |
| `motorA_step_pin` | int | 22 | BCM pin for Motor A step |
| `motorB_dir_pin` | int | 6 | BCM pin for Motor B direction |
| `motorB_step_pin` | int | 5 | BCM pin for Motor B step |
| `motor_enable_pin` | int | 17 | Shared A4988 enable pin (active LOW) |

---

### servo_node

**Purpose**: Control SG90 servo for Z-axis permanent-magnet lift.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `servo_node` |
| File | `nodes/servo_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/servo/engage` | `std_srvs/Trigger` | Drag position — magnet actuates a piece |
| `/servo/release` | `std_srvs/Trigger` | Clear position — no piece interaction |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `servo_pin` | int | 12 | BCM pin for PWM signal |
| `engage_angle_deg` | float | 145.0 | Servo angle for drag position |
| `release_angle_deg` | float | 170.0 | Servo angle for clear position |
| `movement_time` | float | 0.5 | Wait time after movement (s) |

Angles calibrated on real hardware via `code/test_z_servo.py`'s interactive sweep.

---

### limit_switch_node

**Purpose**: Monitor limit switches for homing and clock detection.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `limit_switch_node` |
| File | `nodes/limit_switch_node.py` |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/limit_switch/state` | `LimitSwitchState` | State of all limit switches |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `limit_switch_pins.x_min` | int | 10 | BCM pin for X-axis limit |
| `limit_switch_pins.y_min` | int | 9 | BCM pin for Y-axis limit |
| `limit_switch_pins.clock_hit` | int | 15 | BCM pin for clock button |
| `debounce_ms` | int | 20 | Debounce time in milliseconds |

---

### clock_display_node

**Purpose**: Control 7-segment displays for chess clock.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `clock_display_node` |
| File | `nodes/clock_display_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/clock/start_player` | `StartPlayer` | Start timer for specified player |
| `/clock/stop` | `std_srvs/Trigger` | Stop all timers |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/clock/set_time` | `ClockTime` | Set time on displays |

<!-- USER_ATTENTION: Define display wiring before implementing this node -->

---

### gpio_watchdog_node

**Purpose**: Safety monitoring and emergency stop.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `gpio_watchdog_node` |
| File | `nodes/gpio_watchdog_node.py` |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/safety/estop` | `std_msgs/Bool` | Emergency stop triggered |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/limit_switch/state` | `LimitSwitchState` | Monitor for unexpected triggers |

---

### chess_clock_node

**Purpose**: Dual chess timer — tracks remaining time for each player and fires flag-fall events.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `chess_clock_node` |
| File | `nodes/chess_clock_node.py` |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/clock/white_time` | `std_msgs/Float32` | White remaining seconds (1Hz) |
| `/clock/black_time` | `std_msgs/Float32` | Black remaining seconds (1Hz) |
| `/game_manager/clock_event` | `std_msgs/String` | `FLAG_WHITE` or `FLAG_BLACK` on timeout |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/game_manager/state` | `std_msgs/String` | GAME_OVER pauses; PROMOTION_WAIT pauses; PROMOTION_DONE resumes |
| `/game_manager/turn` | `std_msgs/String` | `WHITE` or `BLACK` to switch active clock |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/clock/reset` | `Trigger` | Reset both clocks to full time |
| `/clock/pause` | `Trigger` | Pause active clock |
| `/clock/resume` | `Trigger` | Resume paused clock |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `time_per_player_s` | float | 600.0 | Initial time per player in seconds |

---


### camera_node

**Purpose**: Capture images from Raspberry Pi camera.

| Property | Value |
|----------|-------|
| Package | `chess_perception` |
| Executable | `camera_node` |
| File | `nodes/camera_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/camera/capture` | `std_srvs/Trigger` | Capture single image |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Captured image |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | int[2] | [640, 480] | Image resolution [width, height] |
| `camera_id` | int | 0 | Camera device ID |

---

### board_detector_node

**Purpose**: Detect chess board grid and corners.

| Property | Value |
|----------|-------|
| Package | `chess_perception` |
| Executable | `board_detector_node` |
| File | `nodes/board_detector_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input image |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/perception/board_geometry` | `BoardGeometry` | Board corners and transformation |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `canny_threshold1` | int | 50 | Canny edge lower threshold |
| `canny_threshold2` | int | 150 | Canny edge upper threshold |
| `hough_threshold` | int | 100 | Hough line threshold |

---

### piece_detector_node

**Purpose**: Detect chess pieces via occupancy + color classification and publish FEN. Uses game-state-assisted piece typing (subscribes to `/game_manager/board_fen` for authoritative board state).

| Property | Value |
|----------|-------|
| Package | `chess_perception` |
| Executable | `piece_detector_node` |
| File | `nodes/piece_detector_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Live camera feed |
| `/perception/board_geometry` | `BoardState` | Board corners (from board_detector) |
| `/game_manager/board_fen` | `std_msgs/String` | Authoritative FEN for piece typing |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/perception/board_state` | `BoardState` | Detected FEN + 64-element piece array |
| `/perception/piece_debug` | `sensor_msgs/Image` | Annotated warped board (W/B overlays) |
| `/perception/reference_status` | `std_msgs/String` | Reference baseline capture progress |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/perception/capture_reference` | `Trigger` | Capture empty-board baseline (call before game) |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `occupancy_diff_threshold` | int | 25 | Pixel diff threshold for piece detection |
| `white_piece_brightness` | float | 0.65 | Brightness percentile for white piece classification |
| `reference_capture_count` | int | 5 | Frames to average for baseline |
| `warp_size` | int | 400 | Warped board size in pixels |

> **Pre-game setup**: Call `/perception/capture_reference` 5+ times with empty board before starting a game.

---

## chess_logic Package

### game_manager_node

**Purpose**: Full 10-state game manager — orchestrates homing, player move detection, engine calls, gantry execution, and clock management.

| Property | Value |
|----------|-------|
| Package | `chess_logic` |
| Executable | `game_manager_node` |
| File | `nodes/game_manager_node.py` |

**States**: `STARTUP` → `HOMING` → `IDLE` → `WAITING_PLAYER_MOVE` → `CAPTURING_BOARD` → `VALIDATING_MOVE` → `CALCULATING_RESPONSE` → `EXECUTING_MOVE` → `HITTING_CLOCK` → `GAME_OVER` (+ `PROMOTION_WAIT`)

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/limit_switch/clock_hit` | `Bool` | Human pressed chess clock |
| `/perception/board_state` | `BoardState` | Post-move detected FEN |
| `/game_manager/clock_event` | `String` | FLAG_WHITE or FLAG_BLACK from chess_clock_node |
| `/motion/done` | `Bool` | Motion planner move complete |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/game_manager/state` | `String` | Current state name (10 states) |
| `/game_manager/turn` | `String` | `WHITE` or `BLACK` for clock_node |
| `/game_manager/board_fen` | `String` | Authoritative FEN for piece_detector |
| `/motion/command` | `String` | `"UCI FEN"` to motion_planner |

**Services Called**:
| Service | Type | Description |
|---------|------|-------------|
| `/gantry/home` | `Trigger` | Home gantry on startup |
| `/camera/capture` | `Trigger` | Trigger human move capture |
| `/chess_engine/request_move` | `RequestMove` | Get engine response |
| `/clock/hit` | `Trigger` | Servo presses clock button |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `engine_think_time_s` | float | 2.0 | Engine think time |
| `board_capture_timeout_s` | float | 5.0 | Camera capture timeout |
| `motion_timeout_s` | float | 120.0 | Gantry move timeout |
| `homing_timeout_s` | float | 90.0 | Homing timeout |

---

### chess_engine_node

**Purpose**: Interface with Stockfish chess engine.

| Property | Value |
|----------|-------|
| Package | `chess_logic` |
| Executable | `chess_engine_node` |
| File | `nodes/chess_engine_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/chess_engine/request_move` | `RequestMove` | Returns best move UCI for given FEN |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `engine_path` | string | `/usr/games/stockfish` | Path to Stockfish binary |
| `think_time` | float | 2.0 | Engine think time in seconds |

---

## gantry_control Package

### gantry_kinematics_node

**Purpose**: CoreXY kinematics — converts (x_mm, y_mm) goals into stepper motor step sequences via trapezoidal velocity profiles.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `gantry_kinematics_node` |
| File | `nodes/gantry_kinematics_node.py` |

**Actions**:
| Action | Type | Description |
|--------|------|-------------|
| `/gantry/move` | `MoveGantry` | Move to (target_x_mm, target_y_mm) |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/stepper/command` | `geometry_msgs/Point` | Motor A/B step commands |
| `/gantry/pose` | `geometry_msgs/Point` | Current X/Y position in mm |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steps_per_mm` | float | 5.0 | Steps per mm (calibrate with hardware) |
| `max_speed_mm_s` | float | 100.0 | Maximum speed |
| `acceleration_mm_s2` | float | 200.0 | Acceleration |

---

### motion_planner_node

**Purpose**: High-level move sequencer — supports captures, castling, en passant, promotion with safe graveyard routing.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `motion_planner_node` |
| File | `nodes/motion_planner_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/motion/command` | `String` | `"UCI FEN"` move command |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/motion/done` | `Bool` | True on success, False on failure |
| `/game_manager/state` | `String` | `PROMOTION_WAIT` when pawn promotes |

**Actions Called**:
| Action | Type | Description |
|--------|------|-------------|
| `/gantry/move` | `MoveGantry` | Move to absolute XY position |

**Services Called**:
| Service | Type | Description |
|---------|------|-------------|
| `/servo/engage` | `Trigger` | Lower permanent magnet arm |
| `/servo/release` | `Trigger` | Raise permanent magnet arm |

**Parameters** (from `board_map.yaml`):
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `square_size_mm` | float | 25.0 | Board square width in mm |
| `board_origin_x_mm` | float | 20.0 | X of a1 square center from origin (mm) — calibrate on hardware |
| `board_origin_y_mm` | float | 20.0 | Y of a1 square center from origin (mm) — calibrate on hardware |
| `graveyard_origin_x_mm` | float | 210.0 | Graveyard zone X origin |
| `graveyard_origin_y_mm` | float | 215.0 | Graveyard zone Y origin |
| `graveyard_slot_spacing_mm` | float | 22.0 | Spacing between graveyard slots |

---

### homing_node

**Purpose**: Home the gantry to the bottom-left origin (0,0). Sequence: home Y (front limit at Y=0), home X (right limit at X_MAX), drive X back to X=0, reset stepper position counter.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `homing_node` |
| File | `nodes/homing_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/gantry/home` | `Trigger` | Execute full homing sequence |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `homing_speed_mm_s` | float | 30.0 | Homing travel speed |
| `backoff_distance_mm` | float | 5.0 | Backoff distance after limit hit |

---

*See [interfaces.md](interfaces.md) for message definitions.*
```

## Test Commands Reference

```bash
# Hardware tests
python3 -m chess_hw_interface.testing.test_runner --test square_nav      # Square navigation
python3 -m chess_hw_interface.testing.test_runner --test vision           # Vision pipeline
python3 -m chess_hw_interface.testing.test_runner --test clock_integration # Clock timer
python3 -m chess_hw_interface.testing.test_runner --test magnet           # Magnet servo

# Full system
ros2 launch smart_chess_board full_system_launch.py
ros2 launch smart_chess_board full_system_launch.py use_picamera2:=False think_time:=1.0
```
