# ROS 2 Node Reference

> **Detailed specifications for all ROS 2 nodes.**

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

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `motorA_dir_pin` | int | 27 | BCM pin for Motor A direction |
| `motorA_step_pin` | int | 22 | BCM pin for Motor A step |
| `motorB_dir_pin` | int | 6 | BCM pin for Motor B direction |
| `motorB_step_pin` | int | 5 | BCM pin for Motor B step |
| `dir_setup_us` | int | 5 | DIR setup time in microseconds |
| `step_pulse_us` | int | 20 | STEP pulse width in microseconds |
| `min_step_delay_ms` | float | 3.0 | Min delay (max speed, ~90%) |
| `max_step_delay_ms` | float | 50.0 | Max delay (min speed, ~0%) |

---

### servo_node

**Purpose**: Control SG90 servo for Z-axis electromagnet lift.

| Property | Value |
|----------|-------|
| Package | `chess_hw_interface` |
| Executable | `servo_node` |
| File | `nodes/servo_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/servo/engage` | `std_srvs/Trigger` | Lower magnet (engage) |
| `/servo/release` | `std_srvs/Trigger` | Raise magnet (release) |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `servo_pin` | int | 12 | BCM pin for PWM signal |
| `engage_pwm` | float | 2.5 | Duty cycle for down position |
| `release_pwm` | float | 7.5 | Duty cycle for up position |
| `movement_time` | float | 0.5 | Wait time after movement (s) |

<!-- USER_ATTENTION: Calibrate engage_pwm and release_pwm for your setup -->

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
| `limit_switch_pins.x_min` | int | 6 | BCM pin for X-axis limit |
| `limit_switch_pins.y_min` | int | 13 | BCM pin for Y-axis limit |
| `limit_switch_pins.clock_hit` | int | 19 | BCM pin for clock button |
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

## chess_perception Package

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

**Purpose**: Identify pieces on each square.

| Property | Value |
|----------|-------|
| Package | `chess_perception` |
| Executable | `piece_detector_node` |
| File | `nodes/piece_detector_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Input image |
| `/perception/board_geometry` | `BoardGeometry` | Board transform |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/perception/board_state` | `BoardState` | FEN and piece array |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `detection_method` | string | "histogram" | Detection algorithm |
| `empty_threshold` | float | 0.3 | Threshold for empty squares |

---

## chess_logic Package

### game_manager_node

**Purpose**: Main state machine coordinating the game loop.

| Property | Value |
|----------|-------|
| Package | `chess_logic` |
| Executable | `game_manager_node` |
| File | `nodes/game_manager_node.py` |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/perception/board_state` | `BoardState` | Detected board state |
| `/limit_switch/state` | `LimitSwitchState` | Clock hit detection |

**Services Called**:
| Service | Type | Description |
|---------|------|-------------|
| `/camera/capture` | `Trigger` | Trigger image capture |
| `/chess_engine/request_move` | `RequestMove` | Get engine move |

**Actions Called**:
| Action | Type | Description |
|--------|------|-------------|
| `/gantry/move` | `MoveGantry` | Execute piece movement |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `time_control` | int | 600 | Time per player (seconds) |
| `engine_think_time` | float | 1.0 | Engine calculation time |

---

### chess_engine_node

**Purpose**: Interface with python-chess and Stockfish.

| Property | Value |
|----------|-------|
| Package | `chess_logic` |
| Executable | `chess_engine_node` |
| File | `nodes/chess_engine_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/chess_engine/request_move` | `RequestMove` | Calculate best move |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stockfish_path` | string | "/usr/bin/stockfish" | Path to Stockfish binary |
| `skill_level` | int | 10 | Engine skill (0-20) |
| `depth` | int | 15 | Search depth |

---

## gantry_control Package

### gantry_kinematics_node

**Purpose**: CoreXY kinematics and low-level motion control.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `gantry_kinematics_node` |
| File | `nodes/gantry_kinematics_node.py` |

**Actions**:
| Action | Type | Description |
|--------|------|-------------|
| `/gantry/move` | `MoveGantry` | Move to X/Y position |

**Publications**:
| Topic | Type | Description |
|-------|------|-------------|
| `/stepper/command` | `StepperCommand` | Motor step commands |
| `/gantry/pose` | `geometry_msgs/Point` | Current X/Y position |

**Subscriptions**:
| Topic | Type | Description |
|-------|------|-------------|
| `/stepper/status` | `StepperStatus` | Motor feedback |
| `/limit_switch/state` | `LimitSwitchState` | Limit switch state |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `steps_per_mm` | float | 51.2 | Calibrated steps/mm |
| `max_speed_mm_s` | float | 10.0 | Maximum speed |
| `acceleration_mm_s2` | float | 50.0 | Acceleration |

<!-- USER_ATTENTION: Calibrate steps_per_mm with actual hardware -->

---

### motion_planner_node

**Purpose**: High-level pick-and-place motion sequences.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `motion_planner_node` |
| File | `nodes/motion_planner_node.py` |

**Actions Called**:
| Action | Type | Description |
|--------|------|-------------|
| `/gantry/move` | `MoveGantry` | Execute individual moves |

**Services Called**:
| Service | Type | Description |
|---------|------|-------------|
| `/servo/engage` | `Trigger` | Engage magnet |
| `/servo/release` | `Trigger` | Release magnet |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `z_safe_height` | float | 20.0 | Safe travel height (mm) |
| `approach_speed` | float | 5.0 | Slow approach speed |

---

### homing_node

**Purpose**: Home the gantry to known position.

| Property | Value |
|----------|-------|
| Package | `gantry_control` |
| Executable | `homing_node` |
| File | `nodes/homing_node.py` |

**Services**:
| Service | Type | Description |
|---------|------|-------------|
| `/gantry/home` | `Trigger` | Start homing sequence |

**Parameters**:
| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `homing_speed` | float | 5.0 | Speed during homing |
| `backoff_distance` | float | 5.0 | Distance to back off (mm) |

---

*See [interfaces.md](interfaces.md) for message definitions.*
