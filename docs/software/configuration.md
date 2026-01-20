# Configuration Reference

> **All configuration files and parameters.**

---

## Configuration File Locations

| File | Package | Purpose |
|------|---------|---------|
| `pins.yaml` | chess_hw_interface | GPIO pin assignments |
| `cv_params.yaml` | chess_perception | Computer vision parameters |
| `board_map.yaml` | gantry_control | Square-to-coordinate mapping |

---

## pins.yaml

**Location**: `src/chess_hw_interface/config/pins.yaml`

GPIO pin configuration for all hardware interfaces.

```yaml
# GPIO Pin Configuration for Smart Chess Board
# Uses BCM numbering

stepper_driver:
  ros__parameters:
    # Motor A (CoreXY: X+Y component)
    motorA_pins: [17, 18, 27, 22]  # [IN1, IN2, IN3, IN4]
    # Motor B (CoreXY: X-Y component)  
    motorB_pins: [23, 24, 25, 5]   # [IN1, IN2, IN3, IN4]
    step_sequence: 'half'          # 'full' or 'half' stepping
    step_delay_default: 0.001      # Minimum seconds between steps

servo_node:
  ros__parameters:
    servo_pin: 12                  # Hardware PWM capable pin
    engage_pwm: 2.5                # Duty cycle for magnet down
    release_pwm: 7.5               # Duty cycle for magnet up
    movement_time: 0.5             # Wait time after movement

limit_switch_node:
  ros__parameters:
    limit_switch_pins:
      x_min: 6                     # X-axis homing switch
      y_min: 13                    # Y-axis homing switch
      clock_hit: 19                # Clock/turn button
    debounce_ms: 20                # Debounce time

clock_display:
  ros__parameters:
    display1_pins: [0, 0, 0, 0]    # 7-segment display 1
    display2_pins: [0, 0, 0, 0]    # 7-segment display 2
```

<!-- USER_ATTENTION: Update pin assignments to match your actual wiring! -->

---

## cv_params.yaml

**Location**: `src/chess_perception/config/cv_params.yaml`

Computer vision tuning parameters.

```yaml
# Computer Vision Parameters

camera_node:
  ros__parameters:
    resolution: [640, 480]         # Width, Height
    camera_id: 0                   # /dev/video0
    fps: 30                        # Frames per second
    auto_exposure: true
    exposure_time: -1              # -1 for auto

board_detector_node:
  ros__parameters:
    # Canny edge detection
    canny_threshold1: 50
    canny_threshold2: 150
    # Hough line detection
    hough_rho: 1                   # Distance resolution (pixels)
    hough_theta: 0.0174            # ~1 degree in radians
    hough_threshold: 100           # Accumulator threshold
    hough_min_line_length: 50
    hough_max_line_gap: 10
    # Board detection
    min_board_size: 100            # Minimum board size (pixels)
    max_board_size: 600            # Maximum board size (pixels)

piece_detector_node:
  ros__parameters:
    detection_method: 'histogram'   # 'histogram' or 'blob'
    # For histogram detection
    empty_threshold: 0.3           # Occupancy threshold
    white_hue_range: [0, 30]       # HSV hue range for white pieces
    black_hue_range: [0, 180]      # HSV hue range for black pieces
    saturation_threshold: 40       # Min saturation for color detection
    # For blob detection
    min_blob_area: 100
    max_blob_area: 5000
```

<!-- USER_ATTENTION: Tune these parameters based on your lighting and camera setup -->

---

## board_map.yaml

**Location**: `src/gantry_control/config/board_map.yaml`

Maps chess square notation to physical X/Y coordinates.

```yaml
# Board Coordinate Mapping
# All coordinates in millimeters from home position (0,0)

gantry_kinematics_node:
  ros__parameters:
    # Physical board parameters
    square_size_mm: 25.0           # Size of one square
    board_origin_x_mm: 25.0        # X offset to a1 square center
    board_origin_y_mm: 25.0        # Y offset to a1 square center
    
    # Calibration
    steps_per_mm: 51.2             # Calibrated value
    
motion_planner_node:
  ros__parameters:
    # Square coordinates (auto-calculated from square_size)
    # These can be overridden for fine-tuning
    square_positions:
      a1: [25.0, 25.0]
      a2: [25.0, 50.0]
      a3: [25.0, 75.0]
      a4: [25.0, 100.0]
      a5: [25.0, 125.0]
      a6: [25.0, 150.0]
      a7: [25.0, 175.0]
      a8: [25.0, 200.0]
      b1: [50.0, 25.0]
      # ... (remaining squares)
      h8: [200.0, 200.0]
    
    # Graveyard positions for captured pieces
    graveyard_white:
      positions: [[230, 25], [255, 25], [280, 25],
                  [230, 50], [255, 50], [280, 50],
                  [230, 75], [255, 75], [280, 75]]
    graveyard_black:
      positions: [[230, 125], [255, 125], [280, 125],
                  [230, 150], [255, 150], [280, 150],
                  [230, 175], [255, 175], [280, 175]]
    
    # Motion parameters
    z_safe_height_mm: 20.0         # Height for travel moves
    approach_speed_mm_s: 5.0       # Slow approach to piece
    travel_speed_mm_s: 15.0        # Fast travel between squares
```

<!-- USER_ATTENTION: Calibrate these positions with your actual board placement -->

---

## Launch File Parameters

### full_system_launch.py

Parameters that can be passed at launch time:

```bash
ros2 launch src/launch/full_system_launch.py \
    use_sim:=false \
    debug:=false \
    log_level:=info
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `use_sim` | bool | false | Use simulation instead of real hardware |
| `debug` | bool | false | Enable debug logging |
| `log_level` | string | "info" | Log level (debug/info/warn/error) |
| `time_control` | int | 600 | Time control in seconds |

---

## Environment Variables

| Variable | Purpose | Default |
|----------|---------|---------|
| `GPIOZERO_PIN_FACTORY` | GPIO library backend | "rpigpio" |
| `STOCKFISH_PATH` | Path to Stockfish binary | "/usr/bin/stockfish" |
| `CHESS_LOG_LEVEL` | Application log level | "INFO" |

Set in your `.bashrc` or launch file:
```bash
export STOCKFISH_PATH="/usr/games/stockfish"
```

---

## Parameter Validation

Key constraints to verify:

| Parameter | Valid Range | Notes |
|-----------|-------------|-------|
| BCM pin numbers | 0-27 | Not all pins are GPIO |
| step_delay | ≥0.001 | Faster may miss steps |
| engage_pwm | 0-100 | Percentage |
| debounce_ms | 10-100 | Too low = false triggers |
| steps_per_mm | ~40-60 | Depends on pulley/belt |

---

## Updating Configuration

1. Edit the YAML file
2. Rebuild the package (if file location changed)
3. Re-source the workspace
4. Re-launch nodes

```bash
# After editing config
colcon build --packages-select <package>
source install/setup.bash
ros2 launch src/launch/full_system_launch.py
```

---

*See [pinout.md](../hardware/pinout.md) for GPIO details.*
