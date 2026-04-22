# Vision System Guide

This document outlines the architecture, nodes, and testing procedures for the smart chess board's camera and perception system.

## 1. ROS 2 Perception Stack

The main entry point for the perception sub-system is `perception_launch.py`. It starts the nodes necessary to read frames from the camera, identify the chess board, and isolate the pieces.

### Running the Stack

```bash
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Run default stack (V4L2 by default)
ros2 launch chess_perception perception_launch.py

# Optional Arguments
ros2 launch chess_perception perception_launch.py use_picamera2:=True calibration_file:=config/calibration.npz
```

### Perception Nodes
- **`camera_node`**: Connects to either an OpenCV `VideoCapture` camera or uses PiCamera2. Adjust FPS, Resolution, and Camera ID within the node parameters.
- **`board_detector_node`**: Reads camera frames and identifies the chessboard's 4 outermost corners using OpenCV primitives to find an inner checkerboard grid. Returns perspective mappings.
- **`piece_detector_node`**: Subscribes to warped top-down frames and finds chess pieces using simple color differentiation thresholds, publishing a standard FEN string representing the board state.

## 2. Hardware Testing Suite

The hardware test suite `test_runner.py` provides several test aliases specific to vision verification.

### Running Tests

To run the vision tests, utilize the test runner with the respective alias:

```bash
cd ~/dev/smart_chess_board
source /opt/ros/jazzy/setup.bash

# General vision test
python3 -m chess_hw_interface.testing.test_runner --test vision_<alias>
```

### Available Vision Aliases
- **`vision_corners`**: Validates whether the board's internal intersection corners are detected correctly.
- **`vision_board`**: Focuses singularly on finding the bounds of the chessboard and calculating its geometry.
- **`vision_pieces`**: Checks if pieces are recognized, utilizing configured color thresholds.
- **`vision_squares`**: Outlines all 64 squares visually on the screen for geometry alignment tests.
- **`vision_fen`**: Fully evaluates a position and displays the calculated FEN format visually.

## 3. Standalone Scripts

There are independent scripts located in `src/chess_perception/scripts/` to help verify functionality and generate configuration assets manually.

### Camera Calibration (`calibrate_camera.py`)
Generates an `.npz` intrinsic calibration matrix correcting the camera's lens distortion format. Provide a folder of captured image sequences containing standard checkerboards.

```bash
# Capture image to calibration folder, then run:
python3 scripts/calibrate_camera.py --images /tmp/chess_camera_test/calibration -o config/calibration.npz
```

### Pipeline Detection Flow (`test_detection.py`)
Useful to verify pieces step-by-step or check calibration impacts against a single captured image or live webcam. 

**Usage Examples:**
```bash
# Test just the board detection on an image
python3 scripts/test_detection.py --image test.jpg --step board

# Test piece bounds and limits
python3 scripts/test_detection.py --image test.jpg --step pieces

# Test Live Pipeline
python3 scripts/test_detection.py --camera 0 --step full
```
