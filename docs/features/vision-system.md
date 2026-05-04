# Vision System

> **Camera calibration, perception stack, and live web interfaces for board state detection.**

## Overview

The vision pipeline converts raw camera frames into a FEN string representing the current board state. It runs entirely on the Raspberry Pi and exposes two browser-accessible web interfaces.

```
┌─────────────────┐
│  Raw Camera     │
│  Image (angled) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Undistort       │  ← Apply camera_matrix + distortion_coeffs
│ (lens correct)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Detect Corners  │  ← Find 4 board corners (auto or manual)
│ (a1, a8, h8, h1)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Perspective     │  ← Apply homography matrix
│ Transform       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Rectified       │
│ Top-Down View   │  ← Perfect 8×8 grid
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Piece Detection │  ← Color thresholding per square
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ FEN Output      │  ← Published on /perception/board_state
└─────────────────┘
```

---

## ROS Perception Stack

### Nodes

| Node | Topic/Service | Purpose |
|------|--------------|---------|
| `camera_node` | pub `/camera/image_raw` | Captures frames (PiCamera2 or V4L2) |
| `board_detector_node` | pub `/perception/board_debug` | Finds board corners, computes warp |
| `piece_detector_node` | pub `/perception/board_state` | Color-thresholds squares → FEN string |

### Launching the Stack

```bash
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash
source install/setup.bash

# Default launch (V4L2/USB camera)
ros2 launch chess_perception perception_launch.py

# With Pi CSI camera
ros2 launch chess_perception perception_launch.py use_picamera2:=True

# With pre-computed calibration
ros2 launch chess_perception perception_launch.py \
    use_picamera2:=True \
    calibration_file:=src/chess_perception/config/calibration.npz
```

Wait for: `Camera node ready (backend=picamera2, 1280x720 @ 5.0fps)`

### Key Topics

```bash
# Verify stack is publishing
ros2 topic list | grep perception
ros2 topic hz /perception/board_state     # should be ~5 Hz
ros2 topic hz /camera/image_raw           # should be ~5 Hz

# Echo live FEN
ros2 topic echo /perception/board_state --field fen

# Capture empty-board reference (do BEFORE placing pieces)
ros2 service call /perception/capture_reference std_srvs/srv/Trigger {}
```

---

## Web Interfaces

### FEN Visualizer — Port 5000

A Flask web app that subscribes to `/perception/board_state` and renders a live chess board in the browser.

**Start the visualizer (requires perception stack running):**

```bash
ssh smart-chess-pi
cd ~/dev/smart_chess_board
source install/setup.bash
python3 src/chess_perception/scripts/fen_visualizer.py
```

Output:
```
✓ ROS subscriber started
  Local URL  : http://localhost:5000
  Network URL: http://192.168.1.149:5000
```

**Open in browser on your Mac:**
```
http://192.168.1.149:5000
```

**Offline mode (no ROS/camera needed):**

```bash
# Starting position
python3 src/chess_perception/scripts/fen_visualizer.py --no-ros

# Specific FEN
python3 src/chess_perception/scripts/fen_visualizer.py --no-ros \
    --fen "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"

# Inject FEN via curl (from another terminal)
curl -X POST http://localhost:5000/api/fen \
     -H "Content-Type: application/json" \
     -d '{"fen": "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"}'
```

**Status indicators:**

| Indicator | Meaning |
|-----------|---------|
| Green dot "Live ✓" | FEN received in last 3 seconds |
| Yellow "Stale (Xs)" | No update 3–10s (board not detected?) |
| Red "No data" | Nothing received in 10+ seconds |
| `Valid` green pill | FEN parses as a legal position |
| `Invalid` red pill | FEN string is malformed |

**Diagnostics from the visualizer:**

| What you see | Likely cause |
|-------------|-------------|
| Status: "No data" | Perception stack not running or topic name wrong |
| FEN Invalid (red pill) | `piece_detector_node` producing garbled FEN |
| Board orientation flipped | Corner ordering wrong in `board_detector` |
| Pieces on wrong squares | Board warp size mismatch (400 vs 480px) |

---

### MJPEG Camera Stream — Port 8080

A raw MJPEG stream of the camera feed. Useful for verifying camera position and focus before running the perception stack.

**Run the stream server:**

```bash
ssh smart-chess-pi
python3 ~/dev/smart_chess_board/code/camera_stream_server.py
```

Output:
```
Open in browser: http://smart-chess-pi:8080/
Stream URL:      http://smart-chess-pi:8080/stream
Still capture:   http://smart-chess-pi:8080/capture
```

**Open in browser on your Mac:**
```
http://smart-chess-pi:8080/
```

Supports PiCamera2 (CSI) automatically, falls back to OpenCV (USB/V4L2). No ROS required.

---

## Camera Calibration

### Camera Position

| Parameter | Value |
|-----------|-------|
| Horizontal offset (behind board) | 2 inches (~50mm) |
| Height above board | 7 inches (~178mm) |
| Tilt angle (from horizontal) | 45 degrees |

### Step 1: Capture Checkerboard Images

```bash
# On Pi — captures to /tmp/chess_camera_test/calibration/
cd ~/dev/smart_chess_board
python3 code/capture_calibration_images.py
```

Move the printed checkerboard (9×6 inner corners, 25mm squares) to 10–15 different positions/angles and press Enter to capture each.

### Step 2: Intrinsic Calibration (Lens Distortion)

```bash
python3 src/chess_perception/scripts/calibrate_camera.py \
    --images /tmp/chess_camera_test/calibration \
    -o src/chess_perception/config/calibration.npz
```

Outputs `calibration.npz` with the camera matrix and distortion coefficients.

### Step 3: Board Corner Calibration

The `board_detector_node` handles this automatically at startup. If auto-detection fails, it falls back to manual corner selection (click the 4 corners in the visualizer).

```bash
# Re-run corner detection after camera adjustment
ros2 service call /perception/recalibrate_corners std_srvs/srv/Trigger {}
```

### Step 4: Verify

Run the FEN visualizer (port 5000) and verify:
1. Board renders right-side-up (a1 bottom-left)
2. Grid overlay aligns with physical squares
3. Starting position FEN matches `rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1`

### Calibration Output Files

```
src/chess_perception/config/
├── calibration.npz            # Intrinsic matrix + distortion
├── homography_matrix.npy      # Perspective transform
└── calibration_config.yaml    # Metadata + parameters
```

---

## Standalone Scripts

Located in `src/chess_perception/scripts/`:

| Script | Purpose |
|--------|---------|
| `calibrate_camera.py` | Compute intrinsic calibration from checkerboard images |
| `test_detection.py` | Step-by-step pipeline verification on image or live feed |
| `fen_visualizer.py` | FEN web visualizer (port 5000) |

Located in `code/`:

| Script | Purpose |
|--------|---------|
| `capture_calibration_images.py` | Capture checkerboard images for calibration |
| `camera_stream_server.py` | Raw MJPEG stream server (port 8080) |
| `live_camera_preview.py` | Quick camera sanity check |
| `probe_camera.py` | Detect available cameras |

### test_detection.py Usage

```bash
cd ~/dev/smart_chess_board

# Test board detection on a captured image
python3 src/chess_perception/scripts/test_detection.py --image test.jpg --step board

# Test piece bounds
python3 src/chess_perception/scripts/test_detection.py --image test.jpg --step pieces

# Live full pipeline test
python3 src/chess_perception/scripts/test_detection.py --camera 0 --step full
```

---

## Hardware Test Aliases

Vision-specific tests via the hardware test runner:

```bash
cd ~/dev/smart_chess_board
source /opt/ros/humble/setup.bash && source install/setup.bash

python3 -m chess_hw_interface.testing.test_runner --test vision_corners
python3 -m chess_hw_interface.testing.test_runner --test vision_board
python3 -m chess_hw_interface.testing.test_runner --test vision_pieces
python3 -m chess_hw_interface.testing.test_runner --test vision_squares
python3 -m chess_hw_interface.testing.test_runner --test vision_fen
```

Or use `run_hw_test.sh` from the project root:

```bash
./run_hw_test.sh --test vision_fen
```

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|---------|
| Camera black / no frames | Camera not enabled | Run `sudo raspi-config` → enable camera interface |
| `picamera2` import error | Library not installed | `sudo apt install python3-picamera2 libcamera-ipa` |
| Board warp skewed | Wrong corner order | Re-run corner calibration |
| Uneven lighting / shadows | Ambient light | Add diffuse overhead light source |
| Grid misaligned | Camera moved | Re-calibrate homography |
| Can't see all 64 squares | Camera too close | Increase height or use wider lens |
| FEN always starting pos | `piece_detector` fallback | Check `_build_fen` reference logic |
| Port 5000 refused | Flask not installed | `pip3 install flask` |
| Port 8080 refused | `camera_stream_server.py` not running | Start stream server manually |

---

## Re-Calibration Triggers

Re-run calibration whenever:
- Camera has been physically moved or adjusted
- Board position has shifted
- Lighting conditions have significantly changed
- Grid detection accuracy degrades noticeably
