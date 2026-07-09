# Vision System

> **Camera calibration, perception stack, and live web interfaces for board state detection.**

## Overview

The vision pipeline does **not** reconstruct a full board position from color/piece-type classification. Instead it detects the board's 4 corners, then — on demand — diffs a "pre-move reference" warp against a fresh capture to report *which squares changed* between the two. `game_manager_node` combines that changed-squares list with the set of legal moves from its own `python-chess` board model to figure out which move was actually played, and that model (not vision) is the authoritative source of the game's FEN. This keeps vision's job simple (change detection) and pushes all chess-rules knowledge into `chess_logic`, where it already has to live anyway.

It runs entirely on the Raspberry Pi and exposes two browser-accessible web interfaces (the FEN visualizer, and a raw MJPEG stream).

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
│ Detect Corners  │  ← Find 4 board corners (TL/TR/BR/BL), anchored to the
│                 │    previous frame's labeling to avoid flip-flopping
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
│ Per-square LAB  │  ← Diff current warp against the pre-move reference warp;
│ diff vs. ref    │    squares above diff_threshold are "changed"
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Changed Squares │  ← Published on /perception/changed_squares (e.g. "e2,e4").
│                 │    game_manager_node matches this against legal moves —
└─────────────────┘    FEN itself comes from its board model, not vision.
```

---

## ROS Perception Stack

### Nodes

| Node | Topic/Service | Purpose |
|------|--------------|---------|
| `camera_node` | pub `/camera/image_raw`, `/camera/image_raw/compressed` | Captures frames. Fallback backend (`use_camera_ros:=False`); the default is the external `camera_ros` package, launched separately — see below. |
| `board_detector_node` | pub `/perception/board_geometry`, `/perception/board_debug` | Finds board corners, publishes their pixel coordinates |
| `piece_detector_node` | pub `/perception/changed_squares`, srv `/perception/capture_premove` | Frame-diff change detection against a pre-move reference |

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
ros2 topic hz /perception/board_geometry  # should be ~2 Hz (detection_hz)
ros2 topic hz /camera/image_raw           # should be ~5 Hz

# Echo detected corners (header.stamp = time of last FRESH detection, not
# this publish — compare against now() to check staleness)
ros2 topic echo /perception/board_geometry

# Capture the current board as the pre-move reference (call before a player
# moves; game_manager_node normally does this automatically each turn)
ros2 service call /perception/capture_premove std_srvs/srv/Trigger {}

# After a move + a fresh /camera/capture, watch what changed
ros2 topic echo /perception/changed_squares
```

---

## Web Interfaces

### FEN Visualizer — Port 5000

A Flask web app that subscribes to `/game_manager/board_fen` (the authoritative FEN from `game_manager_node`'s board model) plus `/perception/changed_squares`, `/perception/square_scores`, and `/perception/reference_status` for vision debug context, and renders a live chess board in the browser.

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
| Yellow "Stale (Xs)" | No update 3–10s (no move recently, or game_manager_node down) |
| Red "No data" | Nothing received in 10+ seconds |
| `Valid` green pill | FEN parses as a legal position |
| `Invalid` red pill | FEN string is malformed |

**Diagnostics from the visualizer:**

| What you see | Likely cause |
|-------------|-------------|
| Status: "No data" | `game_manager_node` not running, or `/game_manager/board_fen` never published |
| No changed-squares after a move | `piece_detector_node` down, no pre-move reference captured, or `diff_threshold` too high |
| Board orientation flipped | Corner ordering wrong in `board_detector_node` — check `/perception/board_geometry` |
| False changed-squares every tick | Camera vibration/lighting drift — tune `global_shift_compensation` |

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

`board_detector_node` detects corners continuously (`detection_hz`, default 2Hz) — there's no separate manual calibration step or service call. If the camera or board moves, corner detection just picks up the new geometry on its own next successful detection (anchored to the previous detection's labeling — see the Overview diagram — so labels stay stable across the transition). If detection stops finding the board at all (occlusion, bad lighting), check `/perception/board_debug` for what it's seeing.

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
python3 -m chess_hw_interface.testing.test_runner --test vision_squares
python3 -m chess_hw_interface.testing.test_runner --category vision --subtest full
```

Or use `run_hw_test.sh` from the project root:

```bash
./run_hw_test.sh --test vision_board
```

(`vision_pieces`/`vision_fen` — color piece-classification and full-board FEN
rendering — were removed along with the pipeline stage they tested; see the
Overview above for why vision doesn't do that anymore.)

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|---------|
| Camera black / no frames | Camera not enabled | Run `sudo raspi-config` → enable camera interface |
| `picamera2` import error | Library not installed | `sudo apt install python3-picamera2 libcamera-ipa` |
| Board warp skewed | Wrong corner order | Re-run corner calibration |
| Uneven lighting / shadows | Ambient light | Add diffuse overhead light source |
| Grid misaligned | Camera moved | Corner detection re-anchors automatically on its next successful detection |
| Can't see all 64 squares | Camera too close | Increase height or use wider lens |
| No changed-squares ever reported | No pre-move reference captured yet | Call `/perception/capture_premove`, or check `/perception/reference_status` |
| Port 5000 refused | Flask not installed | `pip3 install flask` |
| Port 8080 refused | `camera_stream_server.py` not running | Start stream server manually |

---

## Re-Calibration Triggers

Re-run calibration whenever:
- Camera has been physically moved or adjusted
- Board position has shifted
- Lighting conditions have significantly changed
- Grid detection accuracy degrades noticeably
