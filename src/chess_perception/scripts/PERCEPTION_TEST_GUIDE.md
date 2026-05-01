# FEN Visualizer — Complete Run Guide

## What It Does

Runs a web server on the Pi that shows a live rendered chess board updating every
second. You open it in a browser on your Mac. No display needed on the Pi.

---

## One-Time Setup (run on Pi)

```bash
# 1. SSH into Pi
ssh pi@smart-chess-pi

# 2. Navigate to project
cd ~/smart_chess_board

# 3. Install Flask (python-chess is already installed for piece_detector_node)
pip3 install flask

# 4. Verify both packages work
python3 -c "import flask; import chess; print('OK')"
```

---

## Running — Three Modes

### MODE 1 — Full Perception Stack + Visualizer (normal use)

Open **3 SSH terminals** to the Pi simultaneously.

**Terminal 1: Build and source the workspace**
```bash
ssh pi@smart-chess-pi
cd ~/dev/smart_chess_board
colcon build --packages-select chess_perception
source install/setup.bash
```

**Terminal 2: Launch the perception stack**
```bash
ssh pi@smart-chess-pi
cd ~/dev/smart_chess_board
source install/setup.bash
ros2 launch chess_perception perception_launch.py use_picamera2:=True
```
Wait for: `Camera node ready (backend=picamera2, 1280x720 @ 5.0fps)`

**Terminal 3: Run the visualizer**
```bash
ssh pi@smart-chess-pi
cd ~/dev/smart_chess_board
source install/setup.bash
python3 src/chess_perception/scripts/fen_visualizer.py
```
You'll see:
```
✓ ROS subscriber started
============================================================
  Chess Perception Live Visualizer
============================================================
  Local URL  : http://localhost:5000
  Network URL: http://<this-machine-ip>:5000
```

**On your Mac — open in browser:**
```
http://192.168.1.149:5000
```
(replace XX with Pi's IP — get it with `hostname -I` on the Pi)

---

### MODE 2 — Offline Test (no ROS, no camera needed)

Use this to verify the visualizer works before the perception stack is ready,
or to paste in a specific FEN to see what it looks like.

**On the Pi or your Mac:**
```bash
# Start with starting position
python3 src/chess_perception/scripts/fen_visualizer.py --no-ros

# Start with your specific FEN
python3 src/chess_perception/scripts/fen_visualizer.py --no-ros \
    --fen "rnb2bnr/8/ppp2ppp/pppPPppp/2PpppP1/PPPpppPP/PPPPPPPP/RNBQKBNR w KQkq - 0 1"
```

Open browser at `http://localhost:5000`

**Inject new FEN via curl (from another terminal):**
```bash
curl -X POST http://localhost:5000/api/fen \
     -H "Content-Type: application/json" \
     -d '{"fen": "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq e3 0 1"}'
```

---

### MODE 3 — Without Custom BoardState Message (fallback)

If `chess_perception` isn't built yet (BUG-02 import issue not fixed), the
visualizer falls back to a plain String topic. You can test it like this:

**Terminal 2: Launch just the camera node**
```bash
ros2 run chess_perception camera_node --ros-args -p use_picamera2:=True
```

**Terminal 3: Run visualizer (it will use fallback topic)**
```bash
python3 src/chess_perception/scripts/fen_visualizer.py
```

**Terminal 4: Manually publish a FEN string**
```bash
ros2 topic pub /perception/board_state_fen std_msgs/msg/String \
    '{data: "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"}' --once
```

---

## Using the Browser Interface

### Layout
```
[Chess Board 480px]    [Status Panel]
                       [Current FEN]
                       [Diff vs Reference]
                       [Inject FEN (offline)]
```

### Accuracy Testing Workflow

**Step 1 — Place pieces in starting position**
- Point camera at board
- Wait for board to be detected (status dot turns green)

**Step 2 — Verify starting position is correct**
- Board should show all 32 pieces in starting positions
- Diff panel shows "✓ Matches reference exactly" (reference defaults to starting pos)

**Step 3 — Test a specific position**
- Move pieces physically on the board
- Wait 1–2 seconds for perception to update
- Browser shows what the camera sees

**Step 4 — Use the diff tool**
- Click "📌 Set Current as Reference" to snapshot current state
- Make a move on the physical board
- Diff panel shows exactly which squares changed:
  ```
  E2: ♙ P → □ empty
  E4: □ empty → ♙ P
  ```

**Step 5 — Check detection accuracy**
- Put a white piece on E4
- Check if board shows white piece on E4 (not black, not empty)
- If wrong → perception color detection bug (BUG-05 from audit)

---

## Status Indicators

| Indicator | Meaning |
|-----------|---------|
| 🟢 Green dot "Live ✓" | FEN received in last 3 seconds |
| 🟡 Yellow "Stale (Xs)" | No update for 3–10 seconds (board not detected?) |
| 🔴 Red "No data" | Nothing received in 10+ seconds |
| `Valid` green pill | FEN parses as legal chess position |
| `Invalid` red pill | FEN string is malformed (bug in detection) |

---

## Diagnosing Problems from the Visualizer

| What you see | Likely cause | Fix |
|-------------|-------------|-----|
| Status: "No data" | Perception stack not running, or topic name wrong | Check Terminal 2 for errors |
| FEN Invalid (red pill) | piece_detector_node producing garbled FEN | BUG-05 (threshold formula) |
| Board orientation flipped | Corner ordering wrong in board_detector | BUG-04 H/V inversion |
| White/black pieces swapped | RGB/BGR bug in camera_node | BUG-06 from audit |
| Pieces on wrong squares | Board warp size mismatch (400 vs 480) | ARCH-01 from audit |
| Ghost pieces on empty squares | Reference frame not captured | Run capture_reference service |
| Board shows starting pos always | piece_detector using auth FEN only | Check _build_fen fallback logic |

---

## Quick Diagnostic Commands

**Check if topics are publishing:**
```bash
ros2 topic list | grep perception
ros2 topic hz /perception/board_state        # should be ~5 Hz
ros2 topic hz /camera/image_raw              # should be ~5 Hz
```

**Echo the raw FEN from the topic:**
```bash
ros2 topic echo /perception/board_state --field fen
```

**Capture the empty board reference:**
```bash
# Do this BEFORE placing pieces (board must be empty)
ros2 service call /perception/capture_reference std_srvs/srv/Trigger {}
```

**Check the board debug image (saves to file):**
```bash
ros2 run image_transport republish raw raw \
    --ros-args -r in:=/perception/board_debug -r out:=/board_debug_out &
ros2 run image_saver image_saver --ros-args -r image:=/perception/board_debug
```
Or view directly with rqt if you have a display:
```bash
rqt_image_view /perception/board_debug
```

**Check camera is not publishing black frames:**
```bash
ros2 topic echo /camera/image_raw --once | grep "data:" | head -c 200
# Should NOT be all zeros
```

---

## Stopping Everything

```bash
# In each terminal: Ctrl+C
# Or kill all ROS processes:
pkill -f ros2
pkill -f fen_visualizer
```
