---
description: How to test individual hardware components
---

# Hardware Testing Workflow

Step-by-step guide for validating hardware components on Raspberry Pi.

> **IMPORTANT**: All hardware tests require a Raspberry Pi with GPIO and physical hardware connected. Tests will NOT work on development machines.

## Prerequisites

- [ ] Raspberry Pi running **Raspberry Pi OS** or **Ubuntu**
- [ ] SSH access to Raspberry Pi
- [ ] All components wired according to `docs/hardware/wiring.md`
- [ ] Python 3 installed with `RPi.GPIO` and `pigpio`
- [ ] pigpiod daemon running: `sudo systemctl start pigpiod`
- [ ] User added to `gpio` group

### Quick Setup Check

```bash
# On Raspberry Pi:

# 1. Check GPIO permissions
groups  # Should include 'gpio'

# 2. Add yourself to gpio group if not present
sudo usermod -a -G gpio $USER
# Then logout and login again

# 3. Test GPIO library
python3 -c "import RPi.GPIO; print('GPIO OK')"
```

---

## Hardware Test Suite

Primary entrypoint is the ROS package test runner:

```bash
cd ~/dev/smart_chess_board
python3 -m src.chess_hw_interface.chess_hw_interface.testing.test_runner --list
```

### Available Tests

| Command | Description |
|---------|-------------|
| `--category gantry --subtest limits` | Validate X/Y limit polarity |
| `--category gantry --subtest pulse` | Software pulse timing/jitter diagnostics |
| `--category gantry --subtest motor_a motor_b` | Single-motor direction/torque checks |
| `--category gantry --subtest lockstep` | Both motors sync — hardware validation |
| `--category gantry --subtest corexy` | Verify axis mapping (+X/-X/+Y/-Y) |
| `--category gantry --subtest diagonal_sync` | Diagonal motion — no X/Y wobble |
| `--category gantry --subtest speed_sweep` | Identify stall speed ranges |
| `--category gantry --subtest square_return` | Return-to-origin accuracy |
| `--category gantry --subtest repeatability` | Multi-loop stress repeatability |
| `--category gantry --subtest enable_hold` | Check holding torque behavior |
| `--category gantry --subtest manual` | Curses manual control/tuning (with diagonals) |
| `--category gantry --subtest full` | Guided full gantry workflow |

### Manual Gantry Control

Interactive arrow-key control from the player's perspective (sitting at white's side).

```bash
python3 -m src.chess_hw_interface.chess_hw_interface.testing.test_runner \
  --category gantry --subtest manual
```

**CoreXY Motor Layout:**
- Motor A: Bottom-left (BCM 27 dir, 22 step)
- Motor B: Top-right (BCM 6 dir, 5 step)

**Direction Chart (inverted DIR pins):**
| Arrow Key | Motor A (BL) | Motor B (TR) |
|-----------|--------------|--------------|
| → Right   | Counter-CW   | Clockwise    |
| ← Left    | Clockwise    | Counter-CW   |
| ↑ Up      | Counter-CW   | Counter-CW   |
| ↓ Down    | Clockwise    | Clockwise    |

> **NOTE**: All gantry tests now run through ROS.
> Start nodes first: `ros2 launch chess_hw_interface hw_interface_launch.py`

---

## Manual Component Tests

### Legacy Script Notes

Standalone scripts in `code/` are useful for ad-hoc checks but are not the canonical suite.
Prefer the test runner under `src/chess_hw_interface/chess_hw_interface/testing/`.

### 3. Test Limit Switches

Per pinout.md ground truth diagram:
- X-MIN = GPIO10 (Physical Pin 19)
- Y-MIN = GPIO9 (Physical Pin 21)
- Clock = GPIO15 (Physical Pin 10)

```bash
python3 -c "
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # X-MIN
GPIO.setup(9, GPIO.IN, pull_up_down=GPIO.PUD_UP)   # Y-MIN
GPIO.setup(15, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Clock
print('X-MIN (GPIO10):', 'PRESSED' if GPIO.input(10)==0 else 'OPEN')
print('Y-MIN (GPIO9):', 'PRESSED' if GPIO.input(9)==0 else 'OPEN')
print('CLOCK (GPIO15):', 'PRESSED' if GPIO.input(15)==0 else 'OPEN')
GPIO.cleanup()
"
```

Press each switch and re-run to verify detection.

### 4. Test Camera

```bash
# For CSI camera
libcamera-still -o test.jpg

# For USB camera
fswebcam -r 640x480 test.jpg

# View image (copy to local machine if headless)
scp pi@raspberrypi:~/test.jpg .
```

Expected: Clear image of the chess board area.

### 5. Test Magnet (Servo-Actuated, Permanent Magnet)

The magnet is permanent, not an electromagnet — there's no GPIO pin to toggle. "Testing the magnet" means testing the Z-axis servo that raises/lowers it, via the real hardware test suite:

```bash
./run_hw_test.sh --category magnet --subtest full
```

Or directly via ROS services (with `servo_node` running):

```bash
ros2 service call /servo/engage std_srvs/srv/Trigger {}   # lower — should attract a nearby piece
ros2 service call /servo/release std_srvs/srv/Trigger {}  # raise — should release it
```

Expected: Servo moves to the engage position and a piece placed underneath is gripped; release lifts clear.

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `ModuleNotFoundError: RPi.GPIO` | Run `pip3 install RPi.GPIO` |
| `RuntimeError: Not running on a RPi!` | You must run on actual Raspberry Pi hardware |
| `RuntimeError: No access to /dev/mem` | Run `sudo usermod -a -G gpio $USER` then re-login |
| `RuntimeError: You must setup() the GPIO channel first` | Pin not initialized - update test code |
| No motor movement | Verify 5V power supply connected |
| Camera not found | Check `vcgencmd get_camera` or `lsusb` |
| Servo jitters | Add 470µF capacitor to power rail |

---

## Next Steps

After gantry tests pass:
1. Run `gantry/full` once end-to-end.
2. Launch ROS nodes and test `/gantry/home` and `/gantry/move`.
3. Record calibration updates in docs and config.

---

## Phase 4 Checklist — Remaining Physical Validation (real motor movement)

Everything below is the last physical-only work tracked in `.agent/IMPLEMENTATION_PLAN.md`'s
Phase 4 — code-complete, never yet exercised against real motor motion. Software-only
fixes (Phases 1-3, 5-9) are done and live-verified; this is what's left.

**Every command below causes real motor movement. Run these yourself, or with the
user present live — never scripted/run autonomously by an agent.** (This project has
twice had an agent's production-touching action correctly blocked by the permission
system for exactly this reason — see `feedback_production_pi_workflow` — real motor
motion is a categorically different risk tier from software-only ROS restarts.)

- [ ] **First live-motor session.** `pigpiod` is confirmed running. Start with the
  guided full workflow, not an isolated subtest, so homing/limits/basic motion are
  all sanity-checked together before anything more targeted:
  ```bash
  ./run_hw_test.sh --category gantry --subtest full
  ```
- [ ] **Homing in isolation** (repeat a few times to check consistency):
  ```bash
  ./run_hw_test.sh --category gantry --subtest homing
  ```
- [ ] **Return-to-origin accuracy**:
  ```bash
  ./run_hw_test.sh --category gantry --subtest square_return
  ```
- [ ] **Repeatability under load** (multi-loop stress):
  ```bash
  ./run_hw_test.sh --category gantry --subtest repeatability
  ```
- [ ] **Physically measure and record** `x_max_mm`, `board_origin_x_mm`, `board_origin_y_mm`
  into `src/gantry_control/config/board_map.yaml` — Phase 1 already fixed the
  *code's* handling of these values (they were previously inconsistent across 3
  files); only the physical numbers themselves remain unmeasured.
- [ ] **Corner-routing BFS with a real obstruction** — place a piece so the direct
  path between two squares is physically blocked, then drive a move through it
  (`--category gantry --subtest square_nav` to the blocked square, or a full game
  move via chess_ui once Phase 4's other items are done) and confirm the BFS
  correctly routes around it instead of colliding.
- [ ] **E-stop during real motion** — newly available now that `pigpiod` is confirmed
  running (previously blocked by no `pigpiod` access): start a real gantry move, then
  trigger `/emergency_stop` mid-motion, and confirm it actually halts (Phase 3 fixed
  the blocking-executor bug that prevented this from working at all, but only
  verified it with synthetic feedback, never a real in-flight motor move).
- [ ] Once the above are solid, play one full real game start-to-finish through
  chess_ui as the final end-to-end confirmation.

**After Phase 4 is complete**, a fresh full top-to-bottom audit is worth doing —
real motor movement is where genuinely new failure modes are most likely to surface
(not more static code reading). See `.agent/IMPLEMENTATION_PLAN.md`'s Phase 9
write-up for the fuller reasoning.
