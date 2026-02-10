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

**Direction Chart:**
| Arrow Key | Motor A (BL) | Motor B (TR) |
|-----------|--------------|--------------|
| → Right   | Clockwise    | Counter-CW   |
| ← Left    | Counter-CW   | Clockwise    |
| ↑ Up      | Clockwise    | Clockwise    |
| ↓ Down    | Counter-CW   | Counter-CW   |

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

### 5. Test Electromagnet

```bash
# WARNING: Keep magnet away from sensitive electronics
python3 -c "
import RPi.GPIO as GPIO
import time
MAGNET_PIN = 26  # Update with your pin
GPIO.setmode(GPIO.BCM)
GPIO.setup(MAGNET_PIN, GPIO.OUT)
print('Magnet ON for 2 seconds...')
GPIO.output(MAGNET_PIN, GPIO.HIGH)
time.sleep(2)
GPIO.output(MAGNET_PIN, GPIO.LOW)
print('Magnet OFF')
GPIO.cleanup()
"
```

Expected: Magnet engages (should attract steel).

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
