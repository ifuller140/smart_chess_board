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
- [ ] Python 3 installed with `RPi.GPIO`
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

All hardware tests are consolidated in a single script. Run it directly on the Raspberry Pi:

```bash
cd ~/dev/smart_chess_board/code

# Start the test suite
python3 hardware_test.py
```

### Available Tests

| Option | Test | Description |
|--------|------|-------------|
| 1 | Stepper Motors | Test Motor A/B movement + CoreXY axes |
| 2 | Speed Range | Test motors at 20%, 40%, 60%, 80%, 100% |
| 3 | Servos | Sweep clock and magnet servos |
| 4 | Limit Switches (Monitor) | 10-second monitoring mode |
| 5 | Limit Switches (Interactive) | Guided verification with clock confirmation |
| 6 | Clock Displays | Display '8888' on both TM1637 displays |
| 7 | Set Speed | Adjust motor speed percentage |
| 8 | Enable/Disable | Test motor holding torque |
| 9 | Run All | Execute full test sequence |
| 10 | Interactive Stepper | Launch stepper_interactive_test.py |
| 11 | Manual Gantry Control | Arrow key control with direction chart |

### Manual Gantry Control (Option 11)

Interactive arrow-key control from the player's perspective (sitting at white's side).

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

### 1. Test Stepper Motors

```bash
cd ~/smart_chess_ws/src/smart_chess_board/code

# Test Motor A (should rotate one direction then back)
python3 square.py
```

Expected: Motor A rotates ~90 degrees and returns.

### 2. Test Servo Motor

```bash
cd ~/smart_chess_ws/src/smart_chess_board/code

# Test servo sweep
python3 ServoTestController.py
```

Expected: Servo moves from 0° to 180° and back.

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

After all components pass:
1. Run individual ROS 2 nodes
2. Test full system launch
