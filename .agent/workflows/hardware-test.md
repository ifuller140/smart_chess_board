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

## Automated Test Suite (Recommended)

Run the integrated hardware test suite:

```bash
cd ~/smart_chess_ws/src/smart_chess_board

# List available tests
python3 -m chess_hw_interface.testing.test_runner --list

# Run all tests
python3 -m chess_hw_interface.testing.test_runner --all

# Run specific test
python3 -m chess_hw_interface.testing.test_runner --test gantry
python3 -m chess_hw_interface.testing.test_runner --test servo
python3 -m chess_hw_interface.testing.test_runner --test camera
python3 -m chess_hw_interface.testing.test_runner --test magnet
python3 -m chess_hw_interface.testing.test_runner --test clock
```

The test runner will:
- Automatically initialize GPIO pins
- Display test status on the 7-segment display (if connected)
- Wait for clock button input between test steps

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
