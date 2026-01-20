---
description: How to test individual hardware components
---

# Hardware Testing Workflow

Step-by-step guide for validating hardware components before ROS 2 integration.

## Prerequisites

- [ ] SSH access to Raspberry Pi
- [ ] All components wired according to `docs/hardware/wiring.md`
- [ ] Python 3 installed with `RPi.GPIO`

## 1. Test GPIO Access

```bash
# Check GPIO permissions
groups  # Should include 'gpio'

# Test GPIO library
python3 -c "import RPi.GPIO; print('GPIO OK')"
```

## 2. Test Stepper Motors

```bash
cd ~/smart_chess_ws/src/smart_chess_board/code

# Test Motor A (should rotate one direction then back)
python3 square.py
```

Expected: Motor A rotates ~90 degrees and returns.

<!-- USER_ATTENTION: Update script parameters if using different pins -->

## 3. Test Servo Motor

```bash
cd ~/smart_chess_ws/src/smart_chess_board/code

# Test servo sweep
python3 ServoTestController.py
```

Expected: Servo moves from 0° to 180° and back.

## 4. Test Limit Switches

```bash
# Quick test - monitor GPIO state
python3 -c "
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(6, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(13, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(19, GPIO.IN, pull_up_down=GPIO.PUD_UP)
print('X-MIN (6):', 'PRESSED' if GPIO.input(6)==0 else 'OPEN')
print('Y-MIN (13):', 'PRESSED' if GPIO.input(13)==0 else 'OPEN')
print('CLOCK (19):', 'PRESSED' if GPIO.input(19)==0 else 'OPEN')
GPIO.cleanup()
"
```

Press each switch and re-run to verify detection.

## 5. Test Camera

```bash
# For CSI camera
libcamera-still -o test.jpg

# For USB camera
fswebcam -r 640x480 test.jpg

# View image (copy to local machine if headless)
scp pi@raspberrypi:~/test.jpg .
```

Expected: Clear image of the chess board area.

## 6. Test Electromagnet

```bash
# WARNING: Ensure magnet is not near sensitive electronics
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

## Troubleshooting

| Issue | Check |
|-------|-------|
| "Permission denied" | Run `sudo usermod -a -G gpio $USER`, re-login |
| No motor movement | Verify 5V power supply connected |
| Camera not found | Check `vcgencmd get_camera` or `lsusb` |
| Servo jitters | Add 470µF capacitor to power |

## Next Steps

After all components pass:
1. Run individual ROS 2 nodes
2. Test full system launch
