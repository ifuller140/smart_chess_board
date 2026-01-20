# Component Specifications

> **Detailed specifications for all hardware components.**

## Bill of Materials

<!-- USER_ATTENTION: Update quantities, prices, and supplier links as you source components -->

| Component | Quantity | Est. Price | Supplier | Part Number | Status |
|-----------|----------|------------|----------|-------------|--------|
| Raspberry Pi 4B (4GB) | 1 | $55 | Various | RPI4-MODBP-4GB | ✅ |
| 28BYJ-48 Stepper Motor | 2 | $5 ea | Amazon/AliExpress | 28BYJ-48 | ⬜ |
| ULN2003 Driver Board | 2 | $2 ea | Amazon/AliExpress | ULN2003A | ⬜ |
| SG90 Micro Servo (Z-axis) | 1 | $3 | Amazon/AliExpress | SG90 | ⬜ |
| SG90 Micro Servo (Clock) | 1 | $3 | Amazon/AliExpress | SG90 | ⬜ |
| Electromagnet 5V | 1 | $8 | Amazon | P20/15 | ⬜ |
| RPi Camera Module v2 | 1 | $25 | Various | RPI-CAM-V2 | ⬜ |
| Micro Limit Switch | 3 | $1 ea | Amazon | KW12-3 | ⬜ |
| GT2 Timing Belt | 2m | $5 | Amazon | GT2-6mm | ⬜ |
| GT2 Pulley 20T | 2 | $3 ea | Amazon | GT2-20T-5mm | ⬜ |
| Linear Rails/Rods | 4 | $10 | Amazon | 8mm smooth rod | ⬜ |
| Linear Bearings | 4 | $5 | Amazon | LM8UU | ⬜ |
| 5V 3A Power Supply | 1 | $10 | Amazon | | ⬜ |
| Dupont Wires | 40pcs | $5 | Amazon | F-F jumpers | ⬜ |

**Estimated Total**: ~$150-200

---

## Raspberry Pi 4B

### Specifications
| Parameter | Value |
|-----------|-------|
| Model | Raspberry Pi 4 Model B |
| RAM | 4GB (minimum recommended) |
| CPU | Broadcom BCM2711, Quad-core Cortex-A72 @ 1.5GHz |
| GPIO | 40-pin header, 26 GPIO pins |
| Power | 5V/3A via USB-C |
| OS | Ubuntu 22.04 Server (64-bit) |

### GPIO Capabilities
- **Total GPIO pins**: 26 (usable)
- **PWM channels**: 2 hardware PWM (GPIO 12, 13, 18, 19)
- **I2C buses**: 2 (GPIO 2/3, GPIO 0/1)
- **SPI buses**: 2
- **UART**: 1 (GPIO 14/15)

> [!WARNING]
> GPIO pins are **3.3V logic**. Do NOT connect 5V signals directly!

---

## 28BYJ-48 Stepper Motor

### Specifications
| Parameter | Value |
|-----------|-------|
| Type | Unipolar stepper motor |
| Voltage | 5V DC |
| Phases | 4 |
| Step Angle | 5.625° / 64 (with gearbox) |
| Gear Ratio | 1:64 |
| Steps/Revolution | 2048 (half-step) / 4096 (full-step) |
| Holding Torque | ~3 N·cm |
| Current | ~240mA per phase |
| Max Speed | ~15 RPM (reliable) |

### Step Sequences

**Half-Step Sequence (Recommended - 2048 steps/rev)**:
```
Step  IN1  IN2  IN3  IN4
  1    1    0    0    0
  2    1    1    0    0
  3    0    1    0    0
  4    0    1    1    0
  5    0    0    1    0
  6    0    0    1    1
  7    0    0    0    1
  8    1    0    0    1
```

**Full-Step Sequence (4096 steps/rev)**:
```
Step  IN1  IN2  IN3  IN4
  1    1    1    0    0
  2    0    1    1    0
  3    0    0    1    1
  4    1    0    0    1
```

<!-- USER_ATTENTION: Verify which stepping mode works best for your application -->

### Speed Calculations
- Minimum step delay: ~0.001s (1ms)
- Maximum practical speed: ~15 RPM
- With 10mm pulley: ~0.31 mm/step, ~7.8 mm/s max

---

## ULN2003A Driver Board

### Specifications
| Parameter | Value |
|-----------|-------|
| Chip | ULN2003APG |
| Channels | 7 Darlington pairs |
| Max Voltage | 50V |
| Max Current | 500mA per channel |
| Input Logic | 3.3V or 5V compatible |
| Flyback Diodes | Built-in |

### Pinout
| Board Pin | Function | Connect To |
|-----------|----------|------------|
| IN1 | Motor Phase A | GPIO (BCM) |
| IN2 | Motor Phase B | GPIO (BCM) |
| IN3 | Motor Phase C | GPIO (BCM) |
| IN4 | Motor Phase D | GPIO (BCM) |
| VCC | Motor Power | 5V (separate supply) |
| GND | Ground | Common ground |

### LED Indicators
- 4 LEDs show which phase is active
- Useful for debugging step sequences

---

## SG90 Micro Servo

### Specifications
| Parameter | Value |
|-----------|-------|
| Operating Voltage | 4.8V - 6V |
| Stall Torque | 1.8 kg·cm @ 4.8V |
| Operating Speed | 0.1 sec/60° @ 4.8V |
| Rotation Range | 180° |
| Control Signal | PWM (50Hz) |
| Pulse Width | 500µs - 2400µs |
| Weight | 9g |

### PWM Control
| Position | Pulse Width | Duty Cycle @ 50Hz |
|----------|-------------|-------------------|
| 0° (min) | 500µs | 2.5% |
| 90° (center) | 1500µs | 7.5% |
| 180° (max) | 2400µs | 12% |

### Wiring
| Wire Color | Function |
|------------|----------|
| Brown | Ground |
| Red | VCC (5V) |
| Orange | Signal (PWM) |

<!-- USER_ATTENTION: Determine optimal engage/release positions for your Z-axis mechanism -->

---

## Clock Servo (SG90)

> **NEW**: Second servo mounted under the chess clock to "hit" the clock button after computer's move.

### Purpose
After the computer completes its move, the clock servo actuates to press the clock button, switching the timer to the human player.

### Mounting Position
```
┌─────────────────────────────────────┐
│          CHESS CLOCK                │
│   ┌─────────────┬─────────────┐     │
│   │   WHITE     │   BLACK     │     │
│   │   05:00     │   05:00     │     │
│   └──────┬──────┴──────┬──────┘     │
│          │             │            │
│      ┌───┴───┐     ┌───┴───┐        │
│      │ BUTTON│     │ BUTTON│        │
│      └───┬───┘     └───┬───┘        │
│          │             │            │
│      ┌───┴─────────────┴───┐        │
│      │   CLOCK SERVO       │← Hits button
│      │   (SG90)            │        │
│      └─────────────────────┘        │
└─────────────────────────────────────┘
```

### Specifications
| Parameter | Value |
|-----------|-------|
| Model | SG90 (same as Z-axis) |
| Operating Voltage | 4.8V - 6V |
| Control Signal | PWM (50Hz) |
| Rest Position | Servo horn away from button |
| Hit Position | Servo horn presses button |

<!-- USER_ATTENTION: Define which button (white/black) the servo hits and calibrate PWM values -->

### Configuration
```yaml
clock_servo_node:
  ros__parameters:
    clock_servo_pin: 16          # BCM pin (hardware PWM or software PWM)
    rest_pwm: 2.5                # Duty cycle for rest position
    hit_pwm: 7.5                 # Duty cycle for button press
    hit_duration: 0.3            # How long to hold hit position (seconds)
```

---

## Electromagnet

### Specifications (Typical P20/15 5V)
<!-- USER_ATTENTION: Update with actual electromagnet specifications -->

| Parameter | Value |
|-----------|-------|
| Model | P20/15 (or similar) |
| Voltage | 5V DC |
| Current Draw | ~400mA |
| Holding Force | 2.5 kg |
| Diameter | 20mm |
| Height | 15mm |

### Control
- Use a transistor/MOSFET to switch (GPIO cannot source 400mA)
- Or integrate with servo power circuit

> [!CAUTION]
> Electromagnet draws significant current. Ensure power supply can handle it!

---

## Camera Module

### Raspberry Pi Camera v2 Specifications
| Parameter | Value |
|-----------|-------|
| Sensor | Sony IMX219 |
| Resolution | 8 megapixels |
| Video | 1080p @ 30fps, 720p @ 60fps |
| Still | 3280 × 2464 |
| FOV | 62.2° horizontal |
| Interface | CSI (ribbon cable) |

### Alternative: USB Webcam
- Any V4L2-compatible USB camera
- Minimum 720p resolution recommended
- Check `/dev/video0` for device

### Mounting Position

**Actual Camera Setup**:
| Parameter | Value |
|-----------|-------|
| Horizontal offset | 2 inches (~50mm) behind board |
| Height above board | 7 inches (~178mm) |
| Tilt angle | 45 degrees down toward board |
| Field of view | Covers all 64 squares |

```
Side View:
                                    ┌─────┐
                                    │ CAM │
                                    └──┬──┘
                                       │╲  45°
                                       │ ╲
          7 inches                     │  ╲
                                       │   ╲
                                       │    ╲ (view direction)
    ─────────────────────────────┬─────┴─────────────────────
                                 │
          BOARD                  │ 2 inches (behind)
    ══════════════════════════════════════════════════════
```

> [!IMPORTANT]
> Camera is NOT directly above the board. Perspective correction is required.
> See [docs/features/vision-calibration.md](../features/vision-calibration.md) for calibration procedure.

---

## Limit Switches

### Specifications (KW12-3 or similar)
| Parameter | Value |
|-----------|-------|
| Type | Micro switch with lever |
| Voltage Rating | 125V AC / 250V AC |
| Current Rating | 5A |
| Configuration | NO (Normally Open) + NC (Normally Closed) |
| Actuation Force | ~50g |

### Wiring (Using Normally Open)
```
Switch          Pi GPIO
  COM ─────────── GND
  NO  ─────────── GPIO pin (with internal pull-up enabled)
```

When switch is pressed: GPIO reads LOW
When switch is released: GPIO reads HIGH (pulled up)

### Positions
| Switch | Location | Purpose |
|--------|----------|---------|
| X-MIN | Left edge of X travel | X-axis home position |
| Y-MIN | Front edge of Y travel | Y-axis home position |
| CLOCK | Near chess clock | Detect player move complete |

---

## Timing Belt & Pulleys

### GT2 Belt
| Parameter | Value |
|-----------|-------|
| Type | GT2 (2mm pitch) |
| Width | 6mm |
| Material | Rubber with fiberglass core |

### GT2 Pulley (20 Teeth)
| Parameter | Value |
|-----------|-------|
| Teeth | 20 |
| Pitch Diameter | 12.73mm |
| Bore | 5mm (for motor shaft) |
| Belt Width | 6mm |

### Motion Calculations
```
Circumference = π × 12.73mm ≈ 40mm
Steps per revolution = 2048 (half-step)
Steps per mm = 2048 / 40 ≈ 51.2 steps/mm
```

<!-- USER_ATTENTION: Calibrate steps/mm with actual hardware measurement -->

---

*See [pinout.md](pinout.md) for GPIO assignments.*
