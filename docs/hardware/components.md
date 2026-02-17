# Component Specifications

> **Detailed specifications for all hardware components.**

## Bill of Materials

<!-- USER_ATTENTION: Update quantities, prices, and supplier links as you source components -->

| Component | Quantity | Est. Price | Supplier | Part Number | Status |
|-----------|----------|------------|----------|-------------|--------|
| Raspberry Pi 4B (4GB) | 1 | $55 | Various | RPI4-MODBP-4GB | ✅ |
| NEMA 11 Stepper Motor | 2 | $15 ea | StepperOnline | 11HS13-0404S | ✅ |
| A4988 Stepper Driver | 2 | $3 ea | Amazon/AliExpress | A4988 | ✅ |
| SG90 Micro Servo (Z-axis) | 1 | $3 | Amazon/AliExpress | SG90 | ⬜ |
| SG90 Micro Servo (Clock) | 1 | $3 | Amazon/AliExpress | SG90 | ⬜ |
| Electromagnet 5V | 1 | $8 | Amazon | P20/15 | ⬜ |
| RPi Camera Module v2 | 1 | $25 | Various | RPI-CAM-V2 | ⬜ |
| Micro Limit Switch | 3 | $1 ea | Amazon | KW12-3 | ⬜ |
| GT2 Timing Belt | 2m | $5 | Amazon | GT2-6mm | ⬜ |
| GT2 Pulley 20T | 2 | $3 ea | Amazon | GT2-20T-5mm | ⬜ |
| Linear Rails/Rods | 4 | $10 | Amazon | 8mm smooth rod | ⬜ |
| Linear Bearings | 4 | $5 | Amazon | LM8UU | ⬜ |
| 12V 2A Power Supply | 1 | $12 | Amazon | | ⬜ |
| 5V 3A Power Supply | 1 | $10 | Amazon | | ⬜ |
| Dupont Wires | 40pcs | $5 | Amazon | F-F jumpers | ⬜ |

**Estimated Total**: ~$180-230

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

## NEMA 11 Stepper Motor

### Specifications
| Parameter | Value |
|-----------|-------|
| Type | Bipolar stepper motor |
| Frame Size | NEMA 11 (28mm x 28mm) |
| Voltage | 12V DC (typical) |
| Phases | 2 |
| Step Angle | 1.8° |
| Steps/Revolution | 200 (full-step) |
| Holding Torque | ~6 N·cm |
| Current | ~0.4A per phase |
| Max Speed | ~1000+ RPM |

### Speed Calculations
- Full-step: 200 steps/revolution
- With microstepping (1/16): 3200 steps/revolution
- Minimum step pulse: 2µs (using 10µs for safety)
- Maximum practical speed: Very fast - use speed control (0-100%)

> [!NOTE]
> NEMA 11 motors are significantly faster than 28BYJ-48 motors.
> Use the speed control (0-100%) to find optimal operating speed.

---

## A4988 Stepper Driver

### Specifications
| Parameter | Value |
|-----------|-------|
| Chip | Allegro A4988 |
| Motor Type | Bipolar stepper |
| Max Voltage | 35V |
| Max Current | 2A per phase (with heatsink) |
| Logic Voltage | 3.3V - 5V compatible |
| Microstepping | 1, 1/2, 1/4, 1/8, 1/16 |
| Control Pins | 2 (STEP, DIR) |

### Pinout
| Pin | Function | Connect To |
|-----|----------|------------|
| STEP | Step pulse input | GPIO (BCM) |
| DIR | Direction input | GPIO (BCM) |
| VDD | Logic power | 3.3V from Pi |
| GND | Ground | Common ground |
| VMOT | Motor power | 12V supply |
| 1A, 1B | Motor coil 1 | Motor wires |
| 2A, 2B | Motor coil 2 | Motor wires |
| MS1, MS2, MS3 | Microstepping | Not connected (full-step mode) |
| ENABLE | Enable (active low) | GPIO 17 — software-controlled |
| SLEEP | Sleep (active low) | Bridged to RESET |
| RESET | Reset (active low) | Bridged to SLEEP |

### Control Method
```
To step the motor:
1. Set DIR pin LOW (forward) or HIGH (reverse)
   NOTE: DIR polarity is INVERTED for this CoreXY layout
   (LOW = positive/forward direction)
2. Pulse STEP pin: LOW → HIGH → LOW
3. Each rising edge = 1 step
4. Delay between pulses controls speed
```

### Current Limiting (Vref)

The A4988 potentiometer must be adjusted to limit motor current.

Formula: `Vref = I_max × 8 × R_sense`

For NEMA 11 (11HS13-0404S, 0.4A per phase) with typical R_sense = 0.1Ω:
```
Vref = 0.4 × 8 × 0.1 = 0.32V
```

Measure Vref between the potentiometer wiper and GND with a multimeter.

> [!CAUTION]
> **Vref too high** → overcurrent → motor/driver overheats
> **Vref too low** → insufficient torque → motor vibrates but doesn't step

### Microstepping Configuration
| MS1 | MS2 | MS3 | Resolution |
|-----|-----|-----|------------|
| LOW | LOW | LOW | Full step (200 steps/rev) |
| HIGH | LOW | LOW | 1/2 step (400 steps/rev) |
| LOW | HIGH | LOW | 1/4 step (800 steps/rev) |
| HIGH | HIGH | LOW | 1/8 step (1600 steps/rev) |
| HIGH | HIGH | HIGH | 1/16 step (3200 steps/rev) |

> [!NOTE]
> MS1/MS2/MS3 pins are not connected in current setup (full-step mode).
> Can be added later for smoother motion if needed.

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
    clock_servo_pin: 18          # BCM pin (hardware PWM or software PWM)
    rest_pulse_us: 500           # Pulse width for rest position (microseconds)
    hit_pulse_us: 1500           # Pulse width for button press (microseconds)
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

### Wiring (Using Normally Open — Active HIGH)
```
Switch          Pi GPIO
  COM ─────────── 5V
  NO  ─────────── GPIO pin (with internal pull-down enabled)
```

When switch is pressed: GPIO reads HIGH (5V through switch)
When switch is released: GPIO reads LOW (pulled down)

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
Steps per revolution = 200 (full-step, no microstepping)
Steps per mm = 200 / 40 = 5 steps/mm
```

<!-- USER_ATTENTION: Calibrate steps/mm with actual hardware measurement -->

---

*See [pinout.md](pinout.md) for GPIO assignments.*
