# GPIO Pinout Reference

> **Complete GPIO pin assignments for the Smart Chess Board.**
> 
> ⚠️ **This is the authoritative source for pin assignments. Update `pins.yaml` to match.**

## Pin Assignment Summary

<!-- USER_ATTENTION: ⚠️ CRITICAL - Update these pin numbers to match your actual wiring! -->
<!-- The values below are PLACEHOLDERS based on the existing pins.yaml template -->

| Function | BCM Pin | Physical Pin | Wire Color | Notes |
|----------|---------|--------------|------------|-------|
| **Stepper A (Motor A)** | | | | |
| IN1 | 17 | 11 | <!-- USER: color --> | Phase A |
| IN2 | 18 | 12 | <!-- USER: color --> | Phase B |
| IN3 | 27 | 13 | <!-- USER: color --> | Phase C |
| IN4 | 22 | 15 | <!-- USER: color --> | Phase D |
| **Stepper B (Motor B)** | | | | |
| IN1 | 23 | 16 | <!-- USER: color --> | Phase A |
| IN2 | 24 | 18 | <!-- USER: color --> | Phase B |
| IN3 | 25 | 22 | <!-- USER: color --> | Phase C |
| IN4 | 5 | 29 | <!-- USER: color --> | Phase D |
| **Z-Axis Servo (Gantry)** | | | | |
| PWM Signal | 12 | 32 | Orange | Hardware PWM |
| **Clock Servo (NEW)** | | | | |
| PWM Signal | 16 | 36 | <!-- USER: color --> | Hits clock button |
| **Limit Switches** | | | | |
| X-MIN | 6 | 31 | <!-- USER: color --> | Pull-up enabled |
| Y-MIN | 13 | 33 | <!-- USER: color --> | Pull-up enabled |
| Clock Hit | 19 | 35 | <!-- USER: color --> | Pull-up enabled |
| **Electromagnet** | | | | |
| Control | <!-- USER: assign --> | | <!-- USER: color --> | Via transistor |
| **Clock Display** | | | | |
| Display 1 | <!-- USER: assign --> | | | 7-segment |
| Display 2 | <!-- USER: assign --> | | | 7-segment |

---

## Raspberry Pi 40-Pin Header Diagram

```
                    3V3  (1)  (2)  5V
                  GPIO2  (3)  (4)  5V
                  GPIO3  (5)  (6)  GND
                  GPIO4  (7)  (8)  GPIO14 (UART TX)
                    GND  (9)  (10) GPIO15 (UART RX)
   Stepper A IN1  GPIO17 (11) (12) GPIO18  Stepper A IN2
   Stepper A IN3  GPIO27 (13) (14) GND
   Stepper A IN4  GPIO22 (15) (16) GPIO23  Stepper B IN1
                    3V3  (17) (18) GPIO24  Stepper B IN2
                 GPIO10  (19) (20) GND
                  GPIO9  (21) (22) GPIO25  Stepper B IN3
                 GPIO11  (23) (24) GPIO8
                    GND  (25) (26) GPIO7
                  GPIO0  (27) (28) GPIO1
   Stepper B IN4   GPIO5 (29) (30) GND
       X-MIN       GPIO6 (31) (32) GPIO12  Z-Axis Servo
       Y-MIN      GPIO13 (33) (34) GND
    Clock Hit     GPIO19 (35) (36) GPIO16  Clock Servo ← NEW
                 GPIO26  (37) (38) GPIO20
                    GND  (39) (40) GPIO21
```

<!-- USER_ATTENTION: Update the diagram above to reflect your actual assignments -->

---

## Pin Configuration Details

### Stepper Motor A (X+Y combined in CoreXY)
```yaml
stepper_driver:
  ros__parameters:
    motorA_pins: [17, 18, 27, 22]  # [IN1, IN2, IN3, IN4]
```

| Pin | BCM | Direction | Notes |
|-----|-----|-----------|-------|
| IN1 | 17 | OUTPUT | Blue wire on motor |
| IN2 | 18 | OUTPUT | Pink wire on motor |
| IN3 | 27 | OUTPUT | Yellow wire on motor |
| IN4 | 22 | OUTPUT | Orange wire on motor |

<!-- USER_ATTENTION: Verify motor wire colors match your specific 28BYJ-48 -->

### Stepper Motor B (X-Y combined in CoreXY)
```yaml
stepper_driver:
  ros__parameters:
    motorB_pins: [23, 24, 25, 5]  # [IN1, IN2, IN3, IN4]
```

| Pin | BCM | Direction | Notes |
|-----|-----|-----------|-------|
| IN1 | 23 | OUTPUT | Blue wire on motor |
| IN2 | 24 | OUTPUT | Pink wire on motor |
| IN3 | 25 | OUTPUT | Yellow wire on motor |
| IN4 | 5 | OUTPUT | Orange wire on motor |

### Servo Motor
```yaml
servo_node:
  ros__parameters:
    servo_pin: 12           # Hardware PWM capable
    engage_pwm: 2.5         # Down position (duty %)
    release_pwm: 7.5        # Up position (duty %)
    movement_time: 0.5      # Seconds to wait
```

<!-- USER_ATTENTION: Calibrate engage_pwm and release_pwm for your Z-axis mechanism -->

> [!NOTE]
> GPIO 12, 13, 18, 19 support hardware PWM. Using GPIO 12 for Z-axis servo.
> GPIO 16 used for clock servo (software PWM).

### Clock Servo (NEW)
```yaml
clock_servo_node:
  ros__parameters:
    clock_servo_pin: 16         # Hits clock button after computer move
    rest_pwm: 2.5               # Servo at rest (away from button)
    hit_pwm: 7.5                # Servo pressing button
    hit_duration: 0.3           # Seconds to hold button
```

<!-- USER_ATTENTION: Calibrate rest_pwm and hit_pwm for your clock button position -->

### Limit Switches
```yaml
limit_switch_node:
  ros__parameters:
    limit_switch_pins:
      x_min: 6
      y_min: 13
      clock_hit: 19
    debounce_ms: 20
```

| Switch | BCM | Pull | Active State |
|--------|-----|------|--------------|
| X-MIN | 6 | PULL_UP | LOW when pressed |
| Y-MIN | 13 | PULL_UP | LOW when pressed |
| Clock | 19 | PULL_UP | LOW when pressed |

### Clock Display (7-Segment)
```yaml
clock_display:
  ros__parameters:
    display1_pins: [0, 0, 0, 0]  # PLACEHOLDER - segment pins
    display2_pins: [0, 0, 0, 0]  # PLACEHOLDER - segment pins
```

<!-- USER_ATTENTION: Define 7-segment display wiring if using this feature -->

---

## Reserved/Unavailable Pins

These pins should **NOT** be used for general GPIO:

| BCM | Physical | Reason |
|-----|----------|--------|
| 0, 1 | 27, 28 | I2C ID EEPROM (reserved) |
| 2, 3 | 3, 5 | I2C1 (if using I2C devices) |
| 14, 15 | 8, 10 | UART (if using serial console) |
| 7, 8, 9, 10, 11 | Various | SPI (if using SPI devices) |

---

## Grounding Strategy

All grounds must be connected together:

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ Pi GND       │─────│ Motor PSU   │─────│ Servo/Mag   │
│ (Pin 6,9,14, │     │ GND         │     │ PSU GND     │
│  20,25,30,   │     │             │     │             │
│  34,39)      │     │             │     │             │
└──────────────┘     └──────────────┘     └──────────────┘
```

> [!CAUTION]
> **Common ground is essential!** Floating grounds cause erratic behavior.

---

## Configuration File Location

The authoritative pin configuration is in:
```
src/chess_hw_interface/config/pins.yaml
```

**Always update `pins.yaml` when changing physical wiring!**

---

## Pin Conflict Check

Before assigning new pins, verify no conflicts:

| BCM Pin | Currently Used By |
|---------|-------------------|
| 5 | Stepper B IN4 |
| 6 | X-MIN Limit Switch |
| 12 | Z-Axis Servo (gantry) |
| 13 | Y-MIN Limit Switch |
| 16 | Clock Servo (NEW) |
| 17 | Stepper A IN1 |
| 18 | Stepper A IN2 |
| 19 | Clock Hit Switch |
| 22 | Stepper A IN4 |
| 23 | Stepper B IN1 |
| 24 | Stepper B IN2 |
| 25 | Stepper B IN3 |
| 27 | Stepper A IN3 |

**Available pins**: 4, 7, 8, 9, 10, 11, 14, 15, 20, 21, 26

---

*See [wiring.md](wiring.md) for complete wiring diagrams.*
