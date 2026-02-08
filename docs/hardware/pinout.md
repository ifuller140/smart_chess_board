# GPIO Pinout Reference

> **Complete GPIO pin assignments for the Smart Chess Board.**
> 
> ⚠️ **This is the authoritative source for pin assignments. Update `pins.yaml` to match.**

## Pin Assignment Summary

<!-- USER_ATTENTION: ⚠️ CRITICAL - Update these pin numbers to match your actual wiring! -->
<!-- The values below are PLACEHOLDERS based on the existing pins.yaml template -->

| Function | BCM Pin | Physical Pin | Wire Color | Notes |
|----------|---------|--------------|------------|-------|
| **Stepper A (Motor A)** |         |            |       |
| Direction | 27 | 13 | <!-- USER: color --> | A4988 Driver Direction |
| Step | 22 | 15 | <!-- USER: color --> | A4988 Driver Step |
| **Stepper ENABLE (shared)** | | | | |
| Enable | 17 | 11 | <!-- USER: color --> | A4988 ENABLE (active LOW) |
| **Stepper B (Motor B)** | | | | |
| Direction | 6 | 31 | <!-- USER: color --> | A4988 Driver Direction |
| Step | 5 | 29 | <!-- USER: color --> | A4988 Driver Step |
| **Z-Axis Servo (Gantry)** | | | | |
| PWM Signal | 12 | 32 | Orange | Hardware PWM |
| **Clock Servo (NEW)** | | | | |
| PWM Signal | 18 | 12 | <!-- USER: color --> | Hits clock button |
| **Limit Switches** | | | | |
| X-MIN Limit Switch | 10 | 19 | <!-- USER: color --> | Pull-up needed |
| Y-MIN Limit Switch | 9 | 21 | <!-- USER: color --> | Pull-up needed |
| Clock Hit Limit Switch | 15 | 10 | <!-- USER: color --> | Pull-up needed |
| **Clock Display** | | | | |
| Clock 1 CLK | 25 | 22 | | |
| Clock 1 DIO | 8 | 24 | | |
| Clock 2 CLK | 7 | 26 | | |
| Clock 2 DIO | 1 | 28 | | |

---

## Raspberry Pi 40-Pin Header Diagram

```
Power to A4988 Driver 3V3  (1)  (2)  5V
                    GPIO2  (3)  (4)  5V      To 5V power hub
                    GPIO3  (5)  (6)  GND     GND for 12V power hub
                    GPIO4  (7)  (8)  GPIO14  
GND for 5V hub        GND  (9)  (10) GPIO15  Clock limit switch
Motor ENABLE        GPIO17 (11) (12) GPIO18  Clock Servo PWM signal
Stepper A Direction GPIO27 (13) (14) GND
Stepper A Step      GPIO22 (15) (16) GPIO23 
Power to A4988 Driver 3V3  (17) (18) GPIO24
X limit switch     GPIO10  (19) (20) GND
Y limit switch      GPIO9  (21) (22) GPIO25 Clock 1 CLK
                   GPIO11  (23) (24) GPIO8 Clock 1 DIO
                      GND  (25) (26) GPIO7 Clock 2 CLK
                    GPIO0  (27) (28) GPIO1 Clock 2 DIO
Stepper B Step       GPIO5 (29) (30) GND
Stepper B Direction  GPIO6 (31) (32) GPIO12  Z-Axis Servo
                    GPIO13 (33) (34) GND
                    GPIO19 (35) (36) GPIO16
                   GPIO26  (37) (38) GPIO20
                      GND  (39) (40) GPIO21
```
---

## Pin Configuration Details

### Stepper Motor A (A4988 Driver + NEMA 11)
```yaml
stepper_driver:
  ros__parameters:
    motorA_dir_pin: 27   # Direction control
    motorA_step_pin: 22  # Step pulse
```

| Function | BCM | Physical | Direction | Notes |
|----------|-----|----------|-----------|-------|
| DIR | 27 | 13 | OUTPUT | HIGH = Forward, LOW = Reverse |
| STEP | 22 | 15 | OUTPUT | Rising edge = 1 step |

### Stepper Motor B (A4988 Driver + NEMA 11)
```yaml
stepper_driver:
  ros__parameters:
    motorB_dir_pin: 6    # Direction control
    motorB_step_pin: 5   # Step pulse
```

| Function | BCM | Physical | Direction | Notes |
|----------|-----|----------|-----------|-------|
| DIR | 6 | 31 | OUTPUT | HIGH = Forward, LOW = Reverse |
| STEP | 5 | 29 | OUTPUT | Rising edge = 1 step |

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
> GPIO 18 used for clock servo (hardware PWM per updated pinout).

### Clock Servo (NEW)
```yaml
clock_servo_node:
  ros__parameters:
    clock_servo_pin: 18         # Hits clock button after computer move
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
      x_min: 10
      y_min: 9
      clock_hit: 15
    debounce_ms: 20
```

| Switch | BCM | Pull | Active State |
|--------|-----|------|--------------|
| X-MIN | 10 | PULL_UP | LOW when pressed |
| Y-MIN | 9 | PULL_UP | LOW when pressed |
| Clock | 15 | PULL_UP | LOW when pressed |

### Clock Display (7-Segment)
```yaml
clock_display:
  ros__parameters:
    display1_pins: [25, 8]   # CLK, DIO
    display2_pins: [7, 1]    # CLK, DIO
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
| 1  | Clock 2 DIO |
| 5  | Stepper B STEP (A4988) |
| 6  | Stepper B DIR (A4988) |
| 7  | Clock 2 CLK |
| 8  | Clock 1 DIO |
| 9  | Y-MIN Limit Switch |
| 10 | X-MIN Limit Switch |
| 12 | Z-Axis Servo (gantry) |
| 15 | Clock Limit Switch |
| 17 | Motor ENABLE (A4988) |
| 18 | Clock Servo |
| 22 | Stepper A STEP (A4988) |
| 25 | Clock 1 CLK |
| 27 | Stepper A DIR (A4988) |

**Available pins**: 2, 3, 4, 11, 13, 14, 16, 19, 20, 21, 23, 24, 26

---

*See [wiring.md](wiring.md) for complete wiring diagrams.*
