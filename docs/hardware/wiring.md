# Electrical Wiring Guide

> **Complete wiring diagrams and connection instructions.**

## Safety First

> [!CAUTION]
> Before wiring:
> 1. **Power off all supplies** before making connections
> 2. **Double-check polarity** - reversed power can destroy components
> 3. **Verify voltage levels** - Pi GPIO is 3.3V, motors need 5V
> 4. **Use appropriate wire gauge** - motor power needs thicker wire

---

## Complete System Wiring Diagram

```
                              ┌─────────────────────────────────┐
                              │        RASPBERRY PI 4B          │
                              │                                 │
    ┌─────────────┐           │  ┌───────────────────────────┐  │
    │ 12V/2A PSU  │           │  │      GPIO HEADER          │  │
    │ (Motors)    │           │  │                           │  │
    │   (+) ──────┼───────────┼──┼─→ A4988 VMOT (not to Pi!) │  │
    │   (-) ──────┼──┬────────┼──┼─→ GND (common)            │  │
    └─────────────┘  │        │  │                           │  │
                     │        │  │  27 ─→ A4988 A DIR        │  │
    ┌─────────────┐  │        │  │  22 ─→ A4988 A STEP       │  │
    │ 5V/2A PSU   │  │        │  │   6 ─→ A4988 B DIR        │  │
    │ (Servo/Mag) │  │        │  │   5 ─→ A4988 B STEP       │  │
    │   (+) ──────┼──┼────────┼──┼─→ Servo VCC (red)         │  │
    │   (-) ──────┼──┼────────┼──┼─→ GND                     │  │
    └─────────────┘  │        │  │                           │  │
                     │        │  │  12 ─→ Magnet Servo       │  │
    ┌─────────────┐  │        │  │  16 ─→ Clock Servo        │  │
    │ USB-C PSU   │  │        │  │                           │  │
    │ (Pi)        │──┼────────┼──┼─→ Pi Power                │  │
    └─────────────┘  │        │  │  10 ─→ Limit X-MIN        │  │
                     │        │  │   9 ─→ Limit Y-MIN        │  │
                     │        │  │  15 ─→ Limit CLOCK        │  │
                     │        │  │                           │  │
                     │        │  │  25 ─→ Clock 1 CLK        │  │
                     │        │  │   8 ─→ Clock 1 DIO        │  │
                     │        │  │   7 ─→ Clock 2 CLK        │  │
                     │        │  │   1 ─→ Clock 2 DIO        │  │
                     │        │  │                           │  │
                     │        │  └───────────────────────────┘  │
                     │        │                                 │
                     │        │  ┌───────────────────────────┐  │
                     │        │  │      CSI CAMERA PORT      │  │
                     │        │  │  Ribbon cable to camera   │  │
                     │        │  └───────────────────────────┘  │
                     │        └─────────────────────────────────┘
                     │
    COMMON GROUND ───┴─────────────────────────────────────────────
```

<!-- USER_ATTENTION: Update this diagram to match your actual wiring -->

---

## Stepper Motor Wiring

### A4988 Driver to Raspberry Pi

The A4988 only requires 2 GPIO pins per motor (STEP and DIR):

```
   RASPBERRY PI                    A4988 DRIVER
   ┌──────────┐                    ┌──────────────┐
   │          │                    │              │
   │ GPIO 27 ─┼────────────────────┼─ DIR         │    ┌──────────────┐
   │ GPIO 22 ─┼────────────────────┼─ STEP        │    │   NEMA 11    │
   │          │                    │              │    │   STEPPER    │
   │          │                    │ 1A ──────────┼────┤   MOTOR      │
   │          │                    │ 1B ──────────┼────┤              │
   │          │                    │ 2A ──────────┼────┤              │
   │          │                    │ 2B ──────────┼────┤              │
   │          │                    │              │    └──────────────┘
   │   3.3V ──┼────────────────────┼─ VDD (logic) │
   │   GND ───┼────────────────────┼─ GND ────────┼──── PSU (-) 12V
   │          │                    │              │
   │          │    ┌───────────────┼─ VMOT ───────┼──── PSU (+) 12V
   │          │    │               │              │
   └──────────┘    │               └──────────────┘
                   │
             MOTOR POWER (8-35V, 1A+)
```

> [!IMPORTANT]
> - A4988 logic (VDD) runs on 3.3V from Pi
> - Motor power (VMOT) is separate 12V supply
> - Do NOT connect VMOT to Pi!

### Motor A (GPIO 27/22) and Motor B (GPIO 6/5)

| Driver | DIR Pin | STEP Pin |
|--------|---------|----------|
| Motor A | GPIO 27 | GPIO 22 |
| Motor B | GPIO 6 | GPIO 5 |

### A4988 Connections

| A4988 Pin | Connection | Notes |
|-----------|------------|-------|
| VDD | 3.3V from Pi | Logic power |
| GND | Common ground | Pi GND + PSU GND |
| VMOT | +12V PSU | Motor power (8-35V) |
| STEP | GPIO (22 or 5) | Step pulse |
| DIR | GPIO (27 or 6) | Direction |
| 1A, 1B | Motor coil 1 | NEMA 11 wires |
| 2A, 2B | Motor coil 2 | NEMA 11 wires |
| ENABLE | Not connected (or GND) | Low = enabled |
| MS1/MS2/MS3 | See microstepping | Optional |
| SLEEP | Connect to RESET | Keep awake |
| RESET | Connect to SLEEP | Keep awake |

### NEMA 11 Motor Wire Colors (typical)

| Wire Color | Function | A4988 Pin |
|------------|----------|-----------|
| Black | Coil A+ | 1A |
| Green | Coil A- | 1B |
| Red | Coil B+ | 2A |
| Blue | Coil B- | 2B |

> [!WARNING]
> Wire colors may vary by manufacturer - verify with multimeter!
> Test resistance: 5-50Ω between wires of same coil, infinite between coils.

---

## Servo Motor Wiring

```
    RASPBERRY PI                     SG90 SERVO
    ┌──────────┐                    ┌──────────┐
    │          │                    │          │
    │ GPIO 12 ─┼───── Orange ───────┼─ Signal  │
    │          │                    │          │
    │   GND ───┼───── Brown ────────┼─ GND     │
    │          │                    │          │
    └──────────┘                    └──────────┘
                                         │
                                         │ Red
         5V PSU (+) ─────────────────────┘
```

> [!WARNING]
> Do NOT power servo from Pi's 5V pin - use separate supply!

---

## Electromagnet Wiring

The electromagnet draws ~400mA, which is too much for GPIO. Use a transistor:

```
                                           ┌──────────────┐
                                           │ ELECTROMAGNET│
    5V PSU (+) ────────────────────────────┤ (+)          │
                                           │              │
    RASPBERRY PI        2N2222             │ (-)          │
    ┌──────────┐       ┌─────┐             └───────┬──────┘
    │          │   C   │     │ E                   │
    │ GPIO XX ─┼──────┤  B   ├─────────────────────┘
    │          │   │   │     │
    │   GND ───┼───┼───┴─────┴─────────────────────── PSU (-)
    │          │   │
    └──────────┘   │
                 1kΩ resistor between GPIO and transistor base
```

<!-- USER_ATTENTION: Assign a GPIO pin for electromagnet control and update pinout.md -->

### Alternative: MOSFET Control
If using a logic-level MOSFET (e.g., IRLZ44N):
- Gate → GPIO (direct, no resistor needed)
- Source → GND
- Drain → Electromagnet (-)
- Add flyback diode (1N4007) across electromagnet terminals

---

## Limit Switch Wiring

Using internal pull-up resistors (no external resistors needed):

```
    RASPBERRY PI              LIMIT SWITCH
    ┌──────────┐              ┌──────┐
    │          │              │ COM ─┼──── GND
    │  GPIO 10 ─┼──────────────┼─ NO  │
    │          │              │ NC   │ (not used)
    │   GND ───┼──────────────┼──────┘
    │          │              
    └──────────┘              

    Configuration: GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    
    Switch open:  GPIO reads HIGH (1)
    Switch closed: GPIO reads LOW (0)
```

Repeat for all three limit switches (X-MIN, Y-MIN, CLOCK).

---

## Camera Connection

### CSI Camera (RPi Camera Module)
1. Locate the CSI port between HDMI and audio jack
2. Lift the plastic clip gently
3. Insert ribbon cable with blue side facing the Ethernet port
4. Press clip down to secure

### USB Camera
1. Connect to any USB port
2. Verify with `ls /dev/video*`

---

## Power Distribution Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                     POWER DISTRIBUTION                          │
│                                                                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │  WALL       │    │  WALL       │    │  WALL       │         │
│  │  OUTLET     │    │  OUTLET     │    │  OUTLET     │         │
│  └──────┬──────┘    └──────┬──────┘    └──────┬──────┘         │
│         │                  │                  │                 │
│         ▼                  ▼                  ▼                 │
│  ┌─────────────┐    ┌─────────────┐    ┌─────────────┐         │
│  │ 5V/3A PSU   │    │ 5V/2A PSU   │    │ 5V/3A USB-C │         │
│  │ (Motors)    │    │ (Servo/Mag) │    │ (Pi)        │         │
│  │             │    │             │    │             │         │
│  │    (+)──────┼────┼────(+)──────┼────┼────────────────→ 5V   │
│  │    (-)──────┼────┼────(-)──────┼────┼────────────────→ GND  │
│  └─────────────┘    └─────────────┘    └─────────────┘         │
│         │                  │                                    │
│         ▼                  ▼                                    │
│  ┌─────────────┐    ┌─────────────┐                            │
│  │ ULN2003 ×2  │    │ Servo +     │                            │
│  │             │    │ Electromagnet│                            │
│  └─────────────┘    └─────────────┘                            │
│                                                                 │
│  ════════════════════════════════════════════════════════════  │
│                      COMMON GROUND BUS                          │
└─────────────────────────────────────────────────────────────────┘
```

<!-- USER_ATTENTION: You may be able to use a single 5V PSU if it has enough current capacity -->

---

## Wire Gauge Recommendations

| Connection | Current | Gauge | Notes |
|------------|---------|-------|-------|
| Pi USB-C power | 3A | 18-20 AWG | Use quality cable |
| Motor power | 500mA×2 | 22 AWG | Per motor |
| Servo power | 1A peak | 22 AWG | |
| Electromagnet | 400mA | 22 AWG | |
| GPIO signals | <20mA | 26-28 AWG | Dupont jumpers OK |
| Limit switches | <1mA | 26-28 AWG | Dupont jumpers OK |

---

## Wiring Checklist

Before powering on:

- [ ] All grounds connected together
- [ ] Motor power NOT connected to Pi 5V pin
- [ ] Servo power NOT connected to Pi 5V pin
- [ ] Camera ribbon cable seated correctly
- [ ] Limit switches wired to NO terminal
- [ ] GPIO pins match `pins.yaml` configuration
- [ ] No bare wires touching metal frame
- [ ] All connections secure (no loose jumpers)

---

*See [power.md](power.md) for power budget calculations.*
