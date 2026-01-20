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
    │ 5V/3A PSU   │           │  │      GPIO HEADER          │  │
    │ (Motors)    │           │  │                           │  │
    │   (+) ──────┼───────────┼──┼─→ (not connected to Pi!)  │  │
    │   (-) ──────┼──┬────────┼──┼─→ GND (multiple pins)     │  │
    └─────────────┘  │        │  │                           │  │
                     │        │  │  17 ─→ ULN2003 A IN1      │  │
    ┌─────────────┐  │        │  │  18 ─→ ULN2003 A IN2      │  │
    │ 5V/2A PSU   │  │        │  │  27 ─→ ULN2003 A IN3      │  │
    │ (Servo/Mag) │  │        │  │  22 ─→ ULN2003 A IN4      │  │
    │   (+) ──────┼──┼────────┼──┼─→ Servo VCC (red)         │  │
    │   (-) ──────┼──┼────────┼──┼─→ GND                     │  │
    └─────────────┘  │        │  │                           │  │
                     │        │  │  23 ─→ ULN2003 B IN1      │  │
    ┌─────────────┐  │        │  │  24 ─→ ULN2003 B IN2      │  │
    │ USB-C PSU   │  │        │  │  25 ─→ ULN2003 B IN3      │  │
    │ (Pi)        │──┼────────┼──┼─→ Pi Power                │  │
    └─────────────┘  │        │  │   5 ─→ ULN2003 B IN4      │  │
                     │        │  │                           │  │
                     │        │  │  12 ─→ Servo Signal (org) │  │
                     │        │  │                           │  │
                     │        │  │   6 ─→ Limit X-MIN        │  │
                     │        │  │  13 ─→ Limit Y-MIN        │  │
                     │        │  │  19 ─→ Limit CLOCK        │  │
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

### ULN2003 Driver to Raspberry Pi

```
   RASPBERRY PI                    ULN2003A DRIVER
   ┌──────────┐                    ┌──────────────┐
   │          │                    │              │
   │ GPIO 17 ─┼────────────────────┼─ IN1         │
   │ GPIO 18 ─┼────────────────────┼─ IN2         │
   │ GPIO 27 ─┼────────────────────┼─ IN3         │
   │ GPIO 22 ─┼────────────────────┼─ IN4         │
   │          │                    │              │
   │   GND ───┼────────────────────┼─ GND ────────┼──── PSU (-)
   │          │                    │              │
   │          │    ┌───────────────┼─ VCC ────────┼──── PSU (+) 5V
   │          │    │               │              │
   └──────────┘    │               └──────────────┘
                   │                     │
                   │                     │ Motor connector
                   │                     ▼
                   │               ┌──────────────┐
                   │               │   28BYJ-48   │
                   │               │   STEPPER    │
                   │               │   MOTOR      │
                   └───────────────┤              │
                     (5-wire conn) └──────────────┘
```

### Motor Wire Colors (28BYJ-48)

| Wire Color | Function | ULN2003 Pin |
|------------|----------|-------------|
| Red | Common (+) | (internal to board) |
| Orange | Coil A (end) | OUT1 |
| Yellow | Coil A (center) | OUT3 |
| Pink | Coil B (end) | OUT2 |
| Blue | Coil B (center) | OUT4 |

<!-- USER_ATTENTION: Wire colors may vary by manufacturer - verify with multimeter -->

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
    │  GPIO 6 ─┼──────────────┼─ NO  │
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
