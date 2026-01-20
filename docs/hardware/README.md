# Hardware Documentation

> **Complete hardware reference for the Smart Chess Board.**

## Overview

This smart chess board uses a CoreXY gantry system to move an electromagnet across the board, picking up and placing magnetic chess pieces. The system is controlled by a Raspberry Pi 4B running ROS 2.

## Quick Links

| Document | Description |
|----------|-------------|
| [components.md](components.md) | Bill of materials and component specifications |
| [pinout.md](pinout.md) | GPIO pin assignments (BCM numbering) |
| [wiring.md](wiring.md) | Electrical connections and diagrams |
| [power.md](power.md) | Power distribution and requirements |
| [mechanical.md](mechanical.md) | Mechanical assembly and CAD references |

## System Block Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                      POWER DISTRIBUTION                      │
│  ┌──────────┐    ┌──────────┐    ┌──────────┐               │
│  │ 5V/3A    │    │ 5V/2A    │    │ 5V USB-C │               │
│  │ Motors   │    │ Servo/Mag│    │ RPi      │               │
│  └────┬─────┘    └────┬─────┘    └────┬─────┘               │
└───────┼───────────────┼───────────────┼─────────────────────┘
        │               │               │
        ▼               ▼               ▼
┌───────────────────────────────────────────────────────────┐
│                     RASPBERRY PI 4B                        │
│  ┌─────────────────────────────────────────────────────┐  │
│  │                    GPIO HEADER                       │  │
│  │  Stepper A (4 pins) │ Stepper B (4 pins)            │  │
│  │  Servo PWM (1 pin)  │ Limit Switches (3 pins)       │  │
│  │  Magnet Control     │ Clock Display (8 pins)        │  │
│  └─────────────────────────────────────────────────────┘  │
│                          │                                 │
│  ┌───────────────────────┴───────────────────────────┐    │
│  │              CAMERA (CSI/USB)                      │    │
│  └────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────┘
        │               │               │
        ▼               ▼               ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  ULN2003A   │  │  ULN2003A   │  │   Servo     │
│  Driver A   │  │  Driver B   │  │   + Magnet  │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │
       ▼                ▼                ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│  Stepper A  │  │  Stepper B  │  │   Z-Axis    │
│  (28BYJ-48) │  │  (28BYJ-48) │  │   Lift      │
└─────────────┘  └─────────────┘  └─────────────┘
```

## Safety Considerations

> [!CAUTION]
> **Critical safety rules:**

1. **Never connect motor power to Pi's 5V pins** - Use separate power supply
2. **Common ground required** - All power supplies must share ground
3. **Current limits** - Pi GPIO can only source 16mA per pin, 50mA total
4. **Flyback diodes** - ULN2003 has built-in protection, but verify for other inductive loads
5. **Emergency stop** - Consider adding a physical e-stop button

## Hardware Status

<!-- USER_ATTENTION: Update this table as you test and verify each component -->

| Component | Acquired | Wired | Tested | Notes |
|-----------|----------|-------|--------|-------|
| Raspberry Pi 4B | ✅ | ✅ | ✅ | |
| Stepper Motor A | ⬜ | ⬜ | ⬜ | |
| Stepper Motor B | ⬜ | ⬜ | ⬜ | |
| ULN2003 Driver A | ⬜ | ⬜ | ⬜ | |
| ULN2003 Driver B | ⬜ | ⬜ | ⬜ | |
| SG90 Servo | ⬜ | ⬜ | ⬜ | |
| Electromagnet | ⬜ | ⬜ | ⬜ | |
| Camera Module | ⬜ | ⬜ | ⬜ | |
| Limit Switch X | ⬜ | ⬜ | ⬜ | |
| Limit Switch Y | ⬜ | ⬜ | ⬜ | |
| Limit Switch Clock | ⬜ | ⬜ | ⬜ | |
| Chess Clock Display | ⬜ | ⬜ | ⬜ | |

---

*See [AGENTS.md](../../AGENTS.md) for agent guidelines.*
