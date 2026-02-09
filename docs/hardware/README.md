# Hardware Documentation

> Complete hardware reference for the Smart Chess Board.

## Canonical Gantry Stack

- Stepper motors: `NEMA 11` x2
- Stepper drivers: `A4988` x2 (`STEP/DIR`, shared `ENABLE`, active LOW)
- Controller: Raspberry Pi 4B GPIO (BCM numbering)
- Limit switches: X-min `GPIO10`, Y-min `GPIO9`, clock button `GPIO15` (active LOW, pull-up)
- Servos: gantry lift `GPIO12`, clock servo `GPIO18`

## Quick Links

| Document | Description |
|----------|-------------|
| [components.md](components.md) | Bill of materials and component specifications |
| [pinout.md](pinout.md) | GPIO pin assignments (BCM numbering) |
| [wiring.md](wiring.md) | Electrical connections and diagrams |
| [power.md](power.md) | Power distribution and requirements |
| [mechanical.md](mechanical.md) | Mechanical assembly and CAD references |

## Safety Rules

1. Do not power steppers from Pi 5V pins.
2. All PSU grounds must share a common ground with Pi ground.
3. A4988 VMOT must use a dedicated motor supply rail.
4. Keep `ENABLE` behavior explicit: LOW=enabled, HIGH=disabled.
5. Always call `GPIO.cleanup()` on process shutdown.

## Hardware Status

| Component | Acquired | Wired | Tested | Notes |
|-----------|----------|-------|--------|-------|
| Raspberry Pi 4B | ✅ | ✅ | ✅ | |
| NEMA 11 Motor A | ⬜ | ⬜ | ⬜ | |
| NEMA 11 Motor B | ⬜ | ⬜ | ⬜ | |
| A4988 Driver A | ⬜ | ⬜ | ⬜ | |
| A4988 Driver B | ⬜ | ⬜ | ⬜ | |
| SG90 Z Servo | ⬜ | ⬜ | ⬜ | |
| SG90 Clock Servo | ⬜ | ⬜ | ⬜ | |
| Electromagnet | ⬜ | ⬜ | ⬜ | |
| Limit Switch X | ⬜ | ⬜ | ⬜ | |
| Limit Switch Y | ⬜ | ⬜ | ⬜ | |
| Limit Switch Clock | ⬜ | ⬜ | ⬜ | |
| TM1637 Displays | ⬜ | ⬜ | ⬜ | |

---

See [AGENTS.md](../../AGENTS.md) for agent workflow constraints.
