# Mechanical Assembly & CAD

> **Mechanical design, assembly instructions, and CAD file references.**

## CoreXY Gantry Overview

The gantry uses a CoreXY belt configuration for X/Y motion, with a servo-actuated Z-axis for the permanent magnet.

```
                        ┌─────── Motor A
                        │
    ┌───────────────────┴──────────────────┐
    │ ╔═══════════════════════════════════╗ │
    │ ║   Y-axis carriage                 ║ │
    │ ║   ┌─────────────────────────────┐ ║ │
    │ ║   │                             │ ║ │
    │ ║   │      PERMANENT MAGNET       │ ║ │
    │ ║   │         HEAD                │ ║ │
    │ ║   │           ●                 │ ║ │
    │ ║   │                             │ ║ │
    │ ║   └─────────────────────────────┘ ║ │
    │ ╚═══════════════════════════════════╝ │
    │                                       │
    │       ← X-axis travel →               │
    │                                       │
    └───────────────────┬──────────────────┘
                        │
                        └─────── Motor B
```

---

## Frame Dimensions

<!-- USER_ATTENTION: Update these dimensions with your actual frame measurements -->

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frame outer width | 350mm | <!-- USER: verify --> |
| Frame outer depth | 350mm | <!-- USER: verify --> |
| Frame height | 100mm | <!-- USER: verify --> |
| X-axis travel | 250mm | <!-- USER: verify --> |
| Y-axis travel | 250mm | <!-- USER: verify --> |
| Z-axis travel | 15mm | Servo-actuated |
| Board playing area | 200mm × 200mm | 25mm per square |

---

## CoreXY Belt Path

```
    Motor A (17,18,27,22)         Motor B (23,24,25,5)
           ●                              ●
          ╱ ╲                            ╱ ╲
         ╱   ╲                          ╱   ╲
        ╱     ╲                        ╱     ╲
       ╱       ╲──────────────────────╱       ╲
      │         │                    │         │
      │    ┌────┴────────────────────┴────┐    │
      │    │         CARRIAGE             │    │
      │    │            ●                 │    │
      │    └────┬────────────────────┬────┘    │
      │         │                    │         │
       ╲       ╱──────────────────────╲       ╱
        ╲     ╱                        ╲     ╱
         ╲   ╱                          ╲   ╱
          ╲ ╱                            ╲ ╱
           ●                              ●
       Idler                          Idler
```

### CoreXY Motion Equations
```
To move in +X direction: Motor A + steps, Motor B + steps
To move in +Y direction: Motor A + steps, Motor B - steps

dA = dX + dY
dB = dX - dY

Inverse:
dX = (dA + dB) / 2
dY = (dA - dB) / 2
```

---

## Z-Axis (Permanent Magnet Lift)

### Servo Mount Design
```
    ┌─────────────────────┐
    │  CARRIAGE PLATE     │
    │  ┌───────────────┐  │
    │  │   SERVO SG90  │  │
    │  │   ┌───────┐   │  │
    │  │   │       │   │  │
    │  │   └───┬───┘   │  │
    │  │       │       │  │
    │  └───────┼───────┘  │
    │          │          │
    │    ┌─────┴─────┐    │
    │    │  LINKAGE  │    │
    │    └─────┬─────┘    │
    │          │          │
    │    ┌─────┴─────┐    │
    │    │ MAGNET    │    │
    │    │ HOLDER    │    │
    │    │    ◉      │    │  ← Permanent magnet
    │    └───────────┘    │
    └─────────────────────┘

    Servo 0°   = Magnet UP (released)
    Servo 90°  = Magnet DOWN (engaged)
```

<!-- USER_ATTENTION: Define servo angles based on your linkage geometry -->

---

## Linear Motion Components

### Rails/Rods
| Axis | Type | Length | Quantity |
|------|------|--------|----------|
| X-axis | 8mm smooth rod | 300mm | 2 |
| Y-axis | 8mm smooth rod | 300mm | 2 |

<!-- USER_ATTENTION: Update with your actual rail type and dimensions -->

### Bearings
| Type | Quantity | Location |
|------|----------|----------|
| LM8UU linear bearing | 4 | X-axis carriage |
| LM8UU linear bearing | 4 | Y-axis ends |

### Belt & Pulleys
| Component | Specification |
|-----------|---------------|
| Belt type | GT2 (2mm pitch) |
| Belt width | 6mm |
| Pulley teeth | 20 |
| Pulley bore | 5mm (motor shaft) |
| Idler pulley | Smooth or 20T |

---

## CAD Files

All CAD files are in the `/cad` directory:

```
cad/
├── exports/          # Exported files (STL, STEP, DXF)
├── parts/            # Individual part files
└── versions/         # Version history
```

### Key Files

Actual files under `cad/exports/STLs/` (non-exhaustive — see the directory for the full list, including printable chess pieces: `bishop.STL`, `horse.STL`, `king.STL`, `pawn.STL`, `queen.STL`, `rook.STL`):

| File | Description |
|------|-------------|
| `Servo magnet holder.STL` | Z-axis servo + permanent-magnet mount |
| `Camera holder.STL` | Camera mount bracket |
| `Rail rider.STL` | Linear rail carriage |
| `Corner SW 1.STL`, `Corner SW 2 motor.STL`, `Corner SW 2 motor controller.STL` | Frame corner brackets (motor-mounting variants) |
| `Electronics peg board.STL` / `v2` | Pi/driver mounting board |
| `clock face holder.STL`, `clock bottom V2.STL`, `Clock toy.STL` | Chess clock housing parts |
| `Feet.stl` / `feet V3.STL` | Frame feet |

Engineering drawings (`cad/exports/Engineering drawings/`, also as PDF in `cad/exports/pdfs/`): `Top`, `Bottom V1`, `Side 18in`, `Side 19in`. A DXF cut file is at `cad/exports/DXF/chess1.DXF`.

---

## Assembly Order

1. **Frame Assembly**
   - [ ] Assemble frame corners
   - [ ] Install linear rods
   - [ ] Verify frame squareness

2. **Motor Installation**
   - [ ] Mount Motor A (rear-left or designated position)
   - [ ] Mount Motor B (rear-right or designated position)
   - [ ] Install pulleys on motor shafts

3. **Belt Installation**
   - [ ] Install idler pulleys
   - [ ] Route belts in CoreXY pattern
   - [ ] Adjust belt tension (slight deflection)

4. **Carriage Assembly**
   - [ ] Install linear bearings in carriage
   - [ ] Attach belt ends to carriage
   - [ ] Test X/Y motion by hand

5. **Z-Axis Assembly**
   - [ ] Mount servo to carriage
   - [ ] Attach linkage mechanism
   - [ ] Install magnet holder (`Servo magnet holder.STL`)
   - [ ] Test servo motion

6. **Limit Switch Installation**
   - [ ] Mount X-MIN switch
   - [ ] Mount Y-MIN switch
   - [ ] Mount Clock switch (near clock area)
   - [ ] Verify trigger positions

7. **Camera Mount**
   - [ ] Install camera mount above board center
   - [ ] Aim camera perpendicular to board
   - [ ] Verify full board visibility

8. **Chess Board Placement**
   - [ ] Position board on frame
   - [ ] Align a1 square with homing position
   - [ ] Secure board to frame

---

## Calibration Marks

Add reference marks for calibration:

- **Home position**: Mark on frame where X-MIN and Y-MIN switches trigger
- **Board corners**: Mark positions of a1, a8, h1, h8 squares
- **Graveyard areas**: Mark captured piece zones

---

## Maintenance Notes

### Belt Tension
- Check belt tension monthly
- Replace if frayed or stretched
- Tension should allow ~5mm deflection with finger pressure

### Lubrication
- Linear rods: Light machine oil every 3 months
- Bearings: Should not require regular lubrication

### Alignment
- Check frame squareness if motion becomes rough
- Verify carriage moves freely without binding

---

*See [wiring.md](wiring.md) for electrical connections.*
