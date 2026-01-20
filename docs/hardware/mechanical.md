# Mechanical Assembly & CAD

> **Mechanical design, assembly instructions, and CAD file references.**

## CoreXY Gantry Overview

The gantry uses a CoreXY belt configuration for X/Y motion, with a servo-actuated Z-axis for the electromagnet.

```
                        ┌─────── Motor A
                        │
    ┌───────────────────┴──────────────────┐
    │ ╔═══════════════════════════════════╗ │
    │ ║   Y-axis carriage                 ║ │
    │ ║   ┌─────────────────────────────┐ ║ │
    │ ║   │                             │ ║ │
    │ ║   │      ELECTROMAGNET          │ ║ │
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

## Z-Axis (Electromagnet Lift)

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
    │    │    ◉      │    │  ← Electromagnet
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
<!-- USER_ATTENTION: List your actual CAD files here -->

| File | Description | Format |
|------|-------------|--------|
| `frame_assembly.step` | Complete frame assembly | STEP |
| `carriage.stl` | X/Y carriage plate | STL |
| `servo_mount.stl` | Servo mounting bracket | STL |
| `magnet_holder.stl` | Electromagnet holder | STL |
| `limit_switch_mount.stl` | Limit switch brackets | STL |

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
   - [ ] Install electromagnet holder
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
