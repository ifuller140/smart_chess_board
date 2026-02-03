# CoreXY Gantry System

> **Motion system design, kinematics, and implementation details.**

## Overview

The gantry uses a CoreXY belt configuration that converts rotational motion from two stepper motors into smooth X/Y movement. This design offers:

- Fast motion (both motors contribute to each axis)
- Lightweight moving mass (motors are stationary)
- Good precision with proper belt tension

## CoreXY Kinematics

### Belt Path Diagram

```
       Motor A                                Motor B
          ●                                      ●
         ╱│╲                                    ╱│╲
        ╱ │ ╲                                  ╱ │ ╲
       ╱  │  ╲                                ╱  │  ╲
      ╱   │   ╲                              ╱   │   ╲
   ●─────┴─────●════════════════════════════●─────┴─────●
   │           │          BELT A            │           │
   │           │            ↓               │           │
   │     ┌─────┴────────────────────────────┴─────┐     │
   │     │          CARRIAGE                      │     │
   │     │              ●                         │     │
   │     └─────┬────────────────────────────┬─────┘     │
   │           │          BELT B            │           │
   │           │            ↑               │           │
   ●─────┬─────●════════════════════════════●─────┬─────●
        ╲ │ ╱                                ╲ │ ╱
         ╲│╱                                  ╲│╱
          ●                                    ●
       Idler                                Idler
```

### Motion Equations

> [!IMPORTANT]
> Our physical layout: Motor A at bottom-left, Motor B at top-right.
> This affects the kinematic equations compared to standard CoreXY.

**Forward Kinematics** (Motor steps → Cartesian position):
```
X_position = (steps_A - steps_B) / 2 / steps_per_mm
Y_position = (steps_A + steps_B) / 2 / steps_per_mm
```

**Inverse Kinematics** (Cartesian position → Motor steps):
```
steps_A = (X_mm + Y_mm) * steps_per_mm
steps_B = (-X_mm + Y_mm) * steps_per_mm
```

### Movement Examples

| Desired Motion | Motor A | Motor B | Direction Pattern |
|----------------|---------|---------|-------------------|
| +X (right) | + steps (CW) | - steps (CCW) | OPPOSITE |
| -X (left) | - steps (CCW) | + steps (CW) | OPPOSITE |
| +Y (up/forward) | + steps (CW) | + steps (CW) | SAME |
| -Y (down/backward) | - steps (CCW) | - steps (CCW) | SAME |
| +X +Y (diagonal) | + steps | 0 steps | |
| -X +Y (diagonal) | 0 steps | + steps | |

> [!NOTE]
> **Key insight**: 
> - X movement uses OPPOSITE motor directions
> - Y movement uses SAME motor directions

---

## Steps Per Millimeter Calculation

### Theoretical Value

```
Pulley teeth: 20
Belt pitch: 2mm (GT2)
Pulley circumference = 20 × 2mm = 40mm

Motor steps per revolution: 200 (NEMA 11, full-step)

steps_per_mm = 200 / 40 = 5 steps/mm
```

> [!NOTE]
> With microstepping (1/16), this becomes 80 steps/mm.
> Currently using full-step mode (MS1/MS2/MS3 not connected).

### Calibration Procedure

<!-- USER_ATTENTION: Follow this procedure to determine your actual steps/mm -->

1. Home the gantry
2. Mark current position on frame
3. Command movement of exactly 100mm in X
4. Measure actual distance traveled
5. Calculate correction:
   ```
   actual_steps_per_mm = commanded_distance / measured_distance * 5
   ```
6. Update `board_map.yaml` with calibrated value

---

## Motion Parameters

### Speed Limits

| Parameter | Value | Reason |
|-----------|-------|--------|
| Maximum speed | ~200+ mm/s | NEMA 11 motor capability |
| Safe travel speed | 100 mm/s | Reliable without missed steps |
| Approach speed | 30 mm/s | Precise piece pickup |
| Homing speed | 20 mm/s | Controlled contact with switches |

> [!NOTE]
> NEMA 11 motors are significantly faster than 28BYJ-48.
> Use speed control (0-100%) to find optimal operating speed.
> Start testing at low speeds (20-40%) to verify wiring.

### Acceleration

```
Trapezoid velocity profile:
         ___________
        ╱           ╲
       ╱             ╲
      ╱               ╲
     ╱                 ╲
────●                   ●────
  start               end
  
Acceleration: 50 mm/s²
Deceleration: 50 mm/s² (symmetric)
```

<!-- USER_ATTENTION: Tune acceleration for smooth motion without losing steps -->

---

## Homing Sequence

### Algorithm

```
1. MOVE_X_NEGATIVE until X_MIN limit triggered
2. BACK_OFF 5mm in +X direction
3. MOVE_X_NEGATIVE slowly until X_MIN triggered again
4. SET X_POSITION = 0

5. MOVE_Y_NEGATIVE until Y_MIN limit triggered
6. BACK_OFF 5mm in +Y direction
7. MOVE_Y_NEGATIVE slowly until Y_MIN triggered again
8. SET Y_POSITION = 0

9. GANTRY HOMED
```

### Limit Switch Positions

```
                    ┌─────────────────────────────────┐
                    │                                 │
                    │                                 │
                    │                                 │
                    │                                 │
                    │                                 │
                    │                                 │
          Y_MIN ────●               ●──── (cable exit)
                    │                                 │
           X_MIN ───●─────────────────────────────────┘
                    
                  (0,0) after homing
```

---

## Coordinate System

### Origin and Axes

```
        +Y (ranks 1→8)
         ▲
         │
         │    Board Area
         │   ┌─────────────────────┐
         │   │ a8  b8  ...     h8  │
         │   │  .   .           .  │
         │   │  .   .           .  │
         │   │ a1  b1  ...     h1  │
         │   └─────────────────────┘
         │
    (0,0)└────────────────────────────────► +X (files a→h)
         Home position
```

### Square Centers

Chess squares are addressed by their center point.

For a 25mm square size with origin at a1 center:

| Square | X (mm) | Y (mm) |
|--------|--------|--------|
| a1 | 25 | 25 |
| h1 | 200 | 25 |
| a8 | 25 | 200 |
| h8 | 200 | 200 |

Formula:
```
X_mm = board_origin_x + (file_index * square_size)
Y_mm = board_origin_y + (rank_index * square_size)

where file_index: a=0, b=1, ..., h=7
      rank_index: 1=0, 2=1, ..., 8=7
```

---

## Implementation Notes

### Current Position Tracking

Position is tracked in motor steps from home:
```python
self.position_a = 0  # Steps from home
self.position_b = 0  # Steps from home

# After any movement:
self.position_a += delta_a
self.position_b += delta_b

# Convert to mm:
x_mm = (self.position_a + self.position_b) / 2 / self.steps_per_mm
y_mm = (self.position_a - self.position_b) / 2 / self.steps_per_mm
```

### Synchronized Motor Movement

For smooth diagonal motion, motors must step in sync:

```python
def move_to(self, target_x_mm, target_y_mm):
    delta_a = target_a_steps - self.position_a
    delta_b = target_b_steps - self.position_b
    
    # Bresenham-style stepping for synchronized motion
    steps_a = abs(delta_a)
    steps_b = abs(delta_b)
    max_steps = max(steps_a, steps_b)
    
    for step in range(max_steps):
        if step * steps_a >= (step_a_counter * max_steps):
            step_motor_a(direction_a)
            step_a_counter += 1
        if step * steps_b >= (step_b_counter * max_steps):
            step_motor_b(direction_b)
            step_b_counter += 1
```

### Error Handling

| Error | Detection | Recovery |
|-------|-----------|----------|
| Missed steps | Position drift over time | Re-home periodically |
| Belt skip | Sudden position error | Emergency stop, manual check |
| Motor stall | Motor gets hot, no movement | Reduce speed, check for obstruction |
| Limit switch fail | Hit end without trigger | Software end-stops as backup |

---

## Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Motion not straight | Belt tension uneven | Adjust tension |
| Circles are ovals | steps_per_mm wrong | Recalibrate |
| Grinding noise | Motor too fast | Increase step delay |
| Lost steps | Acceleration too high | Reduce acceleration |
| Binding motion | Rails not parallel | Re-align frame |

---

*See [mechanical.md](../hardware/mechanical.md) for frame assembly.*
