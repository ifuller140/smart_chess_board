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
| +X (right) | + steps (CCW) | - steps (CW) | OPPOSITE |
| -X (left) | - steps (CW) | + steps (CCW) | OPPOSITE |
| +Y (up/forward) | + steps (CCW) | + steps (CCW) | SAME |
| -Y (down/backward) | - steps (CW) | - steps (CW) | SAME |
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

### Algorithm (Prusa-style)

```
# Y axis first — Y limit is at Y=0 (front, player side)
1. RAISE MAGNET (servo to release position — safety)
2. MOVE_Y_NEGATIVE (toward player) until Y_MIN limit triggered
3. BACK_OFF backoff_steps in +Y direction (away from limit)
4. MOVE_Y_NEGATIVE slowly (small batches) until Y_MIN triggered again
   → Gantry is now precisely at Y=0

# X axis — X limit is at X_MAX (right side, h-file side)
5. MOVE_X_POSITIVE (rightward) until X_MAX limit triggered
6. BACK_OFF backoff_steps in -X direction (away from limit)
7. MOVE_X_POSITIVE slowly (small batches) until X_MAX triggered again
   → Gantry is now precisely at X_MAX

# Drive to coordinate origin
8. MOVE_X_NEGATIVE by x_max_mm steps (drive leftward to X=0)
   → Gantry is now at physical position (0,0) = bottom-left

9. RESET stepper driver position counter to (0,0)
10. PUBLISH /gantry/status = "HOMED"
```

### Limit Switch Positions

```
  Player's LEFT side (a-file)        Player's RIGHT side (h-file)

  ┌─────────────────────────────────────────────────────────┐
  │                                          [X limit] ─●  │ ← X_MAX (right)
  │                                                        │
  │                  8×8 CHESS BOARD                       │
  │   a8 ─────────────────────────────── h8                │
  │   │                                  │                 │
  │   │                                  │                 │
  │   a1 ─────────────────────────────── h1                │
  │                                                        │
  ●─[Y limit]─────────────────────────────────────────────┘
  ↑ Y=0, front (player sits here)
  
  ★ Origin (0,0) = bottom-LEFT after homing sequence completes
    (Y limit at front, X driven back to left after touching X limit)
```

---

## Coordinate System

### Origin and Axes

```
  (from player's perspective, player at bottom)

        +Y (ranks 1→8, toward camera/back)
         ▲
         │
         │    Board Area
         │   ┌─────────────────────────────┐──── X limit (right)
         │   │ a8  b8  c8  ...  g8    h8  │     (X_MAX ≈ 240mm)
         │   │  .   .                  .  │
         │   │  .   .      board       .  │
         │   │ a1  b1  c1  ...  g1    h1  │
         │   └─────────────────────────────┘
         │
    (0,0)└──────────────────────────────────► +X (files a→h, rightward)
    Origin                                         (toward h-file)
    (bottom-left)
    ↑ Y limit here (Y=0, player's side)
```

- **(0,0) = bottom-left** — set by homing sequence after touching both limits
- **+X = rightward** toward h-file (and toward X limit switch at X_MAX)
- **+Y = backward** toward rank 8 / camera tower (away from player)
- **X limit switch** at X_MAX (right side). Homing moves in **+X** until triggered.
- **Y limit switch** at Y=0 (front). Homing moves in **−Y** until triggered.

### Square Centers

Chess squares are addressed by their center point.
With `board_origin_x=20`, `board_origin_y=20`, `square_size=25mm`:

| Square | X (mm) | Y (mm) | Notes |
|--------|--------|--------|-------|
| a1 | ~20 | ~20 | Near-left, player's front |
| h1 | ~195 | ~20 | Near-right, player's front (near X limit) |
| a8 | ~20 | ~195 | Far-left, back (Black's queen-side) |
| h8 | ~195 | ~195 | Far-right, back (Black's king-side) |

Formula:
```
X_mm = board_origin_x_mm + (col_index * square_size_mm)
Y_mm = board_origin_y_mm + (rank_index * square_size_mm)

where col_index: a=0, b=1, ..., h=7
      rank_index: rank1=0, rank2=1, ..., rank8=7
```

> [!NOTE]
> `board_origin_x_mm` and `board_origin_y_mm` default to 20.0 each and must be
> calibrated against the physical board position after installation.

---

## Implementation Notes

### Current Position Tracking

Position is tracked in motor steps from home (stepper_driver_node):
```python
self._pos_steps_a = 0  # Motor A step accumulator
self._pos_steps_b = 0  # Motor B step accumulator

# After any movement:
self._pos_steps_a += steps_a
self._pos_steps_b += steps_b

# Forward kinematics (CoreXY):
# +X = A+, B-   →   pos_x = (A - B) / 2
# +Y = A+, B+   →   pos_y = (A + B) / 2
pos_x_mm = (self._pos_steps_a - self._pos_steps_b) / 2 / steps_per_mm
pos_y_mm = (self._pos_steps_a + self._pos_steps_b) / 2 / steps_per_mm
```

Reset after homing:
```python
# Called via /stepper/reset_position (Bool=True)
self._pos_steps_a = 0
self._pos_steps_b = 0
# Now (0,0) = bottom-left corner
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
