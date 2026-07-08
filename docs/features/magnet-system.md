# Permanent Magnet Piece Pickup System

> **Z-axis mechanism and magnet control for piece manipulation.**

## Overview

The piece pickup system uses a servo-actuated arm to lower a **permanent magnet** onto chess pieces (each piece has a steel disc or magnet embedded in its base). Unlike an electromagnet, the magnet itself has no power connection — pickup and release are controlled entirely by the servo's position: lowering the magnet close enough to the piece lets it grip magnetically, and raising it back up breaks the grip. There is no GPIO pin dedicated to magnet power.

This is implemented today by `src/chess_hw_interface/chess_hw_interface/nodes/servo_node.py`, which exposes two services:

| Service | Type | Effect |
|---------|------|--------|
| `/servo/engage` | `std_srvs/Trigger` | Lowers the servo to `engage_pulse_us` (down position) — magnet is close enough to grip a piece |
| `/servo/release` | `std_srvs/Trigger` | Raises the servo to `release_pulse_us` (up position) — magnet lifts away, releasing the piece |

The node also publishes its current state (`"engaged"` / `"released"`) on `/servo/state`, and subscribes to `/emergency_stop` to immediately stop sending pulses if triggered.

## Mechanical Design

### Z-Axis Assembly

```
     ┌─────────────────────────────────────┐
     │         CARRIAGE PLATE              │
     │                                     │
     │    ┌───────────────────────┐        │
     │    │      SG90 SERVO       │        │
     │    │    ┌─────────────┐    │        │
     │    │    │   ╭───╮     │    │        │
     │    │    │   │●  │◄────│────│───── Servo horn
     │    │    │   ╰───╯     │    │        │
     │    │    └──────┼──────┘    │        │
     │    │           │           │        │
     │    │    ┌──────┴──────┐    │        │
     │    │    │   LINKAGE   │◄───│───── Pushrod or lever
     │    │    └──────┬──────┘    │        │
     │    │           │           │        │
     │    └───────────┼───────────┘        │
     │                │                    │
     │         ┌──────┴──────┐             │
     │         │   SLIDER    │◄──────────── Vertical guide
     │         │             │             │
     │         │   ┌─────┐   │             │
     │         │   │     │   │             │
     │         │   │  ◉  │   │◄──────────── Permanent magnet
     │         │   │     │   │             │
     │         │   └─────┘   │             │
     │         └─────────────┘             │
     └─────────────────────────────────────┘

     RAISED POSITION              LOWERED POSITION
     (Released — up)              (Engaged — down)
     
         ╭─●                           ╭─●
         │                              ╲
         │                               ╲
         │                                │
        ═╪═                               │
         │                               ═╪═
```

---

## Servo Control

### PWM Configuration (`src/chess_hw_interface/config/pins.yaml` → `servo_node`)

| Position | Parameter | Default | Meaning |
|----------|-----------|---------|---------|
| DOWN (engaged) | `engage_pulse_us` | 500µs | Magnet close enough to the piece to grip |
| UP (released) | `release_pulse_us` | 1500µs | Magnet lifted clear, piece released |
| — | `servo_pin` | GPIO 12 (BCM) | Hardware PWM pin |
| — | `movement_time` | 0.5s | Time allowed for the servo to reach position before pulses stop |

<!-- USER_ATTENTION: Calibrate engage_pulse_us / release_pulse_us for your linkage geometry -->

`servo_node.py` uses `pigpio`'s `set_servo_pulsewidth()` for jitter-free hardware-timed PWM (not software PWM / `RPi.GPIO`), and stops sending pulses immediately after each move to avoid servo jitter/heat.

---

## Pick and Place Sequence

### Full Pickup Sequence

```
1. Move gantry to source square (X,Y)
2. Call /servo/engage  — lower magnet, grip piece
3. Wait for movement_time (servo settles)
4. Move gantry to destination (X,Y)
5. Call /servo/release — raise magnet, release piece
6. Wait for movement_time (servo settles)
```

There is no separate "enable magnet" step — steps 2 and 5 are the entire pickup/release mechanism, driven purely by servo angle.

### Timing Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Servo movement time | 500ms (`movement_time`) | Full swing up/down |
| Approach speed | 5 mm/s | Slow final approach so pieces aren't knocked over |

---

## Position Calibration

### Z-Axis Travel

<!-- USER_ATTENTION: Measure and update these values on the physical rig -->

The two positions that matter are just `engage_pulse_us` (down/grip) and `release_pulse_us` (up/clear) in `pins.yaml`. There is no separate "safe travel height" parameter today — the gantry only moves in X/Y at the release height.

### Square-Specific Adjustments

Not currently implemented. If the board surface isn't perfectly flat or piece heights vary enough to matter, a future per-square Z offset could be added to `pins.yaml` or `board_map.yaml`, but no such mechanism exists in the code today.

---

## Chess Piece Requirements

### Magnetic Compatibility

Pieces must have magnetic response for the permanent magnet to grip them:

| Method | Pros | Cons |
|--------|------|------|
| Steel disc in base | Cheap, works well with a permanent magnet | May be too light on its own |
| Magnet in base | Strong grip | Polarity/orientation matters |
| Steel-weighted base | Heavy = stable | Needs a stronger magnet to lift |

<!-- USER_ATTENTION: Specify your actual piece type and magnet compatibility -->

### Piece Dimensions

| Piece | Base Diameter | Height | Weight |
|-------|---------------|--------|--------|
| Pawn | 20mm | 30mm | 5g |
| Knight | 22mm | 40mm | 10g |
| Bishop | 22mm | 45mm | 10g |
| Rook | 24mm | 35mm | 15g |
| Queen | 24mm | 55mm | 15g |
| King | 24mm | 60mm | 15g |

<!-- USER_ATTENTION: Measure your actual pieces -->

---

## Error Handling

### Pickup Failures

| Situation | Detection | Recovery |
|-----------|-----------|----------|
| Piece not gripped | No position sensor today — would need post-move vision check | Retry pickup |
| Piece dropped | No mass/position sensing today | Re-home, alert user |
| Collision | Limit switch trigger | Emergency stop (`/emergency_stop`) |
| Wrong piece moved | Post-move vision check (`chess_perception`) | Undo and retry |

### Recovery Sequence

```
1. Call /servo/release (raise to safe/released position)
2. Re-home if position uncertain
3. Alert user if unrecoverable
```

---

## Power Considerations

Since the magnet itself draws no current (it's passive/permanent), the only active-current component here is the servo:

| Component | Active Current | Idle Current |
|-----------|----------------|--------------|
| Servo (moving) | 500mA peak | 10mA |
| Servo (holding) | 100mA | 10mA |

See `docs/hardware/power.md` for the full system power budget.

---

## Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Piece not picked up | `engage_pulse_us` too high (magnet not low enough) | Lower `engage_pulse_us`, recalibrate |
| Piece knocked over | Approach too fast | Reduce approach speed |
| Magnet doesn't hold piece | Piece too far from magnet, or piece's steel disc/magnet too weak | Adjust `engage_pulse_us` or piece magnetic insert |
| Servo jitters | Power instability, or pulses left active too long | Check power supply; confirm `set_servo_pulsewidth(pin, 0)` runs after each move |
| Piece slides off during travel | Approach/travel speed too high | Reduce gantry speed while piece is engaged |

---

*See [servo specs](../hardware/components.md#sg90-micro-servo) for hardware details, and `src/chess_hw_interface/chess_hw_interface/nodes/servo_node.py` for the current implementation.*
