# Electromagnet Piece Pickup System

> **Z-axis mechanism and magnet control for piece manipulation.**

## Overview

The piece pickup system uses a servo-actuated arm to lower an electromagnet onto chess pieces. The magnet grips the piece (which has a steel disc or magnet embedded), and the servo raises it for transport.

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
     │         │   │  ◉  │   │◄──────────── Electromagnet
     │         │   │     │   │             │
     │         │   └─────┘   │             │
     │         └─────────────┘             │
     └─────────────────────────────────────┘

     RAISED POSITION              LOWERED POSITION
     (Servo at 0°)                (Servo at 90°)
     
         ╭─●                           ╭─●
         │                              ╲
         │                               ╲
         │                                │
        ═╪═                               │
         │                               ═╪═
```

---

## Servo Control

### PWM Configuration

| Position | Servo Angle | Duty Cycle | Pulse Width |
|----------|-------------|------------|-------------|
| UP (released) | 0° | 2.5% | 500µs |
| DOWN (engaged) | 90° | 7.5% | 1500µs |

<!-- USER_ATTENTION: Calibrate these values for your linkage geometry -->

### Control Implementation

```python
import RPi.GPIO as GPIO

class ServoController:
    def __init__(self, pin=12, freq=50):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        self.pwm = GPIO.PWM(pin, freq)
        self.pwm.start(0)
        
        # Calibrated positions
        self.UP_DUTY = 2.5
        self.DOWN_DUTY = 7.5
        self.MOVE_TIME = 0.5  # seconds
    
    def raise_magnet(self):
        """Raise the magnet (release piece)"""
        self.pwm.ChangeDutyCycle(self.UP_DUTY)
        time.sleep(self.MOVE_TIME)
        self.pwm.ChangeDutyCycle(0)  # Stop PWM signal
    
    def lower_magnet(self):
        """Lower the magnet (engage piece)"""
        self.pwm.ChangeDutyCycle(self.DOWN_DUTY)
        time.sleep(self.MOVE_TIME)
        self.pwm.ChangeDutyCycle(0)
    
    def cleanup(self):
        self.pwm.stop()
        GPIO.cleanup(self.pin)
```

---

## Electromagnet Control

### Specifications

| Parameter | Value |
|-----------|-------|
| Voltage | 5V DC |
| Current | ~400mA |
| Holding Force | 2.5 kg |
| Control | Via transistor/MOSFET |

### Circuit

```
                                    ┌──────────────┐
                                    │ ELECTROMAGNET│
    5V PSU (+) ────────────────────┤ (+)          │
                                    │              │
                                    │ (-)          │
                                    └───────┬──────┘
                                            │
                                1N4007      │
                             ┌───┤◄├───┐    │
                             │         │    │
                             └────┬────┘    │
                                  │         │
    GPIO Pin ──────[1kΩ]──────────┤ B       │
                                  │   NPN   │
                              C ──┴─────────┘
                              │
                              E
                              │
    GND ──────────────────────┴─────────────────
```

### Control Implementation

```python
class ElectromagnetController:
    def __init__(self, pin):
        self.pin = pin
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    def engage(self):
        """Turn on magnet to grip piece"""
        GPIO.output(self.pin, GPIO.HIGH)
    
    def release(self):
        """Turn off magnet to release piece"""
        GPIO.output(self.pin, GPIO.LOW)
    
    def cleanup(self):
        GPIO.output(self.pin, GPIO.LOW)
        GPIO.cleanup(self.pin)
```

---

## Pick and Place Sequence

### Full Pickup Sequence

```
1. Move gantry to source square (X,Y)
2. Lower servo (Z down)
3. Enable electromagnet
4. Wait for magnet to grip (50-100ms)
5. Raise servo (Z up)
6. Move gantry to destination (X,Y)
7. Lower servo (Z down)
8. Disable electromagnet
9. Wait for piece to release (50ms)
10. Raise servo (Z up)
```

### Timing Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Servo movement time | 500ms | Full swing up/down |
| Magnet engage delay | 100ms | Ensure magnetic grip |
| Magnet release delay | 50ms | Ensure piece released |
| Approach speed | 5 mm/s | Slow final approach |

---

## Position Calibration

### Z-Axis Travel

<!-- USER_ATTENTION: Measure and update these values -->

| Position | Height from board | Servo angle |
|----------|-------------------|-------------|
| Safe travel | 20mm | 0° |
| Piece contact | 2mm | ~80° |
| Magnet engaged | 0mm | 90° |

### Square-Specific Adjustments

Some squares may need height adjustment:
- Board may not be perfectly flat
- Different piece heights (King vs Pawn)

```yaml
# Height offsets per square (mm) - optional
square_z_offsets:
  a1: 0.0
  e4: 0.5  # Slightly higher if needed
  # ...
```

---

## Chess Piece Requirements

### Magnetic Compatibility

Pieces must have magnetic response:

| Method | Pros | Cons |
|--------|------|------|
| Steel disc in base | Cheap, works with electromagnet | May be too light |
| Magnet in base | Strong grip | Polarity matters |
| Steel-weighted base | Heavy = stable | Higher magnet force needed |

<!-- USER_ATTENTION: Specify your piece type and magnet compatibility -->

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
| Piece not gripped | Position sensor (if available) | Retry pickup |
| Piece dropped | Unexpected mass change | Re-home, alert user |
| Collision | Limit switch trigger | Emergency stop |
| Wrong piece | Post-move vision check | Undo and retry |

### Recovery Sequence

```
1. Raise Z to safe height
2. Release magnet
3. Re-home if position uncertain
4. Alert user if unrecoverable
```

---

## Power Considerations

### Current Draw

| Component | Active Current | Idle Current |
|-----------|----------------|--------------|
| Servo (moving) | 500mA peak | 10mA |
| Servo (holding) | 100mA | 10mA |
| Electromagnet | 400mA | 0mA |
| **Total peak** | **900mA** | **10mA** |

### Power Sequencing

Never activate servo and magnet simultaneously at startup:
1. Raise servo first (no magnet)
2. Then enable magnet control

---

## Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Piece not picked up | Z too high | Calibrate servo angle |
| Piece knocked over | Approach too fast | Reduce approach speed |
| Magnet doesn't hold | Insufficient current | Check power supply |
| Servo jitters | Power instability | Add capacitor |
| Piece slides off | Magnet too weak | Use stronger magnet |

---

*See [servo specs](../hardware/components.md#sg90-micro-servo) for hardware details.*
