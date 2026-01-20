# Power Distribution & Requirements

> **Power budget and distribution documentation.**

## Power Supply Requirements

### Summary
| Supply | Voltage | Min Current | Purpose |
|--------|---------|-------------|---------|
| Pi Power | 5V | 3A | Raspberry Pi 4B |
| Motor Power | 5V | 1A | 2× Stepper motors via ULN2003 |
| Servo/Mag Power | 5V | 1A | Servo + Electromagnet |

**Total 5V current needed**: ~5A (with headroom)

<!-- USER_ATTENTION: Consider using a single 5V/5A supply with proper distribution -->

---

## Component Power Budget

### Raspberry Pi 4B
| State | Current Draw |
|-------|--------------|
| Idle (no peripherals) | 600mA |
| With camera active | 800mA |
| Peak (CPU stressed) | 1200mA |
| **Recommended supply** | **3A** |

### Stepper Motors (28BYJ-48)
| State | Current per Motor | Total (2 motors) |
|-------|-------------------|------------------|
| Idle (energized) | 240mA | 480mA |
| Stepping | 200-300mA | 400-600mA |
| Peak (stall) | 300mA | 600mA |
| **Recommended supply** | - | **1A** |

### Servo Motor (SG90)
| State | Current Draw |
|-------|--------------|
| Idle | 10mA |
| Moving (no load) | 100-250mA |
| Moving (load) | 400-650mA |
| Stall | ~750mA |

### Electromagnet
| State | Current Draw |
|-------|--------------|
| Off | 0mA |
| Engaged | 400mA |

### Combined Servo + Electromagnet
| State | Current Draw |
|-------|--------------|
| Both idle | ~10mA |
| Servo moving + magnet on | ~850mA |
| **Recommended supply** | **1A** |

---

## Power Architecture Options

### Option 1: Three Separate Supplies (Safest)
```
┌────────────┐   ┌────────────┐   ┌────────────┐
│ USB-C 5V/3A│   │ 5V/1A PSU  │   │ 5V/1A PSU  │
│ (Pi)       │   │ (Motors)   │   │ (Servo+Mag)│
└─────┬──────┘   └─────┬──────┘   └─────┬──────┘
      │                │                │
      ▼                ▼                ▼
┌──────────┐    ┌──────────┐    ┌──────────┐
│   Pi     │    │ ULN2003  │    │ Servo +  │
│          │    │ Drivers  │    │ Magnet   │
└──────────┘    └──────────┘    └──────────┘
      │                │                │
      └────────────────┴────────────────┘
                COMMON GROUND
```

**Pros**: Complete isolation, easy troubleshooting
**Cons**: More cables, more outlets needed

### Option 2: Single High-Current Supply (Compact)
```
┌─────────────────────────────────────┐
│         5V / 5A Power Supply         │
│         (e.g., MeanWell)            │
└─────────────────┬───────────────────┘
                  │
    ┌─────────────┼─────────────┐
    │             │             │
    ▼             ▼             ▼
┌───────┐   ┌─────────┐   ┌─────────┐
│  Pi   │   │ ULN2003 │   │ Servo + │
│(USB-C)│   │ Drivers │   │ Magnet  │
└───────┘   └─────────┘   └─────────┘
```

<!-- USER_ATTENTION: If using single supply, add bulk capacitor (1000µF) near motors -->

**Pros**: Simpler, fewer cables
**Cons**: Motor noise may affect Pi, need decoupling

### Option 3: Recommended Hybrid
```
┌────────────┐        ┌────────────────────┐
│ USB-C 5V/3A│        │    5V/3A PSU       │
│ (Pi only)  │        │ (Motors+Servo+Mag) │
└─────┬──────┘        └─────────┬──────────┘
      │                         │
      ▼                         ▼
┌──────────┐          ┌─────────────────────┐
│   Pi     │          │ Distribution Board  │
│          │          │  ├── ULN2003 A      │
│     GND ─┼──────────┼──├── ULN2003 B      │
│          │          │  └── Servo + Magnet │
└──────────┘          └─────────────────────┘
```

**Pros**: Pi isolated from motor noise, simpler than 3 supplies
**Cons**: Still needs common ground connection

---

## Decoupling Recommendations

### Near ULN2003 Drivers
- 100µF electrolytic capacitor across VCC and GND
- 0.1µF ceramic capacitor for high-frequency noise

### Near Servo
- 470µF electrolytic capacitor to handle current spikes

### Near Electromagnet
- Flyback diode (1N4007) across electromagnet terminals
- 100µF capacitor for switching noise

```
         (+) ───┬─── Electromagnet ───┬─── (-)
                │         ▲           │
                │    ┌────┴────┐      │
                │    │ 1N4007  │      │
                │    │ (diode) │      │
                │    └────┬────┘      │
                └─────────┴───────────┘
                          
                (Diode stripe toward +)
```

---

## Voltage Level Considerations

| Signal | Voltage | Notes |
|--------|---------|-------|
| Pi GPIO output | 3.3V | ULN2003 accepts 3.3V input |
| Pi GPIO input | 3.3V max! | Never apply 5V to GPIO |
| ULN2003 input | 3.3V or 5V | Works with Pi GPIO |
| Limit switches | Use pull-up | Signal is 0V or 3.3V |

> [!CAUTION]
> **Never connect 5V signals directly to Pi GPIO pins!**
> Use a voltage divider or level shifter if needed.

---

## Cable Gauge Reference

| Current Range | Minimum Gauge | Recommended |
|---------------|---------------|-------------|
| < 500mA | 26 AWG | 24 AWG |
| 500mA - 1A | 24 AWG | 22 AWG |
| 1A - 2A | 22 AWG | 20 AWG |
| 2A - 3A | 20 AWG | 18 AWG |
| 3A - 5A | 18 AWG | 16 AWG |

---

## Troubleshooting Power Issues

| Symptom | Possible Cause | Solution |
|---------|----------------|----------|
| Pi reboots randomly | Insufficient current | Use 3A supply, check cable |
| Motors stutter | Voltage drop | Add capacitor, use thicker wire |
| Servo jitters | Current spike | Add 470µF capacitor |
| Magnet weak | Low voltage | Check power supply output |
| Random GPIO behavior | Floating ground | Verify common ground |

---

*See [wiring.md](wiring.md) for connection diagrams.*
