# Power Distribution & Requirements

> Power budget for the current A4988 + NEMA 11 build.

## Recommended Supplies

| Rail | Voltage | Min Current | Purpose |
|------|---------|-------------|---------|
| Pi rail | 5V | 3A | Raspberry Pi 4B only |
| Motor rail | 12V | 2A | A4988 VMOT for both NEMA 11 steppers |
| Servo/Magnet rail | 5V | 2A | SG90 servos + electromagnet |

## Critical Notes

1. `A4988 VDD` is logic power (3.3V from Pi), `A4988 VMOT` is motor power (12V rail).
2. Do not route motor current through Pi.
3. Tie all grounds together (Pi GND + 12V GND + 5V actuator GND).
4. Add bulk capacitance near each A4988 VMOT input (typically >=100uF low-ESR).

## Estimated Consumption

| Component | Typical Current | Peak Current |
|-----------|-----------------|--------------|
| Raspberry Pi 4B + camera | 0.8A | 1.2A |
| NEMA 11 + A4988 (each) | 0.4A | 0.8A |
| SG90 servo (each) | 0.15A | 0.7A |
| Electromagnet | 0.4A | 0.5A |

## Troubleshooting Power Symptoms

| Symptom | Likely Cause | Action |
|---------|--------------|--------|
| Stepper buzzes/jitters with weak torque | Motor rail sag or current-limit too low | Verify VMOT under load; retune A4988 current limit |
| Random motion glitches | Missing common ground | Re-check ground bus continuity |
| Servo jitters when motors step | Shared noisy rail | Isolate servo/magnet rail and add decoupling |
| Pi brownout icon/reboots | Pi rail undersized | Use known-good 5V/3A USB-C PSU |

---

See [wiring.md](wiring.md) for connection details.
