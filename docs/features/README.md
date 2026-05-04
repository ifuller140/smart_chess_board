# Feature Documentation

> **Deep-dive documentation for major system features.**

## Overview

This section contains detailed technical documentation for each major feature of the Smart Chess Board.

## Features

| Document | Feature | Status |
|----------|---------|--------|
| [corexy-gantry.md](corexy-gantry.md) | CoreXY motion system | 🔄 In Development |
| [moving-logic.md](moving-logic.md) | Collision-aware piece movement | 🔄 In Development |
| [vision-system.md](vision-system.md) | Camera calibration, perception stack, web interfaces | 🔄 In Development |
| [piece-detection.md](piece-detection.md) | Computer vision pipeline | 📋 Planned |
| [game-logic.md](game-logic.md) | Chess engine integration | 📋 Planned |
| [magnet-system.md](magnet-system.md) | Electromagnet piece pickup | 📋 Planned |

## Feature Status Legend

| Status | Meaning |
|--------|---------|
| ✅ Complete | Fully implemented and tested |
| 🔄 In Development | Currently being implemented |
| 📋 Planned | Designed but not yet implemented |
| ⚠️ Blocked | Waiting on dependencies |

## Hardware Testing Suite

Before implementing features, validate hardware with the testing suite:

```bash
# Run all hardware tests
python3 -m chess_hw_interface.testing.test_runner --all

# Run specific category
python3 -m chess_hw_interface.testing.test_runner --category gantry --subtest full
```

**Available test categories:** gantry, servo, camera, magnet, clock (with gantry subtests)

See [CLAUDE.md](../../CLAUDE.md#hardware-testing-suite) for full CLI reference.

## Feature Roadmap

<!-- USER_ATTENTION: Update this roadmap with your priorities -->

### Phase 1: Hardware Validation
- [ ] Motor control (stepper calibration)
- [ ] Servo + magnet testing
- [ ] Limit switch homing (Prusa-style)
- [ ] Camera image capture + calibration

### Phase 2: Core Motion
- [ ] CoreXY kinematics
- [ ] Homing sequence
- [ ] Basic move commands
- [ ] Collision-aware path planning

### Phase 3: Perception
- [ ] Board detection with perspective correction
- [ ] Piece identification
- [ ] FEN generation

### Phase 4: Game Loop
- [ ] State machine
- [ ] Move detection
- [ ] Engine integration
- [ ] Clock servo (hit button after computer move)

### Phase 5: Polish
- [ ] Chess clock display
- [ ] Error recovery
- [ ] User interface

---

*See individual feature docs for implementation details.*

