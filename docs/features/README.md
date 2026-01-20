# Feature Documentation

> **Deep-dive documentation for major system features.**

## Overview

This section contains detailed technical documentation for each major feature of the Smart Chess Board.

## Features

| Document | Feature | Status |
|----------|---------|--------|
| [corexy-gantry.md](corexy-gantry.md) | CoreXY motion system | In Development |
| [piece-detection.md](piece-detection.md) | Computer vision pipeline | Planned |
| [game-logic.md](game-logic.md) | Chess engine integration | Planned |
| [magnet-system.md](magnet-system.md) | Electromagnet piece pickup | Planned |

## Feature Status Legend

| Status | Meaning |
|--------|---------|
| ✅ Complete | Fully implemented and tested |
| 🔄 In Development | Currently being implemented |
| 📋 Planned | Designed but not yet implemented |
| ⚠️ Blocked | Waiting on dependencies |

## Feature Roadmap

<!-- USER_ATTENTION: Update this roadmap with your priorities -->

### Phase 1: Hardware Validation
- [ ] Motor control (stepper calibration)
- [ ] Servo + magnet testing
- [ ] Limit switch homing
- [ ] Camera image capture

### Phase 2: Core Motion
- [ ] CoreXY kinematics
- [ ] Homing sequence
- [ ] Basic move commands

### Phase 3: Perception
- [ ] Board detection
- [ ] Piece identification
- [ ] FEN generation

### Phase 4: Game Loop
- [ ] State machine
- [ ] Move detection
- [ ] Engine integration

### Phase 5: Polish
- [ ] Chess clock
- [ ] Error recovery
- [ ] User interface

---

*See individual feature docs for implementation details.*
