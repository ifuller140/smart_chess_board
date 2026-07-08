# Feature Documentation

> **Deep-dive documentation for major system features.**

## Overview

This section contains detailed technical documentation for each major feature of the Smart Chess Board.

## Features

| Document | Feature | Status |
|----------|---------|--------|
| [corexy-gantry.md](corexy-gantry.md) | CoreXY motion system | ✅ Implemented — needs on-hardware calibration |
| [moving-logic.md](moving-logic.md) | Collision-aware piece movement (BFS corner routing) | ✅ Implemented — needs real-board verification |
| [vision-system.md](vision-system.md) | Camera calibration, perception stack, web interfaces | ✅ Implemented — needs on-hardware tuning |
| [piece-detection.md](piece-detection.md) | Computer vision pipeline (early design) | 🗄️ Superseded by vision-system.md |
| [game-logic.md](game-logic.md) | Chess engine integration, game state machine | ✅ Implemented |
| [magnet-system.md](magnet-system.md) | Permanent magnet piece pickup | ✅ Implemented — needs pulse-width calibration |

## Feature Status Legend

| Status | Meaning |
|--------|---------|
| ✅ Implemented | Code is complete; any remaining work is physical calibration/verification, not implementation |
| 🔄 In Development | Currently being implemented |
| 📋 Planned | Designed but not yet implemented |
| 🗄️ Superseded | Replaced by another doc — kept for historical/background reference only |
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

> Updated 2026-07-08 against actual code state (see `.agent/PROJECT_STATUS.md` for the live version of this list). Checked items are implemented in code; unchecked items still need real work, not just calibration.

### Phase 1: Hardware Validation
- [x] Motor control (stepper driver implemented — physical step/speed tuning still needed)
- [x] Servo + magnet testing (`servo_node`, `clock_servo_node` implemented)
- [x] Limit switch homing (Prusa-style, `homing_node` implemented — `x_max_mm` needs physical measurement)
- [x] Camera image capture + calibration (`camera_node` implemented — intrinsic calibration still needs running on the Pi)

### Phase 2: Core Motion
- [x] CoreXY kinematics (`gantry_kinematics_node`)
- [x] Homing sequence
- [x] Basic move commands (`MoveGantry` action)
- [x] Collision-aware path planning (BFS corner routing in `motion_planner_node` — needs verification with real pieces on the board)

### Phase 3: Perception
- [x] Board detection with perspective correction (`board_detector_node`)
- [x] Piece identification (`piece_detector_node`)
- [x] FEN generation

### Phase 4: Game Loop
- [x] State machine (`game_manager_node`)
- [x] Move detection
- [x] Engine integration (Stockfish via `chess_engine_node`, incl. difficulty setting)
- [x] Clock servo (hit button after computer move)

### Phase 5: Polish
- [x] Chess clock display (`clock_display_node`, dual TM1637)
- [ ] Error recovery (emergency stop exists; piece-drop/mis-detection recovery is still manual)
- [x] User interface (`chess_ui` ROS package — Chess OS architecture consolidation complete: see `.agent/PROJECT_STATUS.md`)

---

*See individual feature docs for implementation details.*

