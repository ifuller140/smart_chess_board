# AGENTS.md - Smart Chess Board

> **This file is the primary entry point for AI agents working on this project.**

## Project Summary

An automated chess board that plays physical chess against a human opponent. Uses a Raspberry Pi 4B running ROS 2 Humble to control a CoreXY gantry system with an electromagnet for moving pieces. Computer vision detects board state and a chess engine (Stockfish) calculates moves.

## Quick Start for Agents

1. **Read this file first** (you're doing it!)
2. **Read** [docs/CONTEXT.md](docs/CONTEXT.md) for full project context
3. **Check** [docs/hardware/pinout.md](docs/hardware/pinout.md) before any GPIO work
4. **Review** [docs/software/architecture.md](docs/software/architecture.md) for system design

## Repository Structure

```
smart_chess_board/
├── AGENTS.md                    # 👈 You are here
├── README.md                    # Human-readable project intro
├── docs/                        # 📚 All documentation
│   ├── CONTEXT.md              # Full project context for agents
│   ├── CHANGELOG.md            # Version history & decisions
│   ├── hardware/               # Hardware documentation
│   ├── software/               # Software architecture docs
│   └── features/               # Feature deep-dives
├── .agent/                      # Agent workflows & context
│   └── workflows/              # Reusable task workflows
├── src/                         # ROS 2 packages (main codebase)
│   ├── chess_hw_interface/     # GPIO drivers
│   ├── chess_perception/       # Computer vision
│   ├── chess_logic/            # Game rules & engine
│   └── gantry_control/         # Motion control
├── code/                        # Standalone test scripts
├── cad/                         # CAD files & exports
├── simulation/                  # Simulation environment
└── install/                     # Installation scripts
```

## Hardware Overview

| Component | Model | Purpose | Status |
|-----------|-------|---------|--------|
| Controller | Raspberry Pi 4B (4GB) | Main compute | ✅ Working |
| Stepper Motors | NEMA 11 + A4988 (×2) | CoreXY gantry X/Y | ⚠️ Testing |
| Z-Axis Servo | SG90 | Magnet lift | ⚠️ Testing |
| Clock Servo | SG90 | Hits clock button | ⚠️ Testing |
| Electromagnet | 5V DC | Piece pickup | ⚠️ Testing |
| Camera | RPi Camera Module v2 | Board detection | ⚠️ Testing |
| Limit Switches | Micro switches (×3) | Homing + clock hit | ⚠️ Testing |

<!-- USER_ATTENTION: Update status markers as components are verified -->

## Software Stack

- **OS**: Ubuntu 22.04 (Jammy) on Raspberry Pi
- **Framework**: ROS 2 Humble
- **Language**: Python 3 (rclpy)
- **Key Libraries**: `RPi.GPIO`, `opencv-python`, `python-chess`

## ROS 2 Packages

| Package | Purpose | Key Nodes |
|---------|---------|-----------|
| `chess_hw_interface` | Hardware drivers | `stepper_driver_node`, `servo_node`, `limit_switch_node` |
| `chess_perception` | Computer vision | `camera_node`, `board_detector_node`, `piece_detector_node` |
| `chess_logic` | Game management | `game_manager_node`, `chess_engine_node` |
| `gantry_control` | Motion control | `gantry_kinematics_node`, `motion_planner_node`, `homing_node` |

## Critical Constraints

> [!CAUTION]
> **Read these before making ANY changes:**

1. **Separate Power Supplies**: Motors MUST use separate 5V supply, not Pi's GPIO 5V
2. **BCM Pin Numbering**: All GPIO uses BCM numbering, NOT physical pin numbers
3. **GPIO Cleanup**: Always call `GPIO.cleanup()` on node shutdown
4. **Step Timing**: Stepper minimum delay is 0.001s between steps
5. **Camera Position**: Camera is 2" behind board, 7" above, 45° angle (requires perspective correction)

## Agent Task Guidelines

### Code Review Tasks
- Check GPIO pin conflicts in `src/chess_hw_interface/config/pins.yaml`
- Verify ROS 2 topic/service naming conventions
- Ensure proper error handling for hardware failures
- Validate state machine transitions in `game_manager_node`

### Feature Implementation Tasks
- Always update relevant docs in `docs/` when adding features
- Add new parameters to `pins.yaml` or appropriate config file
- Create unit tests for logic-heavy code
- Update `CHANGELOG.md` with significant changes

### Hardware Modification Tasks
- ALWAYS update `docs/hardware/pinout.md` when changing GPIO
- Update wiring diagrams in `docs/hardware/wiring.md`
- Verify power budget in `docs/hardware/power.md`
- Test with standalone scripts in `code/` before ROS integration

## Planned Features (Future Work)

<!-- USER_ATTENTION: Update this list with your prioritized feature roadmap -->

- [ ] Basic motor control calibration
- [ ] Camera-based board detection
- [ ] Piece identification (by color/size)
- [ ] Full game loop (detect move → respond)
- [ ] Chess clock integration (servo hits clock after computer move)
- [ ] Voice feedback system
- [ ] Web interface for game monitoring
- [ ] PGN game export

## Hardware Testing Suite

Run hardware tests to validate components:

```bash
# On Raspberry Pi
cd ~/smart_chess_ws/src/smart_chess_board

# List available tests
python3 -m chess_hw_interface.testing.test_runner --list

# Run all tests
python3 -m chess_hw_interface.testing.test_runner --all

# Run specific test category
python3 -m chess_hw_interface.testing.test_runner --test gantry
python3 -m chess_hw_interface.testing.test_runner --test servo
python3 -m chess_hw_interface.testing.test_runner --test camera
python3 -m chess_hw_interface.testing.test_runner --test magnet
python3 -m chess_hw_interface.testing.test_runner --test clock

# Mock mode (no real GPIO, for development)
python3 -m chess_hw_interface.testing.test_runner --mock --all
```

Tests use the clock display for feedback and clock button for user confirmation.

## Known Issues

<!-- USER_ATTENTION: Add known bugs and limitations here -->

1. **Motor pins are placeholders** - `pins.yaml` needs actual BCM numbers
2. **Camera position undefined** - Need exact mounting coordinates
3. **Homing sequence untested** - Limit switch positions TBD

## Documentation Index

| Document | Purpose | When to Read |
|----------|---------|--------------|
| [CONTEXT.md](docs/CONTEXT.md) | Full project context | First time on project |
| [hardware/pinout.md](docs/hardware/pinout.md) | GPIO assignments | Any GPIO work |
| [hardware/components.md](docs/hardware/components.md) | Component specs | Hardware questions |
| [software/architecture.md](docs/software/architecture.md) | System design | Understanding data flow |
| [software/nodes.md](docs/software/nodes.md) | Node reference | Modifying/adding nodes |
| [features/corexy-gantry.md](docs/features/corexy-gantry.md) | Motion system | Motion control work |
| [features/moving-logic.md](docs/features/moving-logic.md) | Piece movement planning | Collision avoidance |
| [features/vision-calibration.md](docs/features/vision-calibration.md) | Camera calibration | Vision work |

## 🤖 Specialized Agent Roles

To work simultaneously in harmony, assign agents to these specific scopes:

### 1. Hardware Agent
**Scope**: `src/chess_hw_interface`, `src/gantry_control`
**Responsibilities**:
- Implement low-level GPIO drivers
- Tune kinematics and motion planning
- Verify hardware safety limits

### 2. Logic Agent
**Scope**: `src/chess_logic`
**Responsibilities**:
- Implement Chess Engine integration (Stockfish)
- Manage Game State Machine (IDLE -> MOVING -> etc)
- Handle game rules and turn validation

### 3. Perception Agent
**Scope**: `src/chess_perception`
**Responsibilities**:
- Computer Vision pipeline (OpenCV)
- Board state detection (FEN generation)
- Camera calibration and perspective transforms

### 4. System Architect
**Scope**: `setup.py`, `package.xml`, `launch/`, `docs/`
**Responsibilities**:
- Maintain build system and dependencies
- Define standard Interfaces (`.msg`, `.srv`)
- Orchestrate high-level system launch


## Workflow Files

These workflows in `.agent/workflows/` provide step-by-step instructions:

- `hardware-test.md` - Testing individual hardware components
- `deploy.md` - Deploying to Raspberry Pi
- `code-review.md` - Code review checklist

---

*Last updated: 2026-02-08*
<!-- USER_ATTENTION: Update this date when making significant changes -->
