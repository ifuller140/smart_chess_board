# AGENTS.md - Smart Chess Board

> **This file is the primary entry point for AI agents working on this project.**
> Claude Code loads this file automatically as `CLAUDE.md`.

## Project Summary

An automated chess board that plays physical chess against a human opponent. Uses a Raspberry Pi 4B running ROS 2 Humble to control a CoreXY gantry system with a permanent magnet for moving pieces. Computer vision detects board state and Stockfish calculates moves.

## Quick Start for Agents

1. **Read this file first** (you're doing it!)
2. **Read** [docs/CONTEXT.md](docs/CONTEXT.md) for full project context
3. **Check** [docs/hardware/pinout.md](docs/hardware/pinout.md) before any GPIO work
4. **Review** [docs/software/architecture.md](docs/software/architecture.md) for system design

## Repository Structure

```
smart_chess_board/
├── CLAUDE.md                    # 👈 You are here (agent entry point)
├── README.md                    # Human-readable project intro
├── run_hw_test.sh               # Hardware test runner (wraps sudo + ROS env)
├── docs/                        # 📚 All documentation
│   ├── CONTEXT.md              # Full project context for agents
│   ├── CHANGELOG.md            # Version history & decisions
│   ├── hardware/               # Hardware documentation
│   ├── software/               # Software architecture docs
│   └── features/               # Feature deep-dives
├── .agent/                      # Agent workflows & context
│   ├── workflows/              # Reusable task workflows
│   └── PROJECT_STATUS.md       # To-do tracker and strategic initiatives
├── src/                         # ROS 2 packages (main codebase)
│   ├── chess_hw_interface/     # GPIO drivers + hardware test suite
│   ├── chess_perception/       # Computer vision nodes
│   ├── chess_logic/            # Game rules & Stockfish engine
│   ├── gantry_control/         # Motion control & kinematics
│   └── gantry_control_interfaces/ # ROS 2 action/srv definitions (CMake)
├── code/                        # Standalone scripts (calibration, testing, streaming)
├── setup/                       # System configuration files (sudoers, etc.)
├── cad/                         # CAD files & exports
└── simulation/                  # Simulation environment
```

## Hardware Overview

| Component | Model | Purpose | Status |
|-----------|-------|---------|--------|
| Controller | Raspberry Pi 4B (4GB) | Main compute | ✅ Working |
| Stepper Motors | NEMA 11 + A4988 (×2) | CoreXY gantry X/Y | ⚠️ Testing |
| Z-Axis Servo | SG90 | Magnet lift | ⚠️ Testing |
| Clock Servo | SG90 | Hits clock button | ⚠️ Testing |
| Permanent Magnet | — | Piece pickup (servo-lowered) | ⚠️ Testing |
| Camera | RPi Camera Module v2 | Board detection | ⚠️ Testing |
| Limit Switches | Micro switches (×3) | Homing + clock hit | ⚠️ Testing |

## Software Stack

- **OS**: Ubuntu 22.04 (Jammy) on Raspberry Pi
- **Framework**: ROS 2 Humble
- **Language**: Python 3 (rclpy)
- **Key Libraries**: `RPi.GPIO`, `pigpio`, `opencv-python`, `python-chess`, `flask`

## ROS 2 Packages

| Package | Purpose | Key Nodes |
|---------|---------|-----------|
| `chess_hw_interface` | Hardware drivers | `stepper_driver_node`, `servo_node`, `limit_switch_node` |
| `chess_perception` | Computer vision | `camera_node`, `board_detector_node`, `piece_detector_node` |
| `chess_logic` | Game management | `game_manager_node`, `chess_engine_node` |
| `gantry_control` | Motion control | `gantry_kinematics_node`, `motion_planner_node`, `homing_node` |
| `gantry_control_interfaces` | ROS 2 interfaces | `MoveGantry.action`, `RequestMove.srv` |

## Critical Constraints

> [!CAUTION]
> **Read these before making ANY changes:**

1. **Separate Power Supplies**: Motors MUST use separate 12V/5V supply, NOT Pi's GPIO 5V
2. **BCM Pin Numbering**: All GPIO uses BCM numbering, NOT physical pin numbers
3. **GPIO Cleanup**: Always call `GPIO.cleanup()` on node shutdown
4. **Step Timing**: Stepper minimum delay is 0.001s between steps
5. **Camera Position**: Camera is 2" behind board, 7" above, 45° angle (requires perspective correction)
6. **Magnet is permanent**: No GPIO power control — servo position alone controls pickup/release

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
- Update `docs/CHANGELOG.md` with significant changes

### Hardware Modification Tasks
- ALWAYS update `docs/hardware/pinout.md` when changing GPIO
- Update wiring diagrams in `docs/hardware/wiring.md`
- Verify power budget in `docs/hardware/power.md`
- Test with standalone scripts in `code/` before ROS integration

## Hardware Testing Suite

```bash
# On Raspberry Pi — from project root
cd ~/dev/smart_chess_board

# Preferred: run_hw_test.sh handles ROS env + sudo for /dev/mem access
./run_hw_test.sh --list
./run_hw_test.sh --category gantry --subtest manual
./run_hw_test.sh --category gantry --subtest full

# Vision-specific tests
./run_hw_test.sh --test vision_fen
./run_hw_test.sh --test vision_board
```

See `.agent/workflows/hardware-test.md` for the full test suite reference.

## Known Issues

1. **Motor pins are placeholders** — `pins.yaml` needs actual BCM numbers verified against physical wiring
2. **Camera position** — Exact mounting coordinates need hardware measurement
3. **Homing sequence untested** — Limit switch positions TBD

## Documentation Index

| Document | Purpose | When to Read |
|----------|---------|--------------|
| [docs/CONTEXT.md](docs/CONTEXT.md) | Full project context | First time on project |
| [docs/hardware/pinout.md](docs/hardware/pinout.md) | GPIO assignments | Any GPIO work |
| [docs/hardware/components.md](docs/hardware/components.md) | Component specs | Hardware questions |
| [docs/software/architecture.md](docs/software/architecture.md) | System design | Understanding data flow |
| [docs/software/nodes.md](docs/software/nodes.md) | Node reference | Modifying/adding nodes |
| [docs/features/corexy-gantry.md](docs/features/corexy-gantry.md) | Motion system | Motion control work |
| [docs/features/moving-logic.md](docs/features/moving-logic.md) | Piece movement planning | Collision avoidance |
| [docs/features/vision-system.md](docs/features/vision-system.md) | Camera calibration, perception stack, web ports | All vision work |

## Specialized Agent Roles

### 1. Hardware Agent
**Scope**: `src/chess_hw_interface`, `src/gantry_control`
- Implement low-level GPIO drivers
- Tune kinematics and motion planning
- Verify hardware safety limits

### 2. Logic Agent
**Scope**: `src/chess_logic`
- Implement Stockfish integration
- Manage Game State Machine (IDLE → MOVING → etc.)
- Handle game rules and turn validation

### 3. Perception Agent
**Scope**: `src/chess_perception`
- Computer vision pipeline (OpenCV)
- Board state detection (FEN generation)
- Camera calibration and web interfaces (port 5000, 8080)

### 4. System Architect
**Scope**: `setup.py`, `package.xml`, `launch/`, `docs/`
- Maintain build system and dependencies
- Define standard interfaces (`.msg`, `.srv`, `.action`)
- Orchestrate high-level system launch

## Workflow Files

`.agent/workflows/` — step-by-step task guides:

- `deploy.md` — Full deploy with hardware/perception/full-system sub-sections
- `hardware-test.md` — Hardware component testing
- `code-review.md` — Code review checklist
- `PROJECT_STATUS.md` — To-do tracker and strategic initiatives

---

*Last updated: 2026-05-03*
