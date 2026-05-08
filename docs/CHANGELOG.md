# Changelog

All notable changes to the Smart Chess Board project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- `test_clock_display.py` — TM1637 dual-display integration test via ROS topics (clock/display subtest)
- Corner-based obstacle routing (`_route_via_corner`) in `motion_planner_node.py` — BFS on 9×9 corner grid avoids bumping adjacent pieces when carrying the magnet
- `HomingNode`: `x_max_mm`, `steps_per_mm`, and all homing speeds now exposed as ROS parameters
- Chess OS: Reset Position button + `/api/stepper/reset_position` endpoint
- Chess OS: Live limit switch status with updated labels (X Home/Right, Y Home/Front)

### Changed
- **COORDINATE SYSTEM CHANGE**: Origin (0,0) is now **bottom-left** (from player's perspective).  
  Previously origin was at bottom-right (where limit switches are). New system:
  - **+X = rightward** toward h-file / X limit switch (right side)
  - **+Y = backward** toward rank 8 / camera tower (away from player)
  - **a1 ≈ (20, 20) mm**, **h1 ≈ (195, 20) mm**, **h8 ≈ (195, 195) mm**
- **X LIMIT SWITCH POSITION**: X limit is at **X_MAX** (right side, h-file side). Homing now drives in **+X** to find it, then backs off leftward to X=0 origin. ROS topic `/limit_switch/x_min` name unchanged; UI labels updated to "X Home (Right)".
- **Y LIMIT SWITCH POSITION**: Y limit at Y=0 (front, player's side) — unchanged, homing drives −Y.
- `homing_node.py`: X homing direction reversed (+X approach, −X backoff), Prusa precision approach uses `batch_size_prec=4` (≤1mm overshoot), drives gantry to (0,0) after both limits found, then resets stepper driver counter via `/stepper/reset_position`
- `motion_planner_node.py`: `_square_to_mm()` formula corrected to `x = origin_x + col * sq_size` (was `origin_x − col * sq_size`); default `board_origin_x_mm` updated from 200.0 → 20.0; `board_edge_safe_x_mm` updated from 5.0 → 230.0 (right of h-file)
- All documentation updated to reflect new coordinate system and limit switch locations


- `docs/features/vision-system.md` — comprehensive vision doc covering ROS perception stack, FEN visualizer (port 5000), MJPEG stream server (port 8080), calibration, and standalone scripts
- `setup/` directory for system configuration files (sudoers rule)
- `code/camera_stream_server.py` — MJPEG stream server consolidated into `code/`

### Changed
- `CLAUDE.md` — updated repo structure, corrected magnet description (permanent, not electromagnet), updated docs index
- `README.md` — fixed AGENTS.md reference (file is CLAUDE.md), updated Quick Start with correct paths and commands
- `deploy.md` — rewritten with clear Hardware / Perception / Full System sub-deployment sections; includes web port references
- `docs/CONTEXT.md` — updated file locations reference
- `docs/features/README.md` — updated to reference vision-system.md
- `PROJECT_STATUS.md` moved from root to `.agent/PROJECT_STATUS.md`
- `smart-chess-hw-tests.sudoers` moved from root to `setup/`

### Removed
- `docs/software/ros2_architecture_legacy.md` — superseded by current architecture docs
- `docs/vision_system_guide.md` — content merged into `docs/features/vision-system.md`
- `docs/features/vision-calibration.md` — content merged into `docs/features/vision-system.md`
- `src/chess_perception/scripts/PERCEPTION_TEST_GUIDE.md` — content moved to `docs/features/vision-system.md`
- `scripts/` directory (single file relocated to `code/`)

### Fixed
- Comprehensive agent-first documentation framework
- `AGENTS.md` as primary agent entry point
- `docs/` directory with hardware, software, and feature documentation
- `.agent/workflows/` for reusable task workflows
- ROS 2 package structure for all major subsystems
- Reorganized documentation into centralized `docs/` directory
- Gantry hardware test suite reorganized into focused diagnostics (`limits`, `pulse`, `motor_a`, `motor_b`, `corexy`, `speed_sweep`, `repeatability`, `enable_hold`, `homing`, `manual`, `full`)
- Manual gantry UI updated with live step-delay tuning and ramped stepping behavior
- Stepper driver defaults tuned for Linux userspace stability (longer DIR/STEP timing, safer minimum delay, acceleration ramp)
- Homing node now explicitly controls A4988 `ENABLE` pin
- Hardware docs aligned to A4988 + NEMA 11 architecture (removed ULN2003/28BYJ guidance in primary docs)
- Inconsistent clock servo pin documentation (`GPIO18` is now consistently documented)
- Outdated limit switch defaults in software docs

### Planned
<!-- USER_ATTENTION: Add your planned features here -->
- Motor calibration and testing
- Camera-based board detection
- Piece identification system
- Full game loop implementation

---

## [0.1.0] - 2026-01-20

### Added
- Initial ROS 2 package structure
- Basic node skeletons for all major components
- CoreXY kinematics design
- GPIO pin configuration template (`pins.yaml`)
- Software architecture documentation

### Hardware
- Selected 28BYJ-48 stepper motors with ULN2003 drivers
- Selected SG90 servo for Z-axis
- Raspberry Pi 4B as main controller
- RPi Camera Module v2 for vision

---

<!-- 
Template for new versions:

## [X.Y.Z] - YYYY-MM-DD

### Added
- New features

### Changed
- Changes to existing functionality

### Fixed
- Bug fixes

### Removed
- Removed features

### Hardware
- Hardware changes
-->
