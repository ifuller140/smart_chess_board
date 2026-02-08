# Changelog

All notable changes to the Smart Chess Board project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- Comprehensive agent-first documentation framework
- `AGENTS.md` as primary agent entry point
- `docs/` directory with hardware, software, and feature documentation
- `.agent/workflows/` for reusable task workflows
- ROS 2 package structure for all major subsystems

### Changed
- Reorganized documentation into centralized `docs/` directory
- Gantry hardware test suite reorganized into focused diagnostics (`limits`, `pulse`, `motor_a`, `motor_b`, `corexy`, `speed_sweep`, `repeatability`, `enable_hold`, `homing`, `manual`, `full`)
- Manual gantry UI updated with live step-delay tuning and ramped stepping behavior
- Stepper driver defaults tuned for Linux userspace stability (longer DIR/STEP timing, safer minimum delay, acceleration ramp)
- Homing node now explicitly controls A4988 `ENABLE` pin
- Hardware docs aligned to A4988 + NEMA 11 architecture (removed ULN2003/28BYJ guidance in primary docs)

### Fixed
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
