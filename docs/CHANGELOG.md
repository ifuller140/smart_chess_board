# Changelog

All notable changes to the Smart Chess Board project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).

## [Unreleased]

### Added
- `src/chess_ui/` — Chess OS converted from `code/chess_os.py` (a single 3,336-line file) into a real ROS package: `state.py`/`camera.py`/`ros_client.py`/`routes.py`/`app.py` + `templates/index.html`, installable via colcon (`ros2 run chess_ui chess_ui`), launchable from `full_system_launch.py`. All 41 routes and the ROS client node carried over unchanged. Live-verified on the Pi side-by-side against the old file before cutover: byte-identical index page, identical `/api/status` shape, identical hardware-test and calibration behavior (Step 4 of the Chess OS architecture initiative).
- `chess_interfaces/action/RunHardwareTest` and a new `test_runner_node` in `chess_hw_interface`, exposed at `/hw_test/run` — ROS-native hardware test orchestration replacing Chess OS's in-process `_TestRunner` subprocess management. Live-verified on the Pi: full chain from `/api/tests/run` through the ROS action to `run_hw_test.sh` and back, feedback streamed line-by-line (Step 3 of the Chess OS architecture initiative).
- Chess OS: `api_calib_apply` now pushes calibrated board geometry (`board_origin_x_mm`, `board_origin_y_mm`, `square_size_mm`) to `motion_planner_node` live via ROS `SetParameters` — previously the calibration UI only updated chess_os's own local `board_calibration.json`, never the parameters that actually drive gantry moves during real games. Live-verified on the Pi via a synthetic `/gantry/pose` publish (Step 2 of the Chess OS architecture initiative).
- Chess engine difficulty setting (Stockfish skill level) exposed from `chess_engine_node.py` and Chess OS
- Pawn promotion handler in `game_manager_node.py` / Chess OS (promotion banner + piece-choice UI)
- Chess OS: gantry calibration tests wired into the Tests tab
- `docs/software/nodes.md` and `.agent/PROJECT_INTEGRATION_PLAN.md` — full Chess OS integration reference and 5-phase implementation plan (game control backend, Chess OS API, Game Tab UI, Perception tab, polish)
- Bench-test scripts for the Z-axis magnet servo (`code/test_z_servo.py`) — standalone pigpio sanity check ahead of physical calibration
- `test_clock_display.py` — TM1637 dual-display integration test via ROS topics (clock/display subtest)
- Corner-based obstacle routing (`_route_via_corner`) in `motion_planner_node.py` — BFS on 9×9 corner grid avoids bumping adjacent pieces when carrying the magnet
- `HomingNode`: `x_max_mm`, `steps_per_mm`, and all homing speeds now exposed as ROS parameters
- Chess OS: Reset Position button + `/api/stepper/reset_position` endpoint
- Chess OS: Live limit switch status with updated labels (X Home/Right, Y Home/Front)
- `docs/features/vision-system.md` — comprehensive vision doc covering ROS perception stack, FEN visualizer (port 5000), MJPEG stream server (port 8080), calibration, and standalone scripts
- `setup/` directory for system configuration files (sudoers rule)
- `code/camera_stream_server.py` — MJPEG stream server consolidated into `code/`

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
- Interface definitions consolidated into a single `chess_interfaces` package (`BoardState.msg`, `RequestMove.srv`, `MoveGantry.action`) — the "unified interfaces package" goal from `.agent/PROJECT_STATUS.md` is satisfied under this name, not the originally proposed `smart_chess_interfaces`
- All documentation updated to reflect new coordinate system and limit switch locations
- `CLAUDE.md` — updated repo structure, corrected magnet description (permanent, not electromagnet), updated docs index, fixed `chess_interfaces` package name (previously incorrectly listed as `gantry_control_interfaces`)
- `README.md` — fixed AGENTS.md reference (file is CLAUDE.md), updated Quick Start with correct paths and commands, fixed `chess_interfaces` package name
- `deploy.md` — rewritten with clear Hardware / Perception / Full System sub-deployment sections; includes web port references; fixed package build order naming
- `docs/CONTEXT.md` — updated file locations reference; removed references to `cv_params.yaml`/`calibration.npz` (neither file exists — CV params are ROS parameters, not a yaml file)
- `docs/features/README.md` — updated to reference vision-system.md; status badges corrected to reflect actual implementation state
- `docs/software/interfaces.md` — rewritten to describe the actual `chess_interfaces` package instead of stale pre-consolidation per-package ownership
- `docs/features/magnet-system.md` and all hardware docs (`wiring.md`, `power.md`, `mechanical.md`, `hardware/README.md`, `software/nodes.md`) — corrected from a described electromagnet design (GPIO + transistor circuit) to the actual permanent-magnet, servo-only design
- `PROJECT_STATUS.md` moved from root to `.agent/PROJECT_STATUS.md`
- `smart-chess-hw-tests.sudoers` moved from root to `setup/`

### Removed
- `code/gantry_calibration.py`, `code/hardware_test.py`, `code/calibration_verify.py`, `code/square.py`, `code/ServoTestController.py` — all superseded by ROS equivalents (`gantry_control` package, `chess_hw_interface` test suite via `test_runner_node`). `ServoTestController.py` turned out to be unrelated dead code from the pre-NEMA11/A4988 hardware iteration (a 28BYJ-48/ULN2003 stepper driver script, not a servo at all) that called an undefined function and would crash if run. This closes out Step 5, the last step of the Chess OS architecture consolidation initiative.
- `code/chess_os.py` — fully superseded by the `chess_ui` ROS package (see Added above); deleted after live side-by-side verification on the Pi confirmed identical behavior.
- (Step 1, superseded by the full rewrite above but recorded for history) `code/chess_os.py`'s duplicated local vision/FEN pipeline (`_detect_pieces`, `_fen_from_detection`, `_render_warp`/`_render_raw`, `_process_one`, `_vision_loop`, `_force_reprocess`, `_undistort`, `_warp`, `_load_corners`/`_save_corners`) and its UI (manual corner-drag calibration, warp MJPEG stream, blob-detection/lens-correction param sliders, overlay toggles). `board_detector_node` already does automatic corner detection and `piece_detector_node` already does real diff-based piece detection; chess_os now only displays their output (`/api/diff_frame`, `/api/square_scores`) plus the raw camera feed, and reads FEN solely from `/game_manager/board_fen`.
- `src/chess_perception_upload/` — dead, colcon-ignored duplicate of `chess_perception` with a colliding package name
- Orphaned duplicate interface files not used by any build: `gantry_control/action/MoveGantry.action`, `chess_logic/srv/RequestMove.srv`
- Hardcoded sudo password fallback (`fuller`) in `run_hw_test.sh` and `code/fix_video_permissions.sh` — now requires `CHESS_SUDO_PASS` env var or passwordless sudo
- `docs/software/ros2_architecture_legacy.md` — superseded by current architecture docs
- `docs/vision_system_guide.md` — content merged into `docs/features/vision-system.md`
- `docs/features/vision-calibration.md` — content merged into `docs/features/vision-system.md`
- `src/chess_perception/scripts/PERCEPTION_TEST_GUIDE.md` — content moved to `docs/features/vision-system.md`
- `scripts/` directory (single file relocated to `code/`)

### Fixed
- Chess OS: the manual corner-drag override UI (added in Phase 7/8e) was non-functional — the drag `<canvas>` was set up while its tab was still `display:none`, leaving it permanently sized 0×0, so mousedown/drag fell through to the raw `<img>` underneath and triggered the browser's native image-drag instead of moving a corner handle. Canvas setup now re-runs when the Perception tab is opened. Manual overrides are now persisted to `manual_corners.json` (repo root) by `chess_ui` and reloaded independently by both `chess_ui` and `board_detector_node` at startup, so a corrected corner set survives restarts and is applied by default instead of always reverting to automatic detection.
- Chess OS: the manual corner-drag UI was *still* non-functional after the fix above — `/api/snapshot` (the drag background) returned a bare `204 No Content` whenever no camera frame was buffered yet (e.g. at page load, before the pipeline's first frame), which never fires the image's `load` event, leaving `naturalWidth` at `0` forever and the drag canvas permanently unsized. `/api/snapshot` now always returns a decodable placeholder JPEG (matching `/api/diff_frame`/`/api/corner_frame`), and the drag background is now refreshed on the same live-image interval as those endpoints instead of being set once and never updated.
- Chess OS: `_RosNode` subscribed to the uncompressed `/camera/image_raw` topic, but the real camera backend (`camera_ros`) never actually emits frames on it — only `/camera/image_raw/compressed`. The raw camera preview had likely been silently dead over ROS. Switched to the compressed topic (same one `board_detector_node`/`piece_detector_node` already use) and decode with `cv2.imdecode`.
- Chess OS: `api_capture_reference` always reported success (`session_reference_captured=True`) regardless of whether the underlying `/perception/capture_premove` service call actually succeeded — it only checked whether the Flask response was a tuple (the ROS-not-connected case), never the `ok` field. Now checks the actual result.
- License declarations (`TODO: License declaration` → `MIT`, matching the root `LICENSE` file) across all `package.xml`/`setup.py` files
- `game_manager_node.py` / `chess_engine_node.py` — the "In a real app, we'd..." mock comments flagged in earlier audits are gone; both are now real implementations (state machine + Stockfish integration)
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
- Physical calibration verification (`x_max_mm`, `board_origin_x/y_mm`) on real hardware
- Corner-routing BFS fallback verification with real pieces on the board
- Chess OS / `code/` architecture consolidation — see the initiative in `.agent/PROJECT_STATUS.md`

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
