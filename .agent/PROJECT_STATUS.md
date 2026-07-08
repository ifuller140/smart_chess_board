# Smart Chess Board - Project Status & Master Plan

> **The Central Brain**: This document tracks the heartbeat of the project. All agents should consult and update this file.
>
> Last full audit: 2026-07-08. Previous version of this file (dated 2026-05-08) had drifted significantly from reality — several "to-do" items below had already been completed in commits between 2026-05-08 and 2026-07-01, and the "In a real app, we'd..." mock-code concern in `game_manager_node.py` no longer applies. Re-verify anything below against the actual code before assuming it's stale again; this file decays fast.

## Master Status List

### ✅ Working / Done

- **All 14 ROS nodes** across `chess_hw_interface`, `gantry_control`, `chess_perception`, `chess_logic` are fully implemented — no stubs, `NotImplementedError`, or mock bodies remain.
- **Hardware layer** (`chess_hw_interface`): `stepper_driver_node` (pigpio DMA wave chains), `servo_node` (magnet lift), `clock_servo_node`, `clock_display_node` (dual TM1637), `limit_switch_node`, `gpio_watchdog_node`.
- **Gantry layer** (`gantry_control`): `gantry_kinematics_node` (CoreXY, trapezoidal profile, `MoveGantry` action server), `homing_node` (Prusa-style fast→backoff→precision, corrected coordinate system), `motion_planner_node` (BFS corner routing implemented, not just scaffolded).
- **Perception layer** (`chess_perception`): `camera_node` (picamera2/gstreamer/v4l2 backends), `board_detector_node`, `piece_detector_node` (score-based diff detection, clump filtering).
- **Logic layer** (`chess_logic`): `game_manager_node` (775-line real state machine — move validation, capture/castling/en passant/promotion handling, service-based start/new-game/resign), `chess_engine_node` (real Stockfish integration via `python-chess`, skill-level/difficulty setting, random-move fallback if engine unavailable).
- **Interfaces**: consolidated into one package, `chess_interfaces` (`BoardState.msg`, `RequestMove.srv`, `MoveGantry.action`). This satisfies the old "create a unified `smart_chess_interfaces` package" initiative below — it happened, just under a different name. Orphaned duplicate `.action`/`.srv` files in `gantry_control`/`chess_logic` and the dead `chess_perception_upload` package have been removed (2026-07-08 cleanup).
- **Master launch file**: `src/launch/full_system_launch.py` brings up all 4 layers in dependency order.
- **Chess OS** (`code/chess_os.py`): functionally complete as a UI/control surface — game start/new/resign, promotion banner, engine difficulty setting, board calibration workflow, live gantry/hardware status, hardware test runner integration, node health panel. **Architecturally it needs rework** — see the initiative below.
- **Documentation**: refreshed 2026-07-08 (this pass) — package naming, interfaces doc, magnet system description (permanent, not electromagnet — this error had spread into 5 hardware docs), CONTEXT.md file references, CHANGELOG, feature status badges all corrected.

### ⚠️ Partially Working / Needs Physical Verification Only

Logic is believed correct; these need to be proven on the real board, not redesigned. Re-verify each line against current code before trusting it — some items on this list in past sessions have since been resolved.

- `x_max_mm` and `board_origin_x_mm`/`board_origin_y_mm` in `homing_node`/`motion_planner_node` — defaults are placeholders, need physical measurement.
- Corner-routing BFS fallback in `motion_planner_node` — implemented, untested with real pieces obstructing a path.
- Chess OS's 3 game-control services (`/game/start`, `/game/new_game`, `/game/resign`) — wired end-to-end in code, not yet confirmed live on the Pi.
- Chess OS node-health panel — depends on exact ROS node name matching; unverified.
- Pre-game checklist gate (4 conditions) in Chess OS — implemented, needs a live run-through.
- Promotion and game-over banners — implemented in both `game_manager_node` and Chess OS UI; needs a real promotion/checkmate to confirm the UI actually surfaces correctly.
- Camera intrinsic calibration (`calibration.npz`) — the capture/calibration scripts exist but the output file has never been generated; needs to be run once on the Pi with the actual camera.
- `board_detector_node`'s automatic corner detection — live-verified 2026-07-08 that it runs and publishes `/perception/board_geometry` at a steady 2Hz once given a healthy camera feed, but during this session it never actually found a board (topic stayed empty even with a fresh camera_node) — likely the camera wasn't pointed at a physical board at the time, but worth re-checking with the rig actually assembled before trusting it end-to-end.

### 🔴 Architecturally Broken / Needs Rework

- **`code/chess_os.py` (3,336 lines, was 3,815)** — runs as a standalone Flask process, not a ROS-installable package. As of 2026-07-08 the duplicated vision/FEN pipeline is gone (see Step 1 below); it still duplicates `square_to_mm()` coordinate math (duplicates `gantry_kinematics_node`) and does subprocess-based hardware-test orchestration coupled to `run_hw_test.sh` instead of a ROS-native service. Most of the file is one inline HTML/JS template mixed into the same file as Flask routes and ROS client code. **This is the current top priority** — see the initiative below.
- **`code/` standalone scripts** (`gantry_calibration.py`, `hardware_test.py`, `calibration_verify.py`, `square.py`) duplicate ROS-side homing/motion-planning/testing logic with hardcoded GPIO pins that can drift out of sync with `pins.yaml`. To be deleted once each one's ROS-side equivalent is confirmed working on the Pi (see Chess OS initiative).

---

## 🧠 Strategic Initiatives

### 1. Chess OS / `code/` Architecture Consolidation — CURRENT TOP PRIORITY

**Problem**: `chess_os.py` grew from "a UI" into a monolith that reimplements vision detection, gantry coordinate math, and calibration/homing logic that already exists correctly inside the ROS packages. It isn't installable as a ROS package and isn't launched by `full_system_launch.py`.

**Direction**: convert Chess OS into a proper ROS package (e.g. `chess_ui`) — installable via colcon, launchable alongside everything else — and strip it down to a thin UI/API layer that only ever reads from ROS topics/services/actions, never recomputes its own vision or coordinates.

**Plan**: do this one concern at a time, verifying on the Pi after each step, not as one big rewrite:
1. Delete the duplicated vision pipeline (`_detect_pieces`, `_fen_from_detection`, `_opencv_camera_loop`) — display frames/FEN from `chess_perception` topics only.
2. Delete the duplicated `square_to_mm()` — resolve squares via the `MoveGantry` action instead.
3. Replace `_TestRunner` subprocess orchestration with a ROS-native service/action in `chess_hw_interface`.
4. Split the file into modules (`app.py`, `ros_client.py`, `templates/`, `state.py`) and move it into a real `src/chess_ui/` package with `package.xml`/`setup.py`.
5. Once each `code/` standalone script's ROS equivalent is confirmed superseding it, delete the script (see the Architecturally Broken list above).

- [x] Step 1: remove duplicated vision pipeline from chess_os.py — done and **live-verified on the Pi** 2026-07-08. Removed `_detect_pieces`/`_fen_from_detection`/`_render_warp`/`_render_raw`/`_process_one`/`_vision_loop`/`_force_reprocess`/`_undistort`/`_warp`/`_load_corners`/`_save_corners` and the manual corner-drag UI, warp MJPEG stream, and local blob-detection/lens-correction param sliders (~480 lines net). `board_detector_node` already does automatic corner detection (no manual dragging needed) and `piece_detector_node` already does the real diff-based detection — chess_os now only displays their output (`/api/diff_frame`, `/api/square_scores`) and the raw camera feed, and reads FEN solely from `/game_manager/board_fen`. **Kept** `_opencv_camera_loop` (the `--no-ros` standalone raw-frame fallback used for UI dev without ROS) since it doesn't duplicate detection, only camera capture.
  - Follow-on fix found during live verification: `_RosNode` subscribed to the uncompressed `/camera/image_raw` topic, but `camera_ros` (the real camera backend) never actually emits frames on it — only `/camera/image_raw/compressed`. chess_os's raw camera preview had likely been silently dead over ROS for a while. Switched the subscription to the compressed topic (same one `board_detector_node`/`piece_detector_node` already use) and decode with `cv2.imdecode`. Confirmed live: `cam_info` now reports `640×480 jpeg`, `/api/snapshot` returns a real ~49KB JPEG, `/api/diff_frame` and `/api/square_scores` return live data from `piece_detector_node`.
  - Also found and cleaned up (unrelated to this refactor, pure ops hygiene): 3 duplicate `perception_launch.py` process trees had been running since 2026-07-02, pegging the Pi 4's CPU at ~330% combined across 6 redundant `board_detector_node`/`piece_detector_node` processes; only one had a working `camera_node`. Killed the two dead duplicates, then restarted the surviving stack fresh (a stale long-running `camera_node` was also found to stop serving *new* subscribers — a fresh instance serves them fine at 30Hz — worth remembering if the camera ever appears to "stop working" for new consumers again after days of uptime).
  - New bug found (pre-existing, not caused by this refactor): `api_capture_reference` in chess_os.py always sets `session_reference_captured=True`/`"Reference captured ✓"` regardless of whether the underlying `/perception/capture_premove` service call actually succeeded — it only checks whether the Flask response is a tuple (the ROS-not-connected case), never the `ok` field in the JSON body. Reproduced live: the service returned `{"ok": false, "msg": "No image or board corners available"}` (board_detector_node hadn't found a board) yet the UI claimed success. Needs a one-line fix in `api_capture_reference` (`code/chess_os.py`) to check the parsed response's `ok` field before flipping the state.
- [ ] Step 2: remove duplicated coordinate math from chess_os.py
- [ ] Step 3: ROS-native hardware-test orchestration
- [ ] Step 4: convert to a real `chess_ui` ROS package, split into modules
- [ ] Step 5: delete superseded `code/` scripts

### 2. Physical Calibration & Verification

**Problem**: Several correctly-implemented features have never been proven against the real hardware.
- [ ] Measure and set `x_max_mm`, `board_origin_x/y_mm` on physical hardware
- [ ] Test corner routing on actual board with pieces — verify BFS fallback behavior
- [ ] Run camera intrinsic calibration once and commit/verify `calibration.npz` workflow
- [ ] Confirm Chess OS game-control services end-to-end on the Pi

### 3. Ongoing Documentation Hygiene

**Problem**: Docs drift out of sync with code within a couple months if not actively re-checked (this file was 2 months stale as of the 2026-07-08 audit).
- [ ] Re-run a doc-vs-code consistency check periodically (package names, changelog vs. git log, cross-doc contradictions)
- [ ] Keep this file's Master Status List current as work lands — update it in the same commit/session as the change, not after

---

## 🛠 Active Workflows

To start working on an area, adopt a **Persona**:

- **"I am the Hardware Agent"**: I will ignore game logic and focus purely on making motors spin and reading sensors.
- **"I am the Logic Agent"**: I will mock hardware and focus on chess rules and state machines.
- **"I am the Chess OS Agent"**: I will work through the Architecture Consolidation initiative above, one step at a time, verifying on the Pi after each step before moving to the next.

_Last Updated: 2026-07-08_
