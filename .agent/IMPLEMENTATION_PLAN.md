# Audit Remediation Implementation Plan

> Cross-session tracker for fixing everything found in the 2026-07-08 full logic/system audit
> (see the "Bottom line" / per-package findings that were folded into `.agent/PROJECT_STATUS.md`
> and the strategic-initiatives section there). This file tracks **phase-by-phase execution
> status** so a new session can pick up exactly where the last one left off.
>
> Workflow per phase: implement → build → `git push` → SSH to the Pi (`ian@100.66.77.72`,
> workspace `~/dev/smart_chess_board`) → `git pull` → rebuild the affected package(s) →
> live-verify → update this file + `PROJECT_STATUS.md` → move to the next phase.

## Phase status

| Phase | Scope | Status |
|---|---|---|
| 1 | Coordinate/config fix (`board_map.yaml` + `x_max_mm` reconciliation) | ✅ done 2026-07-08 |
| 2 | `game_manager_node` permanent-death & correctness bugs | 🔄 in progress |
| 3 | Safety: blocking executors defeat e-stop; action double-terminal bug; test-runner interlock | ⬜ not started |
| 4 | First live-motor session + remaining `gantry_control` findings (castling snapshot, corner-BFS start, graveyard reset, physical calibration) | ⬜ not started |
| 5 | `chess_perception` fixes | ⬜ not started |
| 6 | `chess_ui` fixes (e-stop clear, live param push, SSE queue, jog timeout) | ⬜ not started |

## Phase details

### Phase 1 — Coordinate/config fix
**Files:** `src/gantry_control/config/board_map.yaml`, `gantry_kinematics_node.py`, `homing_node.py`

- `board_map.yaml` documented the old (pre-fix) coordinate convention while `motion_planner_node.py`'s code implements the new one — 5 of 8 files were unreachable (e2e4 rejected).
- `board_edge_safe_x_mm` (read by code) vs `board_edge_left_x_mm`/`board_edge_right_x_mm` (yaml) name mismatch — yaml value silently ignored.
- `x_max_mm` disagreed 3 ways: `homing_node` default 240, yaml/`gantry_kinematics_node` 250, `gantry_kinematics_node.py`'s own fallback 300.

**Fix:** rewrite yaml to match the code's own already-correct convention/defaults, rename the edge param, add `homing_node`'s `x_max_mm` to yaml, fix the stray 300 fallback.

**Verification:** `ros2 param get` on the Pi for all three nodes — no live motor movement (deferred to Phase 3/4 for safety).

**Status: ✅ DONE (2026-07-08).** Commit `0c4d8df` pushed to `ros-dev`, pulled and rebuilt on the Pi (`colcon build --symlink-install --packages-select gantry_control`, clean, 5.7s). Live-verified: launched `gantry_kinematics_node` and `motion_planner_node` standalone (no pigpio dependency) with `board_map.yaml`, confirmed via `ros2 param get` — `x_max_mm=250.0`, `y_max_mm=250.0`, `board_origin_x_mm=20.0`, `board_origin_y_mm=20.0`, `board_edge_safe_x_mm=230.0`, `graveyard_origin_x_mm=230.0`, `square_size_mm=25.0`. Hand-checked bounds: e2/e4 (col 4) → x=120mm, h1/h8 (col 7) → x=195mm, both now within `[0, 250]` (h-file was previously rejected at 375mm). Verification processes were killed afterward; the 5 pre-existing production nodes (`camera_node`, `board_detector_node`, `piece_detector_node`, `chess_os`, `test_runner_node`) were untouched throughout.
`homing_node`'s corrected `x_max_mm` param was **not** live-verified — `pigpiod` isn't running on the Pi and `sudo -n pigpiod` confirmed it needs an interactive password not available this session. Verified via static code/yaml inspection only; revisit when pigpiod is available (Phase 3/4).

### Phase 2 — `game_manager_node` fixes
**File:** `src/chess_logic/chess_logic/nodes/game_manager_node.py`

Homing-failure and every game-end path currently `break`/`return` out of the daemon game-loop thread permanently, killing all future `/game/*` service calls silently (services stay registered and lie about success). Plus: `/game/resign` races the loop thread, a failed computer move is pushed to the board model anyway (desync), stale perception/motion completions aren't correlated to the request that caused them, `PROMOTION_WAIT` has no timeout/flag-poll, and clock-hit servo failures are silently discarded.

**Verification:** ROS-graph-level only (service calls + synthetic topic publishes), no hardware required.

### Phase 3 — Safety fixes (gate before live motor testing)
**Files:** `homing_node.py`, `gantry_kinematics_node.py`, `stepper_driver_node.py`, `servo_node.py`, `clock_servo_node.py`, `test_runner_node.py`/`run_hw_test.sh`

Single-threaded executors + blocking move/home callbacks mean `/emergency_stop` (and, for `gantry_kinematics_node`, its own `/gantry/pose` feedback) can't be processed mid-motion. Fix via `MultiThreadedExecutor` + callback groups. Also fixes `gantry_kinematics_node`'s double-terminal-goal-state bug on cancel, hardware-test cancellation not reaching the root-owned subprocess, and no GPIO interlock between a live game and a running hardware test.

### Phase 4 — First live-motor session + remaining gantry findings
Homing/jog/square-nav/pick-and-place + e-stop-during-motion, castling stale-snapshot fix, corner-BFS-can't-start-from-blocked-source fix, graveyard slot-counter reset on new game, physical measurement of `x_max_mm`/`board_origin_x/y_mm`.

### Phase 5 — `chess_perception`
Fallback camera never publishes `CompressedImage`, dead `premove_avg_count` averaging, corner-label cross-frame identity drift, stale-corner republish with no staleness marker, unpopulated `BoardState.msg` fields + stale hardware-test service references.

### Phase 6 — `chess_ui`
E-stop-clear doesn't reset hardware latches, calibration/settings pushes have zero runtime effect until node restart, SSE test-feedback queue not cleared between runs, jog has no timeout, calibration push silently dropped if service not ready, minor TOCTOU/bare-except cleanup.

---
*Created 2026-07-08 from the full audit. Update the status table and add session notes under each phase as work lands.*
