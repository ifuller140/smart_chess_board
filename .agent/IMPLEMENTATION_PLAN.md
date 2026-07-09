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
| 2 | `game_manager_node` permanent-death & correctness bugs | ✅ done 2026-07-08 |
| 3 | Safety: blocking executors defeat e-stop; action double-terminal bug; test-runner interlock | ✅ done 2026-07-08 |
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

**Status: ✅ DONE (2026-07-08).** Commit `0485e3a` pushed to `ros-dev`, pulled and rebuilt on the Pi (`colcon build --symlink-install --packages-select chess_logic`, clean). Verified live using a throwaway stub node (`/tmp/stub_hw_services.py`, deleted after use — not committed) providing fake `/gantry/home`, `/camera/capture`, `/perception/capture_premove`, `/clock/hit`, and `/chess_engine/request_move` services, driven via `ros2 topic pub`/`ros2 service call`:
- **Homing-retry fix**: with `/gantry/home` genuinely unavailable (stub killed), the node cycled `HOMING → HOMING_FAILED → HOMING` every ~15s for 35+ seconds (3 full cycles) without the process ever dying — this is the exact scenario the audit found killed the game loop permanently.
- **Full turn cycle**: drove a complete human move (e2e4, validated via synthetic `/perception/changed_squares`) → engine reply (stub returned a real legal move via python-chess) → motion execution (`/motion/done`) → clock hit → back to `WAITING_PLAYER_MOVE`, all correctly sequenced.
- **The exact audit-reproduced bug**: called `/game/resign` (correctly ended the game, published the result) then `/game/new_game` — confirmed the game genuinely reset to `IDLE` and responded to a fresh clock press, unlike the audit's original finding where `/game/new_game` returned `success=True` but did nothing.
- **`MOTION_ERROR` path**: simulated a failed move (`/motion/done` with `data:false`) — confirmed the move was *not* pushed to the board model, the node transitioned to `MOTION_ERROR` and paused (didn't push a stale/wrong position), and `/game/new_game` recovered it back to `IDLE`.
- **Awaiting-gate fix**: repeated/late `/perception/changed_squares` and `/motion/done` publishes outside their intended wait windows were correctly logged as "ignoring — not currently awaiting" and had no effect, confirming stale-event correlation works.
- One false alarm during testing: `game_manager_node`'s stdout is block-buffered when redirected to a file, making log-only observation initially look like clock-hit wasn't received — resolved by relaunching with `PYTHONUNBUFFERED=1`; not a code bug.
- All verification processes and the stub script were cleaned up afterward; the 5 pre-existing production nodes were confirmed untouched throughout.

### Phase 3 — Safety fixes (gate before live motor testing)
**Files:** `homing_node.py`, `gantry_kinematics_node.py`, `stepper_driver_node.py`, `servo_node.py`, `clock_servo_node.py`, `test_runner_node.py`/`test_runner.py`/`base_test.py`/`run_hw_test.sh`, new `chess_hw_interface/gpio_lock.py`

Single-threaded executors + blocking move/home callbacks mean `/emergency_stop` (and, for `gantry_kinematics_node`, its own `/gantry/pose` feedback) can't be processed mid-motion. Fix via `MultiThreadedExecutor` + callback groups. Also fixes `gantry_kinematics_node`'s double-terminal-goal-state bug on cancel, hardware-test cancellation not reaching the root-owned subprocess, and no GPIO interlock between a live game and a running hardware test.

**Status: ✅ DONE (2026-07-08).** Commits `15cac76` (main fix) and `9ff54a0` (follow-up: `cancel_check` constructor-forwarding fix found during live verification), pushed to `ros-dev`, pulled and rebuilt on the Pi (`chess_hw_interface`, `gantry_control`).

- **Blocking-executor fix**: `homing_node`, `stepper_driver_node`, `servo_node`, `clock_servo_node`, `gantry_kinematics_node` all switched from the default single-threaded `rclpy.spin()` to `MultiThreadedExecutor(num_threads=4)`, with `/emergency_stop` (and, for `gantry_kinematics_node`, `/gantry/pose` + limit-switch subscriptions) moved into their own callback group separate from the blocking service/action-execute callback. Verified live and empirically: launched `gantry_kinematics_node` standalone (no pigpio dependency) and drove a real `/gantry/move` action goal while an external script published synthetic `/gantry/pose` feedback concurrently — the goal correctly detected arrival and received 28 feedback messages *while* `execute_callback` was blocked in its polling loop, proving the pose subscription now runs concurrently instead of being starved. Under the old single-threaded code this would have hung forever (arrival can never be detected if pose can never update).
  - **Update (same day, follow-up session once `pigpiod` was started)**: `homing_node`, `stepper_driver_node`, `servo_node`, `clock_servo_node` all launched standalone against a real `pigpiod` and connected cleanly with no errors — closing the runtime-verification gap noted above. `stepper_driver_node` and `homing_node` were run **simultaneously**, confirming their shared-lock coexistence (see GPIO interlock below) works for real, not just in the unit test. No motors were commanded to move — this only confirms clean startup/pigpio-connection under the new executor/callback-group code; actual `/emergency_stop`-during-motion (real motor movement) is deferred to Phase 4's first live-motor session.
- **`gantry_kinematics_node` double-terminal-goal-state fix**: `_execute_trapezoidal_move` no longer calls `goal_handle.canceled()` itself — it returns a `'completed'`/`'canceled'` status string, and `execute_callback` is now the sole caller of `succeed()`/`abort()`/`canceled()`. Verified live: sent a slow move goal, canceled it mid-flight, confirmed a single clean `CANCELED` terminal state (status 5) with no exception in the node's log (previously this path called two terminal-state methods on the same goal handle, the second of which raises uncaught).
- **Hardware-test cancellation**: root cause was that `test_runner_node` (running as `ian`) calling `os.killpg()`/SIGTERM against the test subprocess is silently rejected by the kernel once that subprocess re-execs as root via `sudo` (see `setup/smart-chess-hw-tests.sudoers`) — an unprivileged process can never signal a more-privileged one, regardless of process group. Added a killswitch-file mechanism instead: `test_runner_node` creates a unique cancel file and passes its path through `run_hw_test.sh`'s existing `sudo VAR=val` env-forwarding (`SETENV` is already in the sudoers rule); `test_runner.py` and `base_test.py`'s `HardwareTest.run()` poll for it between test steps/subtests and exit cleanly. Verified live: a synthetic `HardwareTest` subclass run through the real installed `base_test.py` correctly ran all steps when no cancellation was requested, and stopped after exactly one step when the cancel file appeared mid-run. Also verified the env-var → `cancel_check` wiring in `test_runner.py` directly. **Not verified**: the actual sudo hop itself, end-to-end through `run_hw_test.sh` — this environment has no passwordless sudo and no `CHESS_SUDO_PASS` set (same limitation noted in Phase 1), so the real subprocess can't be exercised. The forwarding syntax is identical to the already-proven-working `PYTHONPATH`/`LD_LIBRARY_PATH` forwarding in the same script, so this is a low-risk gap, but it should be spot-checked once sudo access is available.
- **Found and fixed during live verification** (not part of the original plan): seven `HardwareTest` subclasses (`GantryTestBase`, `VisionDetailBase`, and 5 standalone `__init__` overrides across `test_board_calibration.py`, `test_magnet.py`, `test_clock_integration.py`, `test_square_navigation.py`, `test_manual_gantry.py`) had `__init__(self, gpio_interface=None, display_interface=None)` signatures that didn't forward the new `cancel_check` kwarg to `super().__init__()`, so instantiating almost any real registered test (`GantryFullTest`, `BoardCalibrationTest`, `CameraCornerDetectionTest`, etc.) via `run_test()` raised `TypeError`. Caught by actually running `--all --mock` rather than trusting a read-through; fixed in commit `9ff54a0`.
- **GPIO pin interlock**: new `chess_hw_interface/gpio_lock.py`, a non-blocking `flock()`-based mutex (`/tmp/.chess_gantry_pins.lock`). `stepper_driver_node`/`homing_node` each acquire a **shared** lock for their lifetime (they already coordinate via ROS and are meant to coexist); `test_raw_motor`/`test_timing_sweep` (the only two hardware tests that open their own independent pigpio connection to the identical BCM pins) must acquire an **exclusive** lock before touching the motors, refusing to start with a clear error if a production node is live. Verified directly: a standalone two-process-style script confirmed shared+shared succeeds, shared+exclusive fails, and exclusive+exclusive fails, exactly matching the intended semantics — this logic is pure `fcntl`/`os` with no pigpio/ROS dependency so it could be exercised precisely.
  - **Update (real end-to-end confirmation, `pigpiod` running)**: with `stepper_driver_node` and `homing_node` both live against real pigpio, ran `python3 -m chess_hw_interface.testing.test_runner --category gantry --subtest raw_motor` — it correctly refused to start with `Setup failed: gantry GPIO pins are in use by a running production node (stepper_driver_node/homing_node). Stop the live ROS stack before running this test.`, failing before `PigpioStepper()` was ever constructed and before any pin was touched. This is the interlock's actual intended real-world scenario, fully confirmed.
- **Noted but out of scope for this phase**: running `--all --mock` (independent of any cancellation) never completed within 30s in this environment — some non-mocked test path (likely a `gantry`/`vision` test that spins up a real ROS node regardless of `--mock`) appears to hang without live hardware/production nodes present. Pre-existing, unrelated to this phase's changes (mock mode only swaps `gpio_interface`/`display_interface`, not the real `rclpy.Node` subclasses several tests construct internally). Also independently reproduced a `terminate called without an active exception` C++-level abort/core-dump at `test_runner.py`'s process exit in both the mock run and the real-pigpiod `raw_motor` rejection test above — happens *after* the test's own result is already printed, during `rclpy.shutdown()`/`monitor_node.destroy_node()` cleanup, most likely a race between the daemon `monitor_thread` (still spinning `RosMonitorNode`) and shutdown. Pre-existing, not a regression from this phase's changes, doesn't affect the CLI's printed result — but worth a proper fix during Phase 4 or 5 since it means every CLI test invocation currently core-dumps on exit.

### Phase 4 — First live-motor session + remaining gantry findings
Homing/jog/square-nav/pick-and-place + e-stop-during-motion, castling stale-snapshot fix, corner-BFS-can't-start-from-blocked-source fix, graveyard slot-counter reset on new game, physical measurement of `x_max_mm`/`board_origin_x/y_mm`.

### Phase 5 — `chess_perception`
Fallback camera never publishes `CompressedImage`, dead `premove_avg_count` averaging, corner-label cross-frame identity drift, stale-corner republish with no staleness marker, unpopulated `BoardState.msg` fields + stale hardware-test service references.

### Phase 6 — `chess_ui`
E-stop-clear doesn't reset hardware latches, calibration/settings pushes have zero runtime effect until node restart, SSE test-feedback queue not cleared between runs, jog has no timeout, calibration push silently dropped if service not ready, minor TOCTOU/bare-except cleanup.

---
*Created 2026-07-08 from the full audit. Update the status table and add session notes under each phase as work lands.*
