# Smart Chess Board — Project Integration Plan

## Implementation Status

| Phase | Status | Notes |
|-------|--------|-------|
| Phase 1 — Game Control Backend | ✅ **COMPLETE** | `game_manager_node.py` + `chess_clock_node.py` — 2026-05-10 |
| Phase 2 — Chess OS API + ROS Wiring | ✅ **COMPLETE** | All routes, `_RosNode` additions, `api_status` update — 2026-05-10 |
| Phase 3 — Chess OS Game Tab UI | ✅ **COMPLETE** | Pre-game checklist, game controls, settings, move history update, promotion banner, game-over banner — 2026-05-10 |
| Phase 4 — Perception Tab + Node Health | ✅ **COMPLETE** | Capture Reference banner, diff heatmap, score canvas, threshold sliders, Apply to Detector, Node Health grid — 2026-05-10 |
| Phase 5 — Polish | ✅ **COMPLETE** | IP print at startup, docstring fix, E-stop already publishes zero vel, tests Clear button was already present — 2026-05-10 |

### Perception Pipeline Note (2026-05-10)

The plan originally described a basic `capture_reference` button. The full fen_visualizer perception pipeline has been integrated into chess_os.py instead:

- `/perception/square_scores` (String JSON) subscription added to `_RosNode`
- `/perception/piece_debug` (Image) heatmap subscription added
- `/perception/reference_status` (String) subscription added
- `/api/diff_frame`, `/api/square_scores`, `/api/detector_params` routes added
- `SetParameters` client added for `piece_detector_node` (via `rcl_interfaces`)
- Perception tab completely replaced with fen_visualizer-style Diff Analysis section:
  - Live heatmap from ROS (`/api/diff_frame`)
  - Interactive per-square score canvas (JS-rendered)
  - Diff threshold + shift compensation sliders with preview
  - Clump mode toggle + keep-per-clump slider
  - "Apply to Actual Detector" button (pushes to `piece_detector_node` via ROS params)

### What Still Needs Testing on Pi

1. `/game/start`, `/game/new_game`, `/game/resign` services — verify with `ros2 service call`
2. `/game_manager/move_history` and `/game_manager/result` topics — verify after test moves
3. `/clock/set_time` Float32 subscription — verify clock updates from Game Settings card
4. `/api/nodes/health` — verify node name matching (names must match exactly what `get_node_names()` returns)
5. `piece_detector_node` SetParameters — verify `rcl_interfaces` available on Pi
6. Pre-game checklist Start Game button gate — test all four prerequisite states
7. Promotion banner — trigger PROMOTION_WAIT state to verify banner visibility
8. Game-over banner — play to checkmate / call `/game/resign` to verify

---

## Goal

A complete playable game means: a user opens Chess OS at `http://<pi-ip>:5000`, homes the gantry, calibrates the board, captures a pre-move reference image, presses Start Game, and then plays a full physical game against Stockfish — making moves on the board, pressing the clock, watching the computer's arm move pieces in response, and seeing the result displayed when the game ends. Every step that requires user action is guided by the web UI, and no step requires a terminal. As of today, the backend state machine and all hardware layers are functional; what is missing is the glue between the ROS game loop and Chess OS, the ability to start/reset/resign from the browser, and the visibility tools (node health, move history, promotion prompt) that make the game operable without SSH.

---

## Pre-Game Checklist

This is the exact sequence a user must follow before the first move of every game session. The UI (Phase 3) will surface this as a checklist panel in the Game tab.

1. **Power on** — plug in Pi and motor PSU. Wait ~30s for ROS nodes to start.
2. **Open Chess OS** — `http://<pi-ip>:5000` in a browser. Confirm the ROS pill in the top bar shows green.
3. **Check node health** — Hardware tab → Node Health panel. All 12 nodes should show green. If any are red, relaunch the corresponding ROS package.
4. **Home the gantry** — Gantry tab → click "Home Gantry." Wait for `gantry_homed = true` indicator. The gantry must be homed before starting a game (Start Game button is disabled until this flag is set).
5. **Verify calibration** — Gantry tab → Calibration card. If `calib_applied = false`, manually jog to a1 and h8, save each corner, and click Apply. Calibration persists in `board_calibration.json` across reboots so this step is skipped if already saved.
6. **Set up the board** — place all pieces in the standard starting position on the physical board.
7. **Capture reference** — Perception tab → click "Capture Reference." This calls `/perception/capture_premove` and stores the starting position as the frame-diff baseline. **This step is required before every game**; the button is prominently styled and the Start Game button is disabled until it is pressed in the current session (tracked by `_session_reference_captured` flag in `_state`).
8. **Configure time controls (optional)** — Game tab → Settings card. Set think time (Stockfish, 1–10 s) and time per player (minutes). These call `/api/game/settings`.
9. **Start the game** — Game tab → Pre-game checklist panel → click "Start Game." This calls `/api/game/start` which triggers the `/game/start` ROS service. The game manager transitions from IDLE to WAITING_PLAYER_MOVE. Alternatively, the human can press the physical clock button.
10. **Play** — make moves on the board, press the physical clock after each move. The computer responds; repeat until game over.

---

## Phase 1: Game Control Backend

**Files:** `src/chess_logic/chess_logic/nodes/game_manager_node.py`, plus a new `chess_clock_node.py` if it does not yet exist as a full node (check before creating).

### 1.1 `game_manager_node.py` — New Services and Publishers

#### New threading event

Add `_new_game_event = threading.Event()` alongside the existing events in `__init__`.

#### New service servers

Add three `create_service` calls in `__init__`:

| Service name | Type | Method to add | What it does |
|---|---|---|---|
| `/game/start` | `Trigger` | `_svc_start_game` | Sets `_clock_hit_event` when state is `IDLE` or `PROMOTION_WAIT`. Allows Chess OS to act as a software clock press. |
| `/game/new_game` | `Trigger` | `_svc_new_game` | Resets `_board` to `chess.Board()`, clears `_move_history`, sets `_new_game_event`, then sets `_clock_hit_event`, `_board_state_event`, `_motion_done_event` to unblock any waiting loops. Returns success immediately; the game loop detects the event at the top of the while loop and handles the reset. |
| `/game/resign` | `Trigger` | `_svc_resign` | Calls `_end_game("Resignation")` from the service callback (safe because `_end_game` only sets state and publishes; it does not block). |

#### `_svc_start_game` implementation

```python
def _svc_start_game(self, request, response):
    if self._state in (GS.IDLE, GS.PROMOTION_WAIT):
        self._clock_hit_event.set()
        response.success = True
        response.message = f"Game start triggered from state {self._state}"
    else:
        response.success = False
        response.message = f"Cannot start game from state {self._state}"
    return response
```

#### `_svc_new_game` implementation

```python
def _svc_new_game(self, request, response):
    self._new_game_event.set()
    # Unblock any currently-waiting event.wait() in the game loop
    self._clock_hit_event.set()
    self._board_state_event.set()
    self._motion_done_event.set()
    response.success = True
    response.message = "New game reset triggered"
    return response
```

#### `_game_loop` — new game event check

At the very top of the `while rclpy.ok():` loop body (before the flag-fall check), add:

```python
if self._new_game_event.is_set():
    self._new_game_event.clear()
    self._clock_hit_event.clear()
    self._board_state_event.clear()
    self._motion_done_event.clear()
    self._board = chess.Board()
    self._pre_move_fen = chess.STARTING_FEN
    self._move_history.clear()
    self._publish_board_fen()
    self._pub_move_history()
    self._transition(GS.IDLE)
    self._do_capture_premove()
    self.get_logger().info("New game reset complete — waiting for clock press")
    continue
```

#### New publishers

Add in `__init__`:

```python
self._move_history_pub = self.create_publisher(String, '/game_manager/move_history', 10)
self._result_pub       = self.create_publisher(String, '/game_manager/result', 10)
```

Add `self._move_history: list[str] = []` to the instance variables.

#### `_pub_move_history` helper

```python
def _pub_move_history(self):
    msg = String()
    msg.data = ' '.join(self._move_history)
    self._move_history_pub.publish(msg)
```

Call `self._move_history.append(human_move.uci()); self._pub_move_history()` after `self._board.push(human_move)`, and similarly after `self._board.push(computer_move)`.

#### `_end_game` — publish result

In `_end_game` (or wherever it is currently implemented — if it is inline, extract it), add:

```python
msg = String()
msg.data = reason  # e.g. "Checkmate — White wins!"
self._result_pub.publish(msg)
```

#### Why each item is needed

- `/game/start` — enables the UI Start Game button to substitute for or supplement the physical clock. Without it, the only way to start a game is to walk to the board and press the clock.
- `/game/new_game` — enables resetting mid-game or after game over without restarting the ROS node. Critical for consecutive games.
- `/game/resign` — gives the human an escape hatch via the UI.
- `move_history` publisher — Chess OS needs UCI history to display the move list and for debugging.
- `result` publisher — Chess OS needs the final result string to display the game-over banner.

---

### 1.2 `chess_clock_node.py` — Set Time from Topic

Locate the clock node (expected at `src/chess_logic/chess_logic/nodes/chess_clock_node.py` or similar path; verify before editing). Add:

```python
self.create_subscription(Float32, '/clock/set_time', self._on_set_time, 10)
```

```python
def _on_set_time(self, msg: Float32):
    new_time = float(msg.data) * 60.0  # message is in minutes, internal is seconds
    self._time_per_player_s = new_time
    self._white_time = new_time
    self._black_time = new_time
    self._running = False  # stop both clocks
    self.get_logger().info(f'Clock time set to {new_time:.0f}s per player')
```

This allows Chess OS to configure time controls before a game starts without restarting the node.

---

## Phase 2: Chess OS API + ROS Wiring

**File:** `code/chess_os.py`

### 2.1 New `_state` fields

Add to the `_state` dict (around line 106):

```python
"move_history_uci":         [],   # list of UCI strings from /game_manager/move_history
"game_result":              "",   # final result string from /game_manager/result
"session_reference_captured": False,  # reset on each Chess OS startup
```

### 2.2 `_RosNode` additions

Inside the `_RosNode.__init__` block, add:

**New subscriptions:**

```python
self.create_subscription(String, "/game_manager/move_history", self._on_move_history, 10)
self.create_subscription(String, "/game_manager/result",       self._on_result,       10)
```

**New service clients:**

```python
self._svc_game_start    = self.create_client(Trigger, "/game/start")
self._svc_game_new      = self.create_client(Trigger, "/game/new_game")
self._svc_game_resign   = self.create_client(Trigger, "/game/resign")
self._svc_cap_reference = self.create_client(Trigger, "/perception/capture_premove")
```

Note: `_svc_cap_reference` is a named client so the Phase 4 route can use `call_svc` instead of creating an ad-hoc client the way `api_capture_premove` currently does. The existing `api_capture_premove` route can be refactored to use it.

**New publisher:**

```python
self.set_time_pub = self.create_publisher(Float32, "/clock/set_time", 10)
```

**New subscription callbacks:**

```python
def _on_move_history(self, msg):
    uci_str = msg.data.strip()
    with _lock:
        _state["move_history_uci"] = uci_str.split() if uci_str else []

def _on_result(self, msg):
    with _lock:
        _state["game_result"] = msg.data.strip()
```

### 2.3 New Flask routes

Add after the existing `/api/capture_premove` route:

#### `/api/game/start` (POST)

```python
@app.route("/api/game/start", methods=["POST"])
def api_game_start():
    return _call_svc("_svc_game_start")
```

#### `/api/game/new` (POST)

```python
@app.route("/api/game/new", methods=["POST"])
def api_game_new():
    with _lock:
        _state["game_result"] = ""
        _state["move_history_uci"] = []
    return _call_svc("_svc_game_new")
```

#### `/api/game/resign` (POST)

```python
@app.route("/api/game/resign", methods=["POST"])
def api_game_resign():
    return _call_svc("_svc_game_resign")
```

#### `/api/game/settings` (POST)

Accepts `{"think_time_s": float, "time_per_player_min": float}`.

```python
@app.route("/api/game/settings", methods=["POST"])
def api_game_settings():
    data = request.get_json(silent=True) or {}
    msgs = []
    if "think_time_s" in data:
        # Stored locally; game_manager reads engine_think_time_s parameter on init.
        # For runtime changes, game_manager would need a set_parameters call — store
        # in _state for display only; full dynamic param change is a future item.
        with _lock:
            _state["think_time_s"] = float(data["think_time_s"])
        msgs.append(f"think_time={data['think_time_s']}s")
    if "time_per_player_min" in data and _ros_node is not None and HAS_ROS:
        try:
            m = Float32()
            m.data = float(data["time_per_player_min"])
            _ros_node.set_time_pub.publish(m)
            msgs.append(f"clock={data['time_per_player_min']}min")
        except Exception as e:
            return jsonify({"ok": False, "msg": str(e)}), 500
    return jsonify({"ok": True, "applied": msgs})
```

Also add `"think_time_s": 2.0` to `_state` for the UI to read back.

#### `/api/perception/capture_reference` (POST)

```python
@app.route("/api/perception/capture_reference", methods=["POST"])
def api_capture_reference():
    resp = _call_svc("_svc_cap_reference")
    if not isinstance(resp, tuple):
        # Success — mark session flag
        with _lock:
            _state["session_reference_captured"] = True
    return resp
```

#### `/api/nodes/health` (GET)

```python
EXPECTED_NODES = [
    "stepper_driver_node", "servo_node", "limit_switch_node",
    "clock_display_node", "camera_node", "board_detector_node",
    "piece_detector_node", "gantry_kinematics_node", "motion_planner_node",
    "homing_node", "game_manager_node", "chess_engine_node",
]

@app.route("/api/nodes/health")
def api_nodes_health():
    if _ros_node is None or not HAS_ROS:
        return jsonify({n: False for n in EXPECTED_NODES})
    try:
        alive = set(_ros_node.get_node_names())
        return jsonify({n: (n in alive) for n in EXPECTED_NODES})
    except Exception as e:
        return jsonify({"error": str(e)}), 500
```

### 2.4 Update `api_status` response

In the `api_status` route (around line 743), add these keys to the returned dict:

```python
"move_history_uci":  _state["move_history_uci"],
"game_result":       _state["game_result"],
"think_time_s":      _state.get("think_time_s", 2.0),
"session_reference_captured": _state["session_reference_captured"],
```

---

## Phase 3: Chess OS Game Tab

**File:** `code/chess_os.py` — HTML template (the `HTML = r"""..."""` block)

All additions below go inside the existing Game tab `<div id="tab-game">` panel. They are described by element, data source, and action.

### 3.1 Pre-game checklist panel

**Location:** Top of game tab, above the board display.

**Renders:** A card titled "Pre-Game Checklist" with four status rows and a Start Game button.

| Row | Condition checked | Source |
|---|---|---|
| Homed | `status.gantry_homed === true` | `api_status` |
| Calibrated | `status.calib_applied === true` | `api_status` |
| Reference captured | `status.session_reference_captured === true` | `api_status` |
| Nodes ready | `status.ros_connected === true` | `api_status` |

Start Game button:
- Disabled unless `gantry_homed && session_reference_captured`
- On click: `POST /api/game/start`
- After response: show brief inline feedback ("Game started" or error message)

### 3.2 Game control row

**Location:** Below the board, visible when `game_state` is not `OFFLINE` or `STARTUP`.

Contains:
- **New Game** button — `POST /api/game/new`. On click, confirm with `window.confirm("Start a new game? Current game will be abandoned.")` before sending.
- **Resign** button — `POST /api/game/resign`. Confirm with `window.confirm("Resign this game?")`.

### 3.3 Game settings card

**Location:** Below game control row, always visible in Game tab.

Elements:

| Control | Type | Default | Action |
|---|---|---|---|
| Stockfish think time | Range slider, 1–10, step 0.5 | 2 | `POST /api/game/settings` with `{think_time_s}` on `input` event (debounced 500ms) |
| Time per player | Number input, 1–60 min | 10 | `POST /api/game/settings` with `{time_per_player_min}` on `change` event |

Display the current slider value next to the slider as it changes. On page load, populate from `status.think_time_s` (for the slider) and `status.white_time / 60` (for the clock input).

### 3.4 Move history panel

**Location:** Right side of game tab (alongside the board or below settings on narrow screens).

**Structure:** `<ol id="move-history-list">` with one `<li>` per UCI move, styled in monospace. Auto-scrolls to the bottom when a new move is appended.

**Polling:** The existing `setInterval` status poll (every 1 s) compares `status.move_history_uci` to the rendered list. If different, re-render the full list and scroll to bottom.

**Format:** Show moves in pairs as conventional chess notation or just raw UCI if conversion is not available. Example: `<li>1. e2e4 e7e5</li>`. Pairing logic: iterate `move_history_uci`, group by two.

### 3.5 Promotion notification banner

**Condition:** `status.game_state === 'PROMOTION_WAIT'`

**Renders:** A full-width highlighted banner inside the game tab:

> "The computer promoted a pawn! Place a queen on the indicated square, then click OK or press the physical clock."

With an **OK** button that calls `POST /api/game/start` (which triggers `_svc_start_game`, which sets `_clock_hit_event` — the game loop is blocked in PROMOTION_WAIT waiting for exactly this event).

Hide the banner whenever `game_state !== 'PROMOTION_WAIT'`.

### 3.6 Game-over banner

**Condition:** `status.game_state === 'GAME_OVER'` and `status.game_result !== ''`

**Renders:** A prominent banner showing `status.game_result` with a **New Game** button (same action as the game control row button).

---

## Phase 4: Chess OS Perception Tab and Node Health

### 4.1 Capture Reference button

**File:** `code/chess_os.py` — HTML template, Perception tab.

**Location:** Top of Perception tab, before the camera streams.

**Element:** A large, clearly labeled button: "Capture Pre-Move Reference."

**Styling:** Give it a warning-yellow border when `session_reference_captured === false`, a green border when true. Add a note: "Required before starting a game — captures the current board as the move-detection baseline."

**On click:** `POST /api/perception/capture_reference`. On success, show inline feedback "Reference captured" and update styling to green. On error, show the error message in red.

**Note:** This is distinct from the existing "Capture Premove" functionality in `api_capture_premove`. Both call the same underlying ROS service. The `api_capture_premove` route can be kept for backward compatibility or refactored to call `_call_svc("_svc_cap_reference")` — either way the new `/api/perception/capture_reference` route is the one exposed in the Perception tab UI.

### 4.2 Node health panel

**File:** `code/chess_os.py` — HTML template, Hardware tab.

**Location:** Top of Hardware tab.

**Polling:** `setInterval(() => fetch('/api/nodes/health').then(r=>r.json()).then(renderHealth), 3000)`

**Renders:** A two-column grid of node status rows. Each row: `<span class="dot dot-ok|dot-err"></span> node_name`. CSS: `.dot-ok { background: var(--ok); }`, `.dot-err { background: var(--err); }`.

Expected nodes list (matches `EXPECTED_NODES` in the Flask route):

```
stepper_driver_node    servo_node
limit_switch_node      clock_display_node
camera_node            board_detector_node
piece_detector_node    gantry_kinematics_node
motion_planner_node    homing_node
game_manager_node      chess_engine_node
```

---

## Phase 5: Polish

All changes are in `code/chess_os.py` unless noted.

### 5.1 Print Pi IP at startup

In `main()`, after the argument parsing block but before `app.run(...)`:

```python
import socket
def _get_local_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"

pi_ip = _get_local_ip()
print(f"\n  ChessOS ready — http://{pi_ip}:{args.port}\n")
```

### 5.2 Fix hardcoded IP in module docstring

Change line 13:

```
Access: http://192.168.1.149:5000
```
to:
```
Access: http://<pi-ip>:5000  (IP printed at startup)
```

### 5.3 `/api/hw/estop/clear` — publish zero velocity

In `api_estop_clear`, after clearing `estop_active`, add:

```python
if _ros_node is not None and HAS_ROS:
    try:
        _ros_node.vel_pub.publish(Twist())
    except Exception:
        pass
```

This prevents any residual jog velocity from persisting after an e-stop clear.

### 5.4 Gantry tab — jog keyboard legend

In the Gantry tab HTML, add a small info card with a table:

| Key | Action |
|---|---|
| W / S | Jog +Y / −Y |
| A / D | Jog −X / +X |
| Q / E | Servo down / up |
| Shift | 3× speed |
| Speed slider | % of MAX_VEL (600 mm/s) |

This card should be collapsed by default with a small "Keyboard shortcuts" toggle.

### 5.5 Tests tab — Clear button

In the Tests tab HTML, add a **Clear** button above the terminal `<pre>` output div. On click:

```javascript
document.getElementById('test-output').textContent = '';
```

### 5.6 `think_time_s` in `_state`

Add `"think_time_s": 2.0` to the `_state` dict. This is read by `api_status` (already added in Phase 2.4) so the settings slider can be populated correctly on page load.

---

## Testing Sequence

Work through these steps in order. Each step depends on the previous ones passing.

### Step 1: Unit-verify game_manager services (no hardware)

```bash
ros2 run chess_logic game_manager_node &
ros2 service call /game/start std_srvs/srv/Trigger
ros2 service call /game/new_game std_srvs/srv/Trigger
ros2 service call /game/resign std_srvs/srv/Trigger
```

Expected: each call returns `success=True` and the published `/game_manager/state` topic changes accordingly. Verify with `ros2 topic echo /game_manager/state`.

### Step 2: Verify move history and result publishing

Start a game, manually simulate a few moved-squares messages on `/perception/changed_squares` (or drive the game loop with clock-hit service calls). Confirm:

```bash
ros2 topic echo /game_manager/move_history   # should show growing UCI list
ros2 topic echo /game_manager/result         # should show result on GAME_OVER
```

### Step 3: Verify Chess OS API routes (--no-ros mode or full ROS)

Start Chess OS:

```bash
python3 code/chess_os.py --no-ros
```

Test each new route with curl:

```bash
curl -s -X POST http://localhost:5000/api/game/start | python3 -m json.tool
curl -s -X POST http://localhost:5000/api/game/new | python3 -m json.tool
curl -s -X POST http://localhost:5000/api/game/resign | python3 -m json.tool
curl -s -X POST http://localhost:5000/api/game/settings \
     -H 'Content-Type: application/json' \
     -d '{"think_time_s":3,"time_per_player_min":5}' | python3 -m json.tool
curl -s http://localhost:5000/api/nodes/health | python3 -m json.tool
curl -s http://localhost:5000/api/status | python3 -m json.tool
```

In `--no-ros` mode, game/start and game/new return `503 ROS not connected` — this is expected and confirms routes exist.

### Step 4: Verify Chess OS with full ROS stack

Start all nodes via the full launch file, then start Chess OS normally. Open the browser:

- Hardware tab → Node Health panel should show green dots for all running nodes.
- Game tab → Pre-game checklist should show correct state for homed/calibrated/reference flags.
- Click "Capture Reference" in Perception tab → reference captured banner goes green.
- Click "Start Game" → `game_state` changes from `IDLE` to `WAITING_PLAYER_MOVE`.
- Move a piece on the board and press the physical clock → game processes the move, move history `<ol>` gains a new entry, `game_state` cycles through states and returns to `WAITING_PLAYER_MOVE`.

### Step 5: Full game loop test

Play a complete short game (Scholar's Mate or force game-over quickly by setting think time to 0.5s and time per player to 1 minute). Verify:

- Move history panel fills in correctly.
- Computer physically executes moves via gantry arm.
- If a pawn promotes: promotion banner appears, clicking OK lets game continue.
- On checkmate/stalemate/flag fall: game-over banner appears with correct result string.
- "New Game" button resets the board and move history, and `game_state` returns to `IDLE`.

### Step 6: Regression checks

- E-stop during a move: publish `True` on `/emergency_stop`. Gantry stops. Clear e-stop via UI. Confirm zero-velocity published. Verify jog still works after clear.
- `/api/nodes/health` with one node killed: confirm that node goes red within 3s.
- Node health panel after relaunch: goes green within 3s.
- New Game during computer's move (state = `EXECUTING_MOVE`): `_new_game_event` is checked at top of loop; motion_done_event is set to unblock — game should reset cleanly. Verify gantry does not continue executing the aborted move (may need motion_planner abort support — log a warning if that is not yet implemented).

---

*Last updated: 2026-05-10*
