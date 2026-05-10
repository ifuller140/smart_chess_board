# Project Context - Smart Chess Board

> **Complete context document for AI agents. Read CLAUDE.md first.**

## Project Vision

Build an automated chess board that can:
1. Detect when a human player makes a move (via camera)
2. Calculate the optimal response (via chess engine)
3. Physically move the piece (via CoreXY gantry with electromagnet)
4. Capture pieces and move them to a graveyard area
5. Manage game timing with physical chess clocks

## System Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                         RASPBERRY PI 4B                         │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐              │
│  │   Camera    │  │  Stockfish  │  │    GPIO     │              │
│  │   Module    │  │   Engine    │  │  Interface  │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│  ┌──────▼──────┐  ┌──────▼──────┐  ┌──────▼──────┐              │
│  │   chess_    │  │   chess_    │  │   chess_    │              │
│  │ perception  │  │   logic     │  │hw_interface │              │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘              │
│         │                │                │                      │
│         └────────────────┼────────────────┘                      │
│                          │                                       │
│                   ┌──────▼──────┐                                │
│                   │   gantry_   │                                │
│                   │   control   │                                │
│                   └─────────────┘                                │
└─────────────────────────────────────────────────────────────────┘
                           │
         ┌─────────────────┼─────────────────┐
         │                 │                 │
    ┌────▼────┐      ┌─────▼─────┐     ┌────▼────┐
    │ Stepper │      │   Servo   │     │ Limit   │
    │ Motors  │      │ + Magnet  │     │ Switches│
    └─────────┘      └───────────┘     └─────────┘
```

## Physical Layout

```
┌─────────────────────────────────────────────────┐
│                  GANTRY FRAME                    │
│  ┌───────────────────────────────────────────┐  │
│  │                                           │  │
│  │  [GRAVEYARD ZONE - behind black pieces]   │  │
│  │                                           │  │
│  │  ┌─────────────────────────────────────┐  │  │
│  │  │                                     │  │  │
│  │  │         8×8 CHESS BOARD             │  │  │
│  │  │    rank 8 (Black) ←────────         │  │  │
│  │  │        ┌───┐                        │  │  │
│  │  │        │MAG│ ← Permanent magnet     │  │  │
│  │  │        └───┘   on servo arm         │  │  │
│  │  │    rank 1 (White) ←────────         │  │  │
│  │  └─────────────────────────────────────┘  │  │
│  │                                           │  │
│  └───────────────────────────────────────────┘  │
│                                                  │
│  [Y home limit]                    [X home limit] │
│   (front, Y=0, player side)        (right, X_MAX) │
└─────────────────────────────────────────────────┘
     ↑ Camera mounted above (looking down)
     ↑ Chess clock nearby with clock-hit limit switch
```

**Player sits at the FRONT (bottom). Camera/electronics tower is at the BACK (top).**
**a-file is on the player's LEFT. h-file is on the player's RIGHT.**

### Coordinate System

- **Origin (0,0)**: Bottom-**left** corner — the logical home position after homing sequence completes
- **+X direction**: RIGHT (toward h-file). X increases moving right away from the a-file.
- **+Y direction**: BACKWARD / UP (toward Black's side, rank 8). Y increases moving away from the player.
- **Z-axis**: Servo arm raises and lowers the **permanent magnet** below the board

**Limit switches:**
- **X limit** (`/limit_switch/x_min`): at **X_MAX** (far right, h-file side). Homing drives in **+X** until this triggers. After contact, gantry backs off and creeps to confirm position, then drives back LEFT to X=0 (origin).
- **Y limit** (`/limit_switch/y_min`): at **Y=0** (front/bottom, player's side). Homing drives in **−Y** until this triggers.

> [!IMPORTANT]
> After homing, the gantry is physically at (X_MAX, 0) — right side, front. It then drives
> leftward to X=0 to establish the bottom-left origin. Square **a1 ≈ (20mm, 20mm)** from
> origin. Square **h1 ≈ (195mm, 20mm)**. Board spans X: 20→195mm, Y: 20→195mm.

## Game Flow State Machine

```mermaid
stateDiagram-v2
    [*] --> STARTUP
    STARTUP --> HOMING: System init
    HOMING --> IDLE: Homed successfully
    
    IDLE --> WAITING_PLAYER_MOVE: Game started
    
    WAITING_PLAYER_MOVE --> DETECTING_MOVE: Clock pressed
    DETECTING_MOVE --> VALIDATING_MOVE: Board captured
    VALIDATING_MOVE --> WAITING_PLAYER_MOVE: Invalid move
    VALIDATING_MOVE --> CALCULATING_RESPONSE: Valid move
    
    CALCULATING_RESPONSE --> EXECUTING_MOVE: Engine returned move
    EXECUTING_MOVE --> WAITING_PLAYER_MOVE: Move complete
    
    WAITING_PLAYER_MOVE --> GAME_OVER: Checkmate/Draw
    GAME_OVER --> IDLE: Reset
```

## CoreXY Kinematics

The gantry uses CoreXY belting for X/Y motion:

```
Motor A steps = X_steps + Y_steps
Motor B steps = Y_steps - X_steps
```

Key parameters:
- **Steps per revolution**: 200 (NEMA 11 full-step)
- **Pulley pitch length per revolution**: 40mm (20T GT2 pulley)
- **Steps per mm**: ~5 steps/mm (full-step, no microstepping)

<!-- USER_ATTENTION: Verify these values with actual hardware calibration -->

## Board Mapping

Each chess square maps to physical X/Y coordinates:

<!-- USER_ATTENTION: These are estimated values - calibrate with actual board -->

| Square | X (mm) | Y (mm) | Notes |
|--------|--------|--------|-------|
| a1 | ~20 | ~20 | Player's near-left corner; closest to origin |
| h1 | ~195 | ~20 | Player's near-right corner; near X limit |
| a8 | ~20 | ~195 | Far-left back corner (Black's queen-side rook) |
| h8 | ~195 | ~195 | Far-right back corner (Black's king-side rook) |
| Square size | 25mm | 25mm | **Must verify with physical measurement** |

> [!NOTE]
> These values are estimated. Run the hardware `gantry/square_navigation` test to verify.
> Formula: `square_x = board_origin_x + (col_index * 25)`, `square_y = board_origin_y + (rank_index * 25)`
> where col a=0, b=1...h=7 and rank 1=0, 2=1...8=7.
> The defaults (board_origin_x=20, board_origin_y=20) need calibration on the physical board.

**Graveyard positions** (behind black's pieces, rank 8 side):
- All captured pieces (white and black): behind rank 8, Y ≈ 215mm
- Routing: piece moves horizontally to board edge first, then up to graveyard Y

## FEN Encoding

Board state is communicated using FEN (Forsyth-Edwards Notation):

```
Starting position:
rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1
```

Piece encoding used internally:
| Code | Piece |
|------|-------|
| 0 | Empty |
| 1 | White Pawn |
| 2 | White Knight |
| 3 | White Bishop |
| 4 | White Rook |
| 5 | White Queen |
| 6 | White King |
| -1 | Black Pawn |
| -2 | Black Knight |
| ... | ... |

## Magnet System

The magnet system uses a **permanent magnet** (no electromagnet) attached to the end of the Z-axis servo arm. The servo raises and lowers the magnet through the underside of the chess board to attract and release chess pieces.

- **Servo engaged (down)**: Magnet is lowered to board surface — pieces are attracted
- **Servo released (up)**: Magnet is raised away from board — pieces are free
- **No GPIO power control needed** — the servo position alone controls pickup

### System Dependencies
```bash
sudo apt install python3-pip python3-opencv libcamera-apps
```

### Python Dependencies
```bash
pip3 install RPi.GPIO python-chess opencv-python numpy
```

### ROS 2 Dependencies
- `rclpy` - ROS 2 Python client library
- `std_msgs` - Standard message types
- `geometry_msgs` - Geometry message types
- `sensor_msgs` - Sensor message types (for images)

## Chess OS — Primary User Interface

`code/chess_os.py` is the master control web app. Run it in a second terminal after launching ROS nodes:

```bash
python3 code/chess_os.py        # → http://<pi-ip>:5000
python3 code/chess_os.py --no-ros  # offline / vision-only mode
```

| Tab | Purpose |
|-----|---------|
| **Game** | FEN board display, game start/stop, move history, chess clock |
| **Gantry** | Jog (WASD+servo), homing, board calibration workflow, square goto, canvas |
| **Hardware** | Servo/magnet, limit switch live status, stepper step, E-stop |
| **Perception** | Live MJPEG stream, board corners, CV params, overlay toggles |
| **Tests** | Run any `test_runner.py` test category/subtest from the browser |

Chess OS connects to ROS automatically if `rclpy` is importable and the ROS nodes are running. All state is available via `GET /api/status`.

**Board calibration** is done entirely from the Gantry tab — no CLI required. See the workflow card in that tab.

## File Locations Reference

| Purpose | Path |
|---------|------|
| **Chess OS (main UI)** | `code/chess_os.py` |
| Board calibration data | `board_calibration.json` (project root, created by Chess OS) |
| GPIO pin config | `src/chess_hw_interface/config/pins.yaml` |
| CV parameters | `src/chess_perception/config/cv_params.yaml` |
| Board coordinates | `src/gantry_control/config/board_map.yaml` |
| Camera calibration | `src/chess_perception/config/calibration.npz` |
| Launch all | `src/launch/full_system_launch.py` |
| Test scripts | `code/*.py` |
| CAD exports | `cad/exports/` |
| System config (sudoers) | `setup/` |
| Vision system docs | `docs/features/vision-system.md` |

## Testing Hierarchy

1. **Unit Tests**: Individual function testing (pytest)
2. **Component Tests**: Hardware validation scripts in `code/`
3. **Integration Tests**: ROS 2 launch file testing
4. **System Tests**: Full game loop validation

## Common Failure Modes

| Symptom | Likely Cause | Solution |
|---------|--------------|----------|
| Motors don't move | Wrong pin config | Check `pins.yaml` BCM numbers |
| Motors jitter | Step timing too aggressive or noisy power | Increase `min_step_delay_ms`, verify motor PSU and common ground |
| Camera black | Not enabled | Run `sudo raspi-config`, enable camera |
| Permission denied | GPIO access | Add user to `gpio` group |
| Magnet weak | Insufficient power | Check 5V supply current capacity |

---

*See [CLAUDE.md](../CLAUDE.md) for quick reference and task guidelines.*
