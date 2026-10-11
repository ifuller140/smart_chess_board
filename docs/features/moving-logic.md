# Moving Logic

> **Collision-aware piece movement planning for the chess gantry.**

## Overview

Moving chess pieces on a physical board requires careful planning. Unlike digital chess where pieces teleport, the gantry must navigate around obstacles, handle captures correctly, and execute special moves like castling and en passant.

---

## Movement Types

### 1. Simple Move (No Collisions)

Direct path from source to destination when no pieces are in the way.

```
Source (e2) ─────────────────────► Destination (e4)

Sequence:
1. Move to e2, lower magnet, engage
2. Raise magnet
3. Move directly to e4
4. Lower magnet, release
5. Raise magnet
```

### 2. Capture Move

Must remove captured piece FIRST, then move the capturing piece.

```
Capture on e5 (white pawn d4 takes black pawn e5):

┌─────────────────────────────────────────────────────────────┐
│ PHASE 1: Remove captured piece                              │
├─────────────────────────────────────────────────────────────┤
│ 1. Move to e5 (captured piece location)                     │
│ 2. Lower, engage magnet                                     │
│ 3. Raise, move to graveyard slot                            │
│ 4. Lower, release                                           │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ PHASE 2: Move capturing piece                               │
├─────────────────────────────────────────────────────────────┤
│ 1. Move to d4 (source)                                      │
│ 2. Lower, engage magnet                                     │
│ 3. Raise, move to e5 (now empty)                            │
│ 4. Lower, release                                           │
└─────────────────────────────────────────────────────────────┘
```

### 3. Knight Move (May Jump Over Pieces)

Knights can jump over pieces in chess, but the gantry cannot. Two strategies:

#### Strategy A: Path Planning (Recommended)
Find a clear path around obstacles.

```
Knight on b1 to c3 with pieces on b2 and c2:

     a    b    c    d
   ┌────┬────┬────┬────┐
 3 │    │    │ ♘  │    │  ← Destination
   ├────┼────┼────┼────┤
 2 │    │ ♙  │ ♙  │    │  ← Obstacles
   ├────┼────┼────┼────┤
 1 │    │ ♘  │    │    │  ← Source
   └────┴────┴────┴────┘

Path options:
- Go around via a2: b1 → a1 → a2 → a3 → c3 ✓
- Go around via d1: b1 → d1 → d3 → c3 ✓
```

#### Strategy B: Temporary Piece Relocation
Move blocking pieces temporarily (complex, not recommended).

### 4. Castling

Two pieces move in sequence.

```
Kingside castling (O-O):

Before: ♖ on h1, ♔ on e1
After:  ♖ on f1, ♔ on g1

Sequence:
1. Move King e1 → g1
2. Move Rook h1 → f1

    e    f    g    h
  ┌────┬────┬────┬────┐
  │ ♔→ │    │ ♔  │ ♖  │
  └────┴────┴────┴────┘
        ←─────────♖
```

```
Queenside castling (O-O-O):

Before: ♖ on a1, ♔ on e1
After:  ♖ on d1, ♔ on c1

Sequence:
1. Move King e1 → c1
2. Move Rook a1 → d1
```

### 5. En Passant

Capturing pawn moves diagonally but removes a pawn from a different square.

```
En passant (white pawn d5 captures black pawn e5 that just moved e7→e5):

Before:
    d    e
  ┌────┬────┐
6 │    │    │
  ├────┼────┤
5 │ ♙  │ ♟  │  ← Both pawns on rank 5
  └────┴────┘

After:
    d    e
  ┌────┬────┐
6 │    │ ♙  │  ← White pawn on e6
  ├────┼────┤
5 │    │    │  ← Black pawn removed from e5
  └────┴────┘

Sequence:
1. Remove black pawn from e5 → graveyard
2. Move white pawn d5 → e6
```

### 6. Pawn Promotion

Pawn reaches 8th rank and transforms into another piece.

```
Promotion (pawn e7 → e8 becomes Queen):

Sequence:
1. Move pawn e7 → e8
2. (Physical board: replace pawn with queen manually OR
    use pre-positioned promotion pieces in storage area)
```

<!-- USER_ATTENTION: Decide how to handle physical piece swap for promotion -->

---

## Path Planning Algorithm

### Obstacle Detection

Before moving, check if any squares along the path contain pieces.

```python
def find_path(source, destination, board_state):
    """Find a collision-free path from source to destination."""
    
    # Get all pieces on board
    obstacles = get_occupied_squares(board_state)
    
    # Remove source (we're picking up this piece)
    obstacles.discard(source)
    
    # Try direct path first
    direct_path = get_direct_path(source, destination)
    if not any(sq in obstacles for sq in direct_path):
        return direct_path
    
    # Try L-shaped paths (around obstacles)
    for waypoint in generate_waypoints(source, destination):
        path1 = get_direct_path(source, waypoint)
        path2 = get_direct_path(waypoint, destination)
        combined = path1 + path2
        if not any(sq in obstacles for sq in combined):
            return combined
    
    # Fallback: edge routing (go to edge of board first)
    return route_via_edge(source, destination, obstacles)
```

### Safe Travel Height

When moving over empty squares, travel at safe height (Z raised).
Only lower when at source (to pick up) or destination (to place).

```
Side view of piece movement:

                    Safe travel height
                    ─────────────────────────
                          ┌─────────┐
                          │ MAGNET  │
                          └────┬────┘
                               │
     ┌───────┐            ┌────┴────┐            ┌───────┐
     │ PIECE │            │         │            │       │
     └───────┘            └─────────┘            └───────┘
     Source               (traveling)            Destination
```

---

## Graveyard Management

Captured pieces go to designated graveyard areas.

```
┌─────────────────────────────────────────────────────────────┐
│                        BOARD LAYOUT                          │
│                                                              │
│   WHITE GRAVEYARD          CHESS BOARD          BLACK GRAVEYARD
│   ┌─────────────┐    ┌─────────────────────┐   ┌─────────────┐
│   │ ○ ○ ○       │    │                     │   │       ● ● ● │
│   │ ○ ○ ○       │    │     8x8 squares     │   │       ● ● ● │
│   │ ○ ○ ○       │    │                     │   │       ● ● ● │
│   │ ○ ○ ○       │    │                     │   │       ● ● ● │
│   │ ○ ○ ○       │    │                     │   │       ● ● ● │
│   └─────────────┘    └─────────────────────┘   └─────────────┘
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

<!-- USER_ATTENTION: Define exact graveyard positions in board_map.yaml -->

### Graveyard Slot Tracking

```python
class GraveyardManager:
    def __init__(self):
        self.white_graveyard = [
            (230, 25), (255, 25), (280, 25),
            (230, 50), (255, 50), (280, 50),
            # ... up to 15 slots (max capturable pieces)
        ]
        self.black_graveyard = [
            (230, 125), (255, 125), (280, 125),
            (230, 150), (255, 150), (280, 150),
            # ... up to 15 slots
        ]
        self.next_white_slot = 0
        self.next_black_slot = 0
    
    def get_next_slot(self, color):
        if color == 'white':
            slot = self.white_graveyard[self.next_white_slot]
            self.next_white_slot += 1
            return slot
        else:
            slot = self.black_graveyard[self.next_black_slot]
            self.next_black_slot += 1
            return slot
```

---

## Move Sequence Generator

### High-Level Interface

```python
def generate_move_sequence(move_uci, board_state, graveyard):
    """Generate gantry command sequence for a chess move."""
    
    source = parse_square(move_uci[:2])  # e.g., "e2"
    dest = parse_square(move_uci[2:4])   # e.g., "e4"
    promotion = move_uci[4] if len(move_uci) > 4 else None
    
    commands = []
    
    # Check for capture
    if is_capture(dest, board_state):
        captured_piece = board_state[dest]
        graveyard_slot = graveyard.get_next_slot(piece_color(captured_piece))
        commands.extend(pick_and_place(dest, graveyard_slot))
    
    # Check for en passant
    if is_en_passant(source, dest, board_state):
        captured_square = get_en_passant_capture_square(source, dest)
        graveyard_slot = graveyard.get_next_slot('black')  # opponent color
        commands.extend(pick_and_place(captured_square, graveyard_slot))
    
    # Check for castling
    if is_castling(source, dest, board_state):
        rook_source, rook_dest = get_castling_rook_move(source, dest)
        commands.extend(pick_and_place(source, dest))  # King
        commands.extend(pick_and_place(rook_source, rook_dest))  # Rook
    else:
        # Standard move
        commands.extend(pick_and_place(source, dest))
    
    return commands
```

---

## Configuration Parameters

```yaml
motion_planner_node:
  ros__parameters:
    # Heights
    z_travel_height: 20.0       # Safe height for travel (mm)
    z_pickup_height: 2.0        # Height for magnet engagement
    z_place_height: 0.0         # Height for piece release
    
    # Speeds
    travel_speed: 15.0          # mm/s during travel
    approach_speed: 5.0         # mm/s near pieces
    
    # Timing
    magnet_engage_delay: 0.1    # seconds after lowering
    magnet_release_delay: 0.05  # seconds before raising
    
    # Path planning
    obstacle_clearance: 5.0     # mm extra clearance around pieces
    prefer_edge_routing: false  # Route via board edge if true
```

---

## Error Handling

| Error | Detection | Recovery |
|-------|-----------|----------|
| Piece not picked up | Expected weight not detected | Retry pickup, alert user |
| Obstacle collision | Limit switch triggers unexpectedly | Emergency stop, re-home |
| Wrong piece moved | Post-move vision mismatch | Alert user for manual fix |
| Path planning fails | No valid path found | Alert user, suggest manual move |

---

*See [corexy-gantry.md](corexy-gantry.md) for motion control details.*
