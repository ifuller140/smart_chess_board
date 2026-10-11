# Game Logic System

> **Chess rules, engine integration, and game state management.**

## Overview

The game logic layer manages the chess game rules, coordinates with the Stockfish chess engine, and orchestrates the overall game flow.

## Game Manager State Machine

```
                         STARTUP
                            │
                            ▼
                         HOMING
                            │
                            ▼
┌──────────────────────────────────────────────────────────────┐
│                          IDLE                                 │
│              (Ready to start a new game)                      │
└─────────────────────────────┬────────────────────────────────┘
                              │ User starts game
                              ▼
┌──────────────────────────────────────────────────────────────┐
│                   WAITING_FOR_PLAYER_MOVE                     │◄──┐
│           (Human's turn - monitoring clock button)            │   │
└─────────────────────────────┬────────────────────────────────┘   │
                              │ Clock button pressed                │
                              ▼                                     │
┌──────────────────────────────────────────────────────────────┐   │
│                      CAPTURING_BOARD                          │   │
│               (Taking and analyzing image)                    │   │
└─────────────────────────────┬────────────────────────────────┘   │
                              │                                     │
                              ▼                                     │
┌──────────────────────────────────────────────────────────────┐   │
│                      VALIDATING_MOVE                          │   │
│             (Checking if move is legal)                       │   │
└───────────┬─────────────────┴────────────────┬───────────────┘   │
            │ Invalid                          │ Valid              │
            │                                  ▼                    │
            │        ┌─────────────────────────────────────────┐   │
            │        │           CALCULATING_RESPONSE           │   │
            │        │         (Querying chess engine)          │   │
            │        └────────────────────┬────────────────────┘   │
            │                             │                         │
            │                             ▼                         │
            │        ┌─────────────────────────────────────────┐   │
            │        │            EXECUTING_MOVE                │   │
            │        │        (Moving piece via gantry)         │   │
            │        └────────────────────┬────────────────────┘   │
            │                             │ Move complete           │
            └─────────────────────────────┴─────────────────────────┘
                              │
                              ▼
                          GAME_OVER
                    (Checkmate/Draw/Resign)
```

---

## Move Validation

### Process

1. **Capture new board state** (FEN)
2. **Compare to previous state**
3. **Infer move from differences**
4. **Validate against python-chess**

### Implementation

```python
import chess

def validate_move(previous_fen, current_fen):
    # Create board from previous position
    board = chess.Board(previous_fen)
    
    # Find differences
    differences = find_board_differences(previous_fen, current_fen)
    
    # Infer move (e.g., e2 emptied, e4 filled = e2e4)
    inferred_move = infer_move_from_differences(differences)
    
    if inferred_move is None:
        return False, "Could not determine move"
    
    # Check if move is legal
    try:
        move = chess.Move.from_uci(inferred_move)
        if move in board.legal_moves:
            return True, move
        else:
            return False, f"Illegal move: {inferred_move}"
    except ValueError:
        return False, f"Invalid move format: {inferred_move}"
```

### Special Move Handling

| Move Type | Detection Pattern | Validation |
|-----------|-------------------|------------|
| Standard | 1 square emptied, 1 filled | Normal check |
| Capture | 1 emptied, 1 changes color | Normal check |
| Castling | King moves 2 squares, rook jumps | Check castling rights |
| En passant | Pawn moves diagonally, non-adjacent emptied | Check en passant square |
| Promotion | Pawn reaches 8th rank | Check legal promotions |

---

## Chess Engine Integration

### Stockfish Interface

Using `python-chess` library:

```python
import chess.engine

class ChessEngineInterface:
    def __init__(self, stockfish_path="/usr/bin/stockfish"):
        self.engine = chess.engine.SimpleEngine.popen_uci(stockfish_path)
        self.engine.configure({"Skill Level": 10})
    
    def get_best_move(self, fen, time_limit=1.0):
        board = chess.Board(fen)
        result = self.engine.play(
            board,
            chess.engine.Limit(time=time_limit)
        )
        return result.move.uci()
    
    def close(self):
        self.engine.quit()
```

### Engine Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Skill Level | 0-20 | Engine strength (10 = intermediate) |
| Time Limit | 1.0s | Maximum thinking time |
| Depth | 15 | Search depth (if not using time limit) |
| Threads | 1 | CPU threads (Pi limited) |
| Hash | 64 | Hash table size (MB) |

<!-- USER_ATTENTION: Tune these parameters for desired difficulty -->

### Installing Stockfish

```bash
# On Raspberry Pi (Ubuntu)
sudo apt install stockfish

# Verify
stockfish --version
```

---

## Move Execution

### Standard Move Sequence

```
1. Parse move (e.g., "e2e4")
2. Calculate source square center (e2 → x=125mm, y=50mm)
3. Calculate target square center (e4 → x=125mm, y=100mm)
4. Execute sequence:
   a. Move to source, lower magnet, engage
   b. Move to target
   c. Lower, release magnet, raise
```

### Capture Sequence

```
1. Parse move (e.g., "d4e5" capturing piece on e5)
2. FIRST: Move captured piece to graveyard
   a. Go to e5, engage magnet
   b. Move to next graveyard slot
   c. Release
3. THEN: Move capturing piece
   a. Go to d4, engage magnet
   b. Move to e5
   c. Release
```

### Castling Sequence

```
Kingside castling (e1g1):
1. Move King: e1 → g1
2. Move Rook: h1 → f1

Queenside castling (e1c1):
1. Move King: e1 → c1
2. Move Rook: a1 → d1
```

### En Passant Sequence

```
En passant (e.g., white pawn d5 captures e5 en passant):
1. Move captured pawn (e5) to graveyard FIRST
2. Move capturing pawn: d5 → e6
```

---

## Game End Detection

### Checkmate

```python
def check_game_end(board):
    if board.is_checkmate():
        winner = "Black" if board.turn == chess.WHITE else "White"
        return True, f"Checkmate! {winner} wins."
    return False, None
```

### Draw Conditions

| Condition | Detection |
|-----------|-----------|
| Stalemate | No legal moves, not in check |
| Insufficient material | Only kings, or K vs K+B/N |
| Threefold repetition | Same position 3 times |
| 50-move rule | 50 moves without pawn move or capture |

### Resignation

- Via button press (if implemented)
- Via timeout (chess clock)

---

## Time Control

### Clock Management

```python
class ChessClock:
    def __init__(self, time_per_player=600):  # 10 minutes
        self.time_white = time_per_player
        self.time_black = time_per_player
        self.current_turn = chess.WHITE
        self.running = False
        self.last_tick = None
    
    def start(self, color):
        self.current_turn = color
        self.running = True
        self.last_tick = time.time()
    
    def tick(self):
        if not self.running:
            return
        now = time.time()
        elapsed = now - self.last_tick
        self.last_tick = now
        
        if self.current_turn == chess.WHITE:
            self.time_white -= elapsed
            if self.time_white <= 0:
                return "Black wins on time"
        else:
            self.time_black -= elapsed
            if self.time_black <= 0:
                return "White wins on time"
        return None
```

---

## Error Handling

### Recovery Strategies

| Error | Detection | Recovery |
|-------|-----------|----------|
| Invalid move | Validation fails | Prompt user to correct |
| Board changed during move | Position mismatch | Re-capture and verify |
| Engine timeout | No response | Use default move or ask user |
| Piece knocked over | Unexpected board state | Wait for user to fix |

### User Feedback

- LED indicators for errors
- Audio beeps (if implemented)
- Clock display messages

---

## Configuration

```yaml
game_manager_node:
  ros__parameters:
    time_control: 600           # Seconds per player
    engine_think_time: 1.0      # Seconds
    move_validation_timeout: 5.0
    capture_retry_count: 3
    engine_skill_level: 10
```

---

*See [architecture.md](../software/architecture.md) for system overview.*
