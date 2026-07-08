# Piece Detection System

> **Computer vision pipeline for board state detection.**

> [!NOTE]
> **Superseded by [vision-system.md](vision-system.md).** This document describes an early design (Hough-line board detection, HSV color thresholding) that predates the actual `chess_perception` implementation. `vision-system.md` documents the current ROS perception stack, calibration workflow, and web interfaces. This page is kept for background on FEN encoding and move-detection edge cases (castling, en passant, promotion), which are still accurate, but don't treat the detection-algorithm sections below as current.

## Overview

The vision system captures images of the chess board and determines the position and type of each piece. The pipeline uses classical computer vision techniques (no machine learning) for simplicity and reliability.

## Pipeline Overview

```
┌─────────────────┐
│  Camera Image   │
│    (640×480)    │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Board Detection │ ← Find corners, perspective transform
│  (Hough Lines)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Grid Extraction │ ← Divide into 64 squares
│                 │
└────────┬────────┘
         │
         ▼
┌─────────────────────┐
│ Square Analysis (×64)│
│  ├─ Empty?           │
│  ├─ White piece?     │
│  └─ Black piece?     │
└────────┬─────────────┘
         │
         ▼
┌─────────────────┐
│  FEN Generation │
│  "rnbqkbnr/..."│
└─────────────────┘
```

---

## Board Detection

### Algorithm

1. **Grayscale conversion**
2. **Canny edge detection** (thresholds: 50, 150)
3. **Hough line transform** (detect straight edges)
4. **Filter lines** (horizontal and vertical groups)
5. **Find intersections** (grid points)
6. **Identify corners** (4 extreme points)
7. **Perspective transform** (rectify to square)

### Expected Input

```
┌────────────────────────────────────┐
│                                    │
│      ╱────────────────────╲        │
│     ╱                      ╲       │
│    │ ♜ ♞ ♝ ♛ ♚ ♝ ♞ ♜ │       │
│    │ ♟ ♟ ♟ ♟ ♟ ♟ ♟ ♟ │       │
│    │                    │       │
│    │                    │       │
│    │                    │       │
│    │ ♙ ♙ ♙ ♙ ♙ ♙ ♙ ♙ │       │
│    │ ♖ ♘ ♗ ♕ ♔ ♗ ♘ ♖ │       │
│     ╲                      ╱       │
│      ╲────────────────────╱        │
│                                    │
└────────────────────────────────────┘
         Raw camera image
         (perspective distortion)
```

### Rectified Output

```
┌────────────────────────────────────┐
│ ♜ │ ♞ │ ♝ │ ♛ │ ♚ │ ♝ │ ♞ │ ♜ │
├───┼───┼───┼───┼───┼───┼───┼───┤
│ ♟ │ ♟ │ ♟ │ ♟ │ ♟ │ ♟ │ ♟ │ ♟ │
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │   │   │   │   │   │   │   │
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │   │   │   │   │   │   │   │
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │   │   │   │   │   │   │   │
├───┼───┼───┼───┼───┼───┼───┼───┤
│   │   │   │   │   │   │   │   │
├───┼───┼───┼───┼───┼───┼───┼───┤
│ ♙ │ ♙ │ ♙ │ ♙ │ ♙ │ ♙ │ ♙ │ ♙ │
├───┼───┼───┼───┼───┼───┼───┼───┤
│ ♖ │ ♘ │ ♗ │ ♕ │ ♔ │ ♗ │ ♘ │ ♖ │
└───┴───┴───┴───┴───┴───┴───┴───┘
         Rectified image
         (perspective corrected)
```

---

## Piece Detection

### Method: Color Histogram Analysis

For each square:

1. Extract square region (padding for piece overflow)
2. Convert to HSV color space
3. Analyze saturation and value channels
4. Classify as:
   - **Empty**: Low overall color variation
   - **White piece**: High value, low saturation
   - **Black piece**: Low value, any saturation

### Detection Thresholds

<!-- USER_ATTENTION: Tune these thresholds for your lighting conditions -->

```yaml
empty_threshold: 0.3        # Below this = empty square
white_value_min: 180        # V channel minimum for white pieces
black_value_max: 80         # V channel maximum for black pieces
saturation_ignore: 30       # Below this, ignore hue (grayscale)
```

### Piece Type Identification (Optional)

For advanced piece type detection (not just color):

| Method | Pros | Cons |
|--------|------|------|
| Size/Height | Simple | Needs Z-axis or shadows |
| Contour shape | No ML needed | Lighting dependent |
| Template matching | Accurate | Computationally expensive |
| Machine learning | Most accurate | Training data needed |

**Current Implementation**: Color only (white/black/empty)

---

## FEN Generation

### FEN Format

```
rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1
└───────────────────┬───────────────────┘ │  │   │ │ │
                    │                     │  │   │ │ └─ Fullmove number
                    │                     │  │   │ └─── Halfmove clock
                    │                     │  │   └───── En passant square
                    │                     │  └───────── Castling rights
                    │                     └──────────── Side to move
                    └────────────────────────────────── Piece placement
```

### Piece Encoding

| Character | Piece |
|-----------|-------|
| K/k | King (white/black) |
| Q/q | Queen |
| R/r | Rook |
| B/b | Bishop |
| N/n | Knight |
| P/p | Pawn |
| 1-8 | Empty squares |

### Generation Algorithm

```python
def board_to_fen(pieces_64):
    fen_rows = []
    for rank in range(7, -1, -1):  # 8 to 1
        row = ""
        empty_count = 0
        for file in range(8):  # a to h
            piece = pieces_64[rank * 8 + file]
            if piece == 0:
                empty_count += 1
            else:
                if empty_count > 0:
                    row += str(empty_count)
                    empty_count = 0
                row += piece_to_char(piece)
        if empty_count > 0:
            row += str(empty_count)
        fen_rows.append(row)
    return "/".join(fen_rows)
```

---

## Move Detection

### Difference-Based Detection

Compare consecutive board states:

```
Previous FEN: rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR
Current FEN:  rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR

Differences:
  - e2: was P, now empty
  - e4: was empty, now P
  
Detected move: e2e4
```

### Edge Cases

| Scenario | Detection | Notes |
|----------|-----------|-------|
| Regular move | 1 piece changes squares | Simple case |
| Capture | 1 piece disappears, 1 moves | Target square color changes |
| Castling | King + Rook move | Special handling needed |
| En passant | Pawn moves diagonally, another disappears | Special handling needed |
| Promotion | Pawn reaches 8th rank | Piece type changes |

---

## Camera Calibration

### Intrinsic Calibration (Optional)

For lens distortion correction:

```bash
# Capture checkerboard images
# Run OpenCV calibration
python3 calibrate_camera.py
# Outputs camera_matrix.npy, distortion_coeffs.npy
```

### Extrinsic Calibration

Camera mounting requirements:
- Directly above board center
- Perpendicular to board surface
- Minimal shadows
- Consistent lighting

<!-- USER_ATTENTION: Define exact camera height and position -->

---

## Lighting Considerations

### Ideal Conditions

- Diffuse overhead lighting
- No direct shadows on squares
- Consistent color temperature
- Avoid reflections on glossy pieces

### Problem Mitigation

| Problem | Solution |
|---------|----------|
| Shadows | Add fill lighting or use diffuser |
| Reflections | Matte piece finish, polarizing filter |
| Variable light | Use fixed artificial lighting |
| Color cast | White balance calibration |

---

## Performance

### Target Metrics

| Metric | Target | Actual |
|--------|--------|--------|
| Detection time | <500ms | TBD |
| Accuracy (occupied/empty) | >99% | TBD |
| Accuracy (piece color) | >95% | TBD |
| False positives | <1% | TBD |

<!-- USER_ATTENTION: Fill in actual metrics after testing -->

---

## Troubleshooting

| Symptom | Cause | Solution |
|---------|-------|----------|
| Board not detected | Poor edge contrast | Improve lighting |
| Wrong corners | Reflections | Reduce glare |
| Empty squares detected as occupied | Shadows | Add fill light |
| Wrong piece color | Lighting color cast | White balance |
| Inconsistent results | Camera movement | Secure camera mount |

---

*See [camera specs](../hardware/components.md#camera-module) for hardware details.*
