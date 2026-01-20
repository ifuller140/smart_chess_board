# Vision Calibration

> **Camera calibration and perspective correction for angled camera setup.**

## Overview

The camera is mounted at an angle to the board, requiring perspective correction to accurately detect squares and pieces.

### Camera Position
<!-- USER_ATTENTION: Verify these measurements match your actual setup -->

| Parameter | Value |
|-----------|-------|
| Horizontal offset (behind board) | 2 inches (~50mm) |
| Height above board | 7 inches (~178mm) |
| Tilt angle (from horizontal) | 45 degrees |
| Field of view | Covers all 64 squares |

```
Side View:
                                    ┌─────┐
                                    │CAMERA
                                    └──┬──┘
                                       │╲  45°
                                       │ ╲
                                       │  ╲
          7 inches                     │   ╲
                                       │    ╲
                                       │     ╲
    ─────────────────────────────┬─────┴──────╲────────
                                 │            ╲
          BOARD                  │             ╲
    ═══════════════════════════════════════════════════
                                 │
                              2 inches
                              (behind)

Top View:
    ┌───────────────────────────────┐
    │                               │
    │         CHESS BOARD           │
    │                               │
    │                               │
    └───────────────────────────────┘
                   │
                   │ 2"
                   ▼
                ┌─────┐
                │ CAM │
                └─────┘
```

---

## Calibration Process

### Step 1: Intrinsic Calibration (Camera Lens)

Corrects lens distortion using checkerboard pattern.

```bash
# Capture 10-15 images of checkerboard at different angles
python3 scripts/capture_calibration_images.py

# Run OpenCV calibration
python3 scripts/calibrate_intrinsic.py

# Outputs:
#   - camera_matrix.npy
#   - distortion_coeffs.npy
```

**Checkerboard Requirements**:
- Printed on flat, rigid surface
- Inner corners: 9×6 (standard OpenCV pattern)
- Square size: 25mm (match chess board squares)

### Step 2: Extrinsic Calibration (Camera Position)

Maps camera view to real-world board coordinates.

#### Method A: Corner Detection (Automated)

```python
import cv2
import numpy as np

def detect_board_corners(image):
    """Detect the four corners of the chess board."""
    
    # Convert to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Edge detection
    edges = cv2.Canny(gray, 50, 150)
    
    # Find lines
    lines = cv2.HoughLines(edges, 1, np.pi/180, 100)
    
    # Find intersections for grid
    intersections = find_line_intersections(lines)
    
    # Identify outer corners (4 extreme points)
    corners = find_extreme_corners(intersections)
    
    return corners  # [top-left, top-right, bottom-right, bottom-left]
```

#### Method B: Manual Corner Selection

```python
def manual_corner_selection(image):
    """Let user click on the four corners."""
    corners = []
    
    def click_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            corners.append((x, y))
            cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Click corners: TL, TR, BR, BL", image)
    
    cv2.imshow("Click corners: TL, TR, BR, BL", image)
    cv2.setMouseCallback("Click corners: TL, TR, BR, BL", click_callback)
    
    while len(corners) < 4:
        cv2.waitKey(100)
    
    return corners
```

### Step 3: Perspective Transform

Warps the angled view into a top-down square image.

```python
def compute_perspective_transform(corners, output_size=640):
    """Compute homography matrix for perspective correction."""
    
    # Source points (detected corners in image)
    src = np.float32(corners)
    
    # Destination points (perfect square)
    dst = np.float32([
        [0, 0],                          # Top-left
        [output_size, 0],                # Top-right
        [output_size, output_size],      # Bottom-right
        [0, output_size]                 # Bottom-left
    ])
    
    # Compute homography
    H, _ = cv2.findHomography(src, dst)
    
    return H

def apply_perspective_transform(image, H, output_size=640):
    """Apply perspective warp to image."""
    return cv2.warpPerspective(image, H, (output_size, output_size))
```

---

## Visual Pipeline

```
┌─────────────────┐
│  Raw Camera     │
│  Image (angled) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Undistort       │  ← Apply camera_matrix + distortion_coeffs
│ (lens correct)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Detect Corners  │  ← Find 4 board corners
│ (a1, a8, h8, h1)│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Perspective     │  ← Apply homography matrix
│ Transform       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Rectified       │
│ Top-Down View   │  ← Perfect 8x8 grid
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Extract Squares │  ← 64 individual square images
│ (80x80 each)    │
└─────────────────┘
```

---

## Calibration Procedure (Step-by-Step)

### Hardware Test: Camera Calibration

```
┌─────────────────────────────────────────────────────────────┐
│ STEP 1: Capture Checkerboard Images                         │
├─────────────────────────────────────────────────────────────┤
│ Display: "CALIB 1"                                          │
│ Action: Place checkerboard on board                         │
│ Press clock button to capture                               │
│ Repeat 10-15 times at different angles                      │
│ Display: "CALIB OK" when enough images                      │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ STEP 2: Compute Lens Correction                             │
├─────────────────────────────────────────────────────────────┤
│ Display: "COMPUTE"                                          │
│ System calculates camera matrix and distortion              │
│ Display: "LENS OK" or "RETRY" if calibration fails          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ STEP 3: Board Corner Calibration                            │
├─────────────────────────────────────────────────────────────┤
│ Display: "CORNERS"                                          │
│ System attempts to auto-detect board corners                │
│ If failed, prompt for manual calibration                    │
│ Display: "CORN OK"                                          │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│ STEP 4: Verify Grid Detection                               │
├─────────────────────────────────────────────────────────────┤
│ Display: "VERIFY"                                           │
│ Show rectified image with grid overlay                      │
│ User confirms 8x8 grid aligns with squares                  │
│ Press clock button to confirm                               │
│ Display: "CAM OK"                                           │
└─────────────────────────────────────────────────────────────┘
```

---

## Configuration Files

### Calibration Output Files

```
src/chess_perception/config/
├── camera_matrix.npy          # 3x3 intrinsic matrix
├── distortion_coeffs.npy      # Distortion parameters
├── homography_matrix.npy      # Perspective transform
└── calibration_config.yaml    # Metadata
```

### calibration_config.yaml

```yaml
camera_calibration:
  ros__parameters:
    # Intrinsic calibration
    camera_matrix_file: "config/camera_matrix.npy"
    distortion_file: "config/distortion_coeffs.npy"
    
    # Extrinsic calibration
    homography_file: "config/homography_matrix.npy"
    
    # Output settings
    rectified_size: 640         # Output image size (pixels)
    square_size_pixels: 80      # 640 / 8 = 80 pixels per square
    
    # Detection parameters
    auto_detect_corners: true
    corner_detection_threshold: 0.8
    
    # Physical measurements (for validation)
    board_size_mm: 200          # 8 * 25mm squares
    camera_height_mm: 178       # 7 inches
    camera_offset_mm: 50        # 2 inches behind
    camera_angle_deg: 45
```

---

## Square Extraction

After perspective correction, extract individual squares.

```python
def extract_squares(rectified_image, square_size=80):
    """Extract 64 individual square images from rectified board."""
    squares = {}
    
    for rank in range(8):      # 1-8 (bottom to top in image)
        for file in range(8):  # a-h (left to right)
            # Calculate pixel coordinates
            x = file * square_size
            y = (7 - rank) * square_size  # Flip for correct orientation
            
            # Extract square
            square_img = rectified_image[y:y+square_size, x:x+square_size]
            
            # Name the square
            file_letter = chr(ord('a') + file)
            rank_number = rank + 1
            square_name = f"{file_letter}{rank_number}"
            
            squares[square_name] = square_img
    
    return squares
```

---

## Lighting Compensation

### Adaptive Histogram Equalization

```python
def normalize_lighting(image):
    """Normalize lighting across the board."""
    # Convert to LAB color space
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    
    # Apply CLAHE to L channel
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    l = clahe.apply(l)
    
    # Merge and convert back
    lab = cv2.merge([l, a, b])
    return cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
```

---

## Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| Warped squares | Corners detected wrong | Re-run corner calibration |
| Blurry image | Motion blur or focus | Check camera focus, add delay |
| Uneven lighting | Shadows from pieces | Add diffuse lighting |
| Grid misaligned | Camera moved | Re-calibrate homography |
| Can't see all squares | Camera too close | Increase height or use wider lens |

---

## Re-Calibration Triggers

Re-run calibration if:
- Camera has been moved or adjusted
- Board position has changed
- Lighting conditions have significantly changed
- Grid detection accuracy degrades

---

*See [piece-detection.md](piece-detection.md) for piece identification after calibration.*
