#!/usr/bin/env python3
"""
Calibration Verification Patterns

Run this after calibration to verify the gantry is properly calibrated.
Patterns:
1. Edge Perimeter - Trace the outer edge of the board
2. Diagonal Pattern - Visit A1, B2, C3 ... H8
3. Full Sweep - Visit every square in order
"""

import json
import os
import sys

# Import from calibration module
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration import (
    setup, cleanup, load_calibration, home_all,
    move_to_square, magnet_engage, magnet_release,
    wait_for_clock, calibration
)

def edge_perimeter():
    """Trace the outer edge of the board."""
    print("\n" + "="*50)
    print("EDGE PERIMETER PATTERN")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece under the magnet, then press clock")
    
    # Corner squares
    corners = [
        (0, 0, "A1"),  # Bottom-left
        (7, 0, "H1"),  # Bottom-right
        (7, 7, "H8"),  # Top-right
        (0, 7, "A8"),  # Top-left
        (0, 0, "A1"),  # Return
    ]
    
    for file_idx, rank_idx, name in corners:
        print(f"Moving to {name}...")
        move_to_square(file_idx, rank_idx)
        wait_for_clock(f"Confirm centered on {name}")
    
    magnet_release()
    print("\n✓ Edge perimeter complete!")

def diagonal_pattern():
    """Visit squares along the main diagonal."""
    print("\n" + "="*50)
    print("DIAGONAL PATTERN (A1 → H8)")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece under the magnet, then press clock")
    
    for i in range(8):
        file_char = chr(ord('A') + i)
        rank = i + 1
        square = f"{file_char}{rank}"
        
        print(f"Moving to {square}...")
        move_to_square(i, i)
        wait_for_clock(f"Confirm centered on {square}")
    
    magnet_release()
    print("\n✓ Diagonal pattern complete!")

def full_sweep():
    """Visit every square in a serpentine pattern."""
    print("\n" + "="*50)
    print("FULL SWEEP PATTERN")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece under the magnet, then press clock")
    
    for rank in range(8):
        # Serpentine: even ranks go a→h, odd ranks go h→a
        files = range(8) if rank % 2 == 0 else range(7, -1, -1)
        
        for file_idx in files:
            file_char = chr(ord('A') + file_idx)
            square = f"{file_char}{rank + 1}"
            
            print(f"Moving to {square}...", end=" ", flush=True)
            move_to_square(file_idx, rank)
            print("✓")
    
    wait_for_clock("Sweep complete! Press clock to finish")
    magnet_release()
    print("\n✓ Full sweep complete!")

def knight_tour():
    """Demo knight's tour pattern (simplified)."""
    print("\n" + "="*50)
    print("KNIGHT TOUR DEMO")
    print("="*50)
    
    # Simple knight moves from E4
    moves = [
        (4, 3, "E4"),  # Start
        (5, 5, "F6"),  # Knight move
        (3, 4, "D5"),
        (5, 3, "F4"),
        (4, 5, "E6"),
        (2, 4, "C5"),
        (4, 3, "E4"),  # Return
    ]
    
    magnet_engage()
    wait_for_clock("Place a piece under the magnet, then press clock")
    
    for file_idx, rank_idx, name in moves:
        print(f"Knight to {name}...")
        move_to_square(file_idx, rank_idx)
        wait_for_clock(f"Confirm centered on {name}")
    
    magnet_release()
    print("\n✓ Knight tour demo complete!")

def main():
    setup()
    
    if not load_calibration():
        print("[ERROR] No calibration found! Run calibration.py first.")
        cleanup()
        return
    
    print("\nCalibration loaded:")
    print(f"  Steps/inch X: {calibration['steps_per_inch_x']:.2f}")
    print(f"  Steps/inch Y: {calibration['steps_per_inch_y']:.2f}")
    
    # Home first
    print("\nHoming gantry before verification...")
    if not home_all():
        print("[ERROR] Homing failed!")
        cleanup()
        return
    
    try:
        while True:
            print("\n" + "="*50)
            print("CALIBRATION VERIFICATION PATTERNS")
            print("="*50)
            print("1. Edge Perimeter (A1→H1→H8→A8→A1)")
            print("2. Diagonal Pattern (A1→B2→...→H8)")
            print("3. Full Sweep (All 64 squares)")
            print("4. Knight Tour Demo")
            print("q. Quit")
            
            choice = input("\nSelect pattern: ").strip().lower()
            
            if choice == '1':
                edge_perimeter()
            elif choice == '2':
                diagonal_pattern()
            elif choice == '3':
                full_sweep()
            elif choice == '4':
                knight_tour()
            elif choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        cleanup()
        print("GPIO cleaned up. Goodbye!")

if __name__ == "__main__":
    main()
