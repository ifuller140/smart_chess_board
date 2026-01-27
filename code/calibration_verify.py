#!/usr/bin/env python3
"""
Calibration Verification Patterns

Run this after calibration to verify gantry accuracy.
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from calibration import (
    setup, cleanup, load_state, save_state, home_all,
    move_to_square, magnet_engage, magnet_release,
    wait_for_clock, gantry, show_status
)

def edge_perimeter():
    """Trace the board perimeter."""
    print("\n" + "="*50)
    print("EDGE PERIMETER PATTERN")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece, press clock to start")
    
    for sq in ['a1', 'h1', 'h8', 'a8', 'a1']:
        move_to_square(sq[0], int(sq[1]))
        wait_for_clock(f"Confirm centered on {sq.upper()}")
    
    magnet_release()
    save_state()
    print("\n✓ Edge perimeter complete!")

def diagonal_pattern():
    """Visit diagonal squares A1-H8."""
    print("\n" + "="*50)
    print("DIAGONAL PATTERN (A1 → H8)")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece, press clock to start")
    
    for i in range(8):
        f = chr(ord('a') + i)
        r = i + 1
        move_to_square(f, r)
        wait_for_clock(f"Confirm centered on {f.upper()}{r}")
    
    magnet_release()
    save_state()
    print("\n✓ Diagonal pattern complete!")

def full_sweep():
    """Visit all 64 squares."""
    print("\n" + "="*50)
    print("FULL SWEEP (All 64 squares)")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece, press clock to start")
    
    for rank in range(1, 9):
        files = 'abcdefgh' if rank % 2 == 1 else 'hgfedcba'
        for f in files:
            print(f"Moving to {f.upper()}{rank}...", end=" ", flush=True)
            move_to_square(f, rank)
            print("✓")
    
    wait_for_clock("Sweep complete! Press clock")
    magnet_release()
    save_state()
    print("\n✓ Full sweep complete!")

def center_test():
    """Test center squares."""
    print("\n" + "="*50)
    print("CENTER SQUARE TEST")
    print("="*50)
    
    magnet_engage()
    wait_for_clock("Place a piece, press clock to start")
    
    for sq in ['d4', 'e4', 'e5', 'd5', 'd4']:
        move_to_square(sq[0], int(sq[1]))
        wait_for_clock(f"Confirm centered on {sq.upper()}")
    
    magnet_release()
    save_state()
    print("\n✓ Center test complete!")

def main():
    setup()
    
    if not load_state():
        print("[WARNING] No calibration found!")
    
    show_status()
    
    if not gantry.homed:
        print("\n[WARNING] Gantry not homed. Homing now...")
        if not home_all():
            cleanup()
            return
    
    try:
        while True:
            print("\n" + "="*50)
            print("VERIFICATION PATTERNS")
            print("="*50)
            print("1. Edge Perimeter (A1→H1→H8→A8)")
            print("2. Diagonal (A1→B2→...→H8)")
            print("3. Full Sweep (64 squares)")
            print("4. Center Test (D4-E5)")
            print("5. Show Status")
            print("q. Quit")
            
            choice = input("\nSelect: ").strip().lower()
            
            if choice == '1':
                edge_perimeter()
            elif choice == '2':
                diagonal_pattern()
            elif choice == '3':
                full_sweep()
            elif choice == '4':
                center_test()
            elif choice == '5':
                show_status()
            elif choice == 'q':
                break
    
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        save_state()
        cleanup()
        print("Goodbye!")

if __name__ == "__main__":
    main()
