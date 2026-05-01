#!/usr/bin/env python3
"""
Live Camera Preview — Smart Chess Board
Saves frames every second to /tmp/chess_preview/ so you can verify the camera
feed without a display (scp the latest file to view it on your Mac).

Usage:
    python3 code/live_camera_preview.py               # V4L2 /dev/video0
    python3 code/live_camera_preview.py --device 1    # /dev/video1
    python3 code/live_camera_preview.py --picam       # picamera2 (if installed)
    python3 code/live_camera_preview.py --save-dir /tmp/preview
"""

import argparse
import os
import sys
import time
import signal

import cv2

SAVE_DIR = "/tmp/chess_preview"
LATEST = os.path.join(SAVE_DIR, "latest.jpg")


def open_v4l2(device_id: int, width: int = 1280, height: int = 720):
    cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
    if not cap.isOpened():
        # Try without explicit backend
        cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open /dev/video{device_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, 5)
    # Flush buffer
    for _ in range(5):
        cap.read()
    return cap


def open_picam(width: int = 1280, height: int = 720):
    from picamera2 import Picamera2
    picam = Picamera2()
    cfg = picam.create_video_configuration(main={"size": (width, height)})
    picam.configure(cfg)
    picam.start()
    time.sleep(1.5)
    return picam


def main():
    parser = argparse.ArgumentParser(description="Live camera headless preview")
    parser.add_argument("--device", "-d", type=int, default=0, help="V4L2 device index")
    parser.add_argument("--picam", action="store_true", help="Use picamera2")
    parser.add_argument("--save-dir", default=SAVE_DIR, help="Directory for saved frames")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between saves")
    args = parser.parse_args()

    os.makedirs(args.save_dir, exist_ok=True)
    latest_path = os.path.join(args.save_dir, "latest.jpg")

    print("=" * 60)
    print("CHESS CAMERA LIVE PREVIEW")
    print("=" * 60)
    print(f"Save dir : {args.save_dir}")
    print(f"Interval : {args.interval}s")
    print(f"Backend  : {'picamera2' if args.picam else f'V4L2 /dev/video{args.device}'}")
    print()
    print(f"Frames saved to: {latest_path}")
    print("Press Ctrl+C to stop.")
    print()

    running = True
    def _stop(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    # Open camera
    picam = None
    cap = None
    try:
        if args.picam:
            picam = open_picam(args.width, args.height)
            print(f"picamera2 open: {args.width}x{args.height}")
        else:
            cap = open_v4l2(args.device, args.width, args.height)
            actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"V4L2 open: {actual_w}x{actual_h}")
    except Exception as e:
        print(f"ERROR: {e}")
        sys.exit(1)

    frame_count = 0
    start_time = time.time()

    while running:
        t0 = time.time()

        if picam:
            frame = picam.capture_array("main")
            # picamera2 returns RGB, convert to BGR for OpenCV
            if frame.shape[2] == 3:
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        else:
            ret, frame = cap.read()
            if not ret:
                print("WARN: Frame read failed, retrying...")
                time.sleep(0.5)
                continue

        frame_count += 1
        elapsed = time.time() - start_time
        fps = frame_count / elapsed if elapsed > 0 else 0

        # Overlay info
        h, w = frame.shape[:2]
        overlay = frame.copy()
        cv2.putText(overlay, f"Frame #{frame_count}  {w}x{h}  {fps:.1f}fps",
                    (10, 35), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        cv2.putText(overlay, f"Time: {elapsed:.1f}s",
                    (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        # Save latest
        cv2.imwrite(latest_path, overlay)

        # Also save timestamped copy every 5 frames
        if frame_count % 5 == 0:
            ts_path = os.path.join(args.save_dir, f"frame_{frame_count:04d}.jpg")
            cv2.imwrite(ts_path, overlay)

        print(f"\r  Frame {frame_count:4d} | {w}x{h} | {fps:.1f} fps | saved → {latest_path}   ", end="", flush=True)

        # Sleep for interval
        elapsed_this = time.time() - t0
        sleep_time = max(0, args.interval - elapsed_this)
        time.sleep(sleep_time)

    print("\nStopped.")

    if picam:
        picam.stop()
        picam.close()
    elif cap:
        cap.release()


if __name__ == "__main__":
    main()
