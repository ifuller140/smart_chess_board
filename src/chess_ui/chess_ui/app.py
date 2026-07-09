#!/usr/bin/env python3
"""
ChessOS — Master Control Interface for Smart Chess Board

Unified dashboard: live camera (MJPEG), showcase display, full debug
controls for gantry, hardware, perception, game logic, and test runner.

Usage:
    ros2 run chess_ui chess_ui [--port 5000] [--no-ros] [--camera 0]
    ros2 run chess_ui chess_ui --mode showcase
    ros2 run chess_ui chess_ui --no-ros --image /path/to/frame.jpg

Access: http://<pi-ip>:5000  (IP printed at startup)
"""

import argparse
import os
import socket
import sys
import threading
from pathlib import Path

import cv2

try:
    from flask import Flask
except ImportError:
    print("ERROR: pip3 install flask"); sys.exit(1)

try:
    from ament_index_python.packages import get_package_share_directory
    _HAS_AMENT_INDEX = True
except ImportError:
    _HAS_AMENT_INDEX = False

from . import camera as camera_mod
from . import ros_client, routes, state


def _template_folder() -> str:
    """Resolve templates/ via the installed share dir when running as a ROS
    package; fall back to the source-tree path for standalone dev use."""
    if _HAS_AMENT_INDEX:
        try:
            return os.path.join(get_package_share_directory('chess_ui'), 'templates')
        except Exception:
            pass
    return str(Path(__file__).resolve().parents[1] / 'templates')


def create_app() -> Flask:
    app = Flask(__name__, template_folder=_template_folder())
    app.logger.setLevel("ERROR")
    routes.register_routes(app)
    return app


def _get_local_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


def main():
    parser = argparse.ArgumentParser(description="ChessOS — Smart Chess Board UI")
    parser.add_argument("--port",   type=int, default=5000)
    parser.add_argument("--host",   default="0.0.0.0")
    parser.add_argument("--no-ros", action="store_true",
                        help="Disable ROS2 integration, use OpenCV camera")
    parser.add_argument("--camera", type=int, default=0,
                        help="OpenCV camera device index (no-ROS mode)")
    parser.add_argument("--image",  default=None,
                        help="Static image for offline testing")
    parser.add_argument("--mode",   choices=["showcase", "debug"], default="debug",
                        help="Initial display mode")
    args = parser.parse_args()

    app = create_app()

    state.load_calibration()
    state.load_manual_corners()

    # Static image
    if args.image:
        frame = cv2.imread(args.image)
        if frame is not None:
            with state._lock:
                state._state["raw_frame"] = frame
                state._state["cam_info"]  = f"static {frame.shape[1]}×{frame.shape[0]}"
            camera_mod.camera.update_raw(camera_mod.to_jpeg(frame, 85))
            print(f"  ✓ Static image: {args.image}")
        else:
            print(f"  ⚠  Cannot load: {args.image}")

    # Background threads
    threading.Thread(target=ros_client.jog_loop, daemon=True).start()

    # ROS2 or OpenCV camera
    if not args.no_ros:
        if not ros_client.HAS_ROS:
            print("  ⚠  rclpy not available — falling back to OpenCV camera")
            args.no_ros = True
        else:
            import rclpy
            try:
                rclpy.init()
                ros_client.ros_node = ros_client.RosNode()
                threading.Thread(
                    target=lambda: rclpy.spin(ros_client.ros_node),
                    daemon=True).start()
                print("  ✓ ROS2 node started")
                print("    Subscribing: /camera/image_raw/compressed, /game_manager/{board_fen,state,turn}")
            except Exception as e:
                print(f"  ⚠  ROS2 init failed: {e}")
                args.no_ros = True

    if args.no_ros and args.image is None:
        print(f"  OpenCV camera index {args.camera}")
        threading.Thread(
            target=camera_mod.opencv_camera_loop, args=(args.camera,),
            daemon=True).start()

    pi_ip = _get_local_ip()
    print(f"\n  ♞  ChessOS ready")
    print(f"     http://{pi_ip}:{args.port}")
    print(f"     http://localhost:{args.port}")
    print(f"     Mode:   {args.mode}")
    print(f"     ROS2:   {'enabled' if not args.no_ros else 'disabled'}")
    print(f"     Tests:  via test_runner_node (/hw_test/run action)\n")

    app.run(host=args.host, port=args.port,
            debug=False, use_reloader=False, threaded=True)


if __name__ == "__main__":
    main()
