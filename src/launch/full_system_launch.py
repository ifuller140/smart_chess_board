"""
Full System Launch File — Smart Chess Board.

Launches ALL nodes for a complete game session in dependency order:
  Layer 1: Hardware Interface  (GPIO/steppers/servos/switches/display)
  Layer 2: Gantry Control      (kinematics/homing/motion planner)
  Layer 3: Perception          (camera/board detector/piece detector)
  Layer 4: Chess Logic         (engine/clock/game manager)

Usage:
  ros2 launch smart_chess_board full_system_launch.py
  ros2 launch smart_chess_board full_system_launch.py engine_path:=/path/to/stockfish
  ros2 launch smart_chess_board full_system_launch.py use_picamera2:=False time_per_player_s:=300.0

Prerequisites:
  sudo pigpiod
  colcon build && source install/setup.bash
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hw_config_dir     = os.path.join(
        get_package_share_directory('chess_hw_interface'), 'config')
    gantry_config_dir = os.path.join(
        get_package_share_directory('gantry_control'), 'config')

    pins_yaml      = os.path.join(hw_config_dir, 'pins.yaml')
    board_map_yaml = os.path.join(gantry_config_dir, 'board_map.yaml')

    # ── Launch arguments ───────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('engine_path', default_value='/usr/games/stockfish',
                              description='Path to chess engine binary'),
        DeclareLaunchArgument('think_time',  default_value='2.0',
                              description='Engine think time in seconds'),
        DeclareLaunchArgument('time_per_player_s', default_value='600.0',
                              description='Clock time per player (seconds)'),
        DeclareLaunchArgument('use_picamera2', default_value='False',
                              description='True = Pi Camera v2 (CSI), False = OpenCV V4L2 (default: V4L2 due to libcamera 0.2.0 IPA bug)'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to camera_calibration.yaml'),
    ]

    engine_path     = LaunchConfiguration('engine_path')
    think_time      = LaunchConfiguration('think_time')
    time_per_player = LaunchConfiguration('time_per_player_s')
    use_picam       = LaunchConfiguration('use_picamera2')
    cal_file        = LaunchConfiguration('calibration_file')

    return LaunchDescription(args + [

        LogInfo(msg='═══ Smart Chess Board — Full System Startup ═══'),

        # ═══════════════════════════════════════════════════════
        # LAYER 1 — Hardware Interface (GPIO ownership)
        # ═══════════════════════════════════════════════════════

        Node(package='chess_hw_interface', executable='stepper_driver_node',
             name='stepper_driver_node', parameters=[pins_yaml], output='screen'),

        Node(package='chess_hw_interface', executable='servo_node',
             name='servo_node', parameters=[pins_yaml], output='screen'),

        Node(package='chess_hw_interface', executable='limit_switch_node',
             name='limit_switch_node', parameters=[pins_yaml], output='screen'),

        Node(package='chess_hw_interface', executable='clock_servo_node',
             name='clock_servo_node', parameters=[pins_yaml], output='screen'),

        Node(package='chess_hw_interface', executable='clock_display_node',
             name='clock_display_node', parameters=[pins_yaml], output='screen'),

        Node(package='chess_hw_interface', executable='gpio_watchdog_node',
             name='gpio_watchdog_node', output='screen'),

        Node(package='chess_hw_interface', executable='test_runner_node',
             name='test_runner_node', output='screen'),

        # ═══════════════════════════════════════════════════════
        # LAYER 2 — Gantry Control
        # ═══════════════════════════════════════════════════════

        Node(package='gantry_control', executable='gantry_kinematics_node',
             name='gantry_kinematics_node',
             parameters=[board_map_yaml], output='screen'),

        Node(package='gantry_control', executable='homing_node',
             name='homing_node',
             parameters=[pins_yaml, board_map_yaml], output='screen'),

        Node(package='gantry_control', executable='motion_planner_node',
             name='motion_planner_node',
             parameters=[board_map_yaml], output='screen'),

        # ═══════════════════════════════════════════════════════
        # LAYER 3 — Perception
        # ═══════════════════════════════════════════════════════

        Node(package='chess_perception', executable='camera_node',
             name='camera_node',
             parameters=[{
                 'use_picamera2':    use_picam,
                 'width':            1280,
                 'height':           720,
                 'fps':              5.0,
                 'calibration_file': cal_file,
             }],
             output='screen'),

        Node(package='chess_perception', executable='board_detector_node',
             name='board_detector_node', output='screen'),

        Node(package='chess_perception', executable='piece_detector_node',
             name='piece_detector_node',
             parameters=[{
                 'occupancy_diff_threshold': 25,
                 'white_piece_brightness':   0.65,
             }],
             output='screen'),

        # ═══════════════════════════════════════════════════════
        # LAYER 4 — Chess Logic
        # ═══════════════════════════════════════════════════════

        Node(package='chess_logic', executable='chess_engine_node',
             name='chess_engine_node',
             parameters=[{
                 'engine_path': engine_path,
                 'think_time':  think_time,
             }],
             output='screen'),

        Node(package='chess_hw_interface', executable='chess_clock_node',
             name='chess_clock_node',
             parameters=[{'time_per_player_s': time_per_player}],
             output='screen'),

        Node(package='chess_logic', executable='game_manager_node',
             name='game_manager_node',
             parameters=[{
                 'engine_think_time_s':     think_time,
                 'board_capture_timeout_s': 5.0,
                 'motion_timeout_s':        120.0,
                 'homing_timeout_s':        90.0,
             }],
             output='screen'),

        # ═══════════════════════════════════════════════════════
        # LAYER 5 — Chess OS (web UI)
        # ═══════════════════════════════════════════════════════
        # Uses ExecuteProcess rather than the Node action: chess_ui's argparse
        # entrypoint isn't itself the rclpy node (it spins one on a background
        # thread), so it doesn't expect the --ros-args launch_ros.Node injects.

        ExecuteProcess(
            cmd=['ros2', 'run', 'chess_ui', 'chess_ui'],
            output='screen',
        ),

        LogInfo(msg='All nodes launched — waiting for system to initialize...'),
    ])
