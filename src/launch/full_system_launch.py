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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                             LogInfo)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    hw_config_dir     = os.path.join(
        get_package_share_directory('chess_hw_interface'), 'config')
    gantry_config_dir = os.path.join(
        get_package_share_directory('gantry_control'), 'config')
    perception_launch_file = os.path.join(
        get_package_share_directory('chess_perception'), 'launch', 'perception_launch.py')

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
        # Layer 3 (camera/board_detector/piece_detector) is delegated entirely
        # to chess_perception's own perception_launch.py below — these args
        # are just forwarded to it, so this file can never again silently
        # diverge from the camera config that's actually been live-verified
        # (see perception_launch.py's own docstring for the camera-framing
        # bug this centralization fixes).
        DeclareLaunchArgument('use_camera_ros', default_value='True',
                              description='True = ros-humble-camera-ros/libcamera (recommended, '
                                          'live-verified), False = chess_perception\'s own '
                                          'Python camera_node fallback (less-tested path)'),
        DeclareLaunchArgument('use_picamera2', default_value='True',
                              description='(only when use_camera_ros=False) True=picamera2, False=OpenCV V4L2'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to camera_calibration.yaml (only used by chess_perception camera_node)'),
    ]

    engine_path     = LaunchConfiguration('engine_path')
    think_time      = LaunchConfiguration('think_time')
    time_per_player = LaunchConfiguration('time_per_player_s')
    use_camera_ros  = LaunchConfiguration('use_camera_ros')
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
        # LAYER 3 — Perception (delegated to perception_launch.py — see
        # its docstring; this used to hand-roll a divergent camera config
        # here that never got the camera-framing fix applied to it)
        # ═══════════════════════════════════════════════════════

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(perception_launch_file),
            launch_arguments={
                'use_camera_ros':    use_camera_ros,
                'use_picamera2':     use_picam,
                'calibration_file':  cal_file,
            }.items(),
        ),

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
