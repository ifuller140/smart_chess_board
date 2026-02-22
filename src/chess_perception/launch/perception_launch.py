"""
Perception Stack Launch File.

Starts only the nodes needed for camera and piece detection:
  - camera_node
  - board_detector_node
  - piece_detector_node

Use this when running vision tests without the full system:
  ros2 launch chess_perception perception_launch.py

Optional args:
  use_picamera2:=True          Pi Camera v2 (CSI) vs USB webcam
  calibration_file:=           Path to camera_calibration.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('use_picamera2', default_value='True',
                              description='True=Pi Camera CSI, False=USB webcam'),
        DeclareLaunchArgument('calibration_file', default_value='',
                              description='Path to camera_calibration.yaml'),
    ]

    use_picam = LaunchConfiguration('use_picamera2')
    cal_file  = LaunchConfiguration('calibration_file')

    return LaunchDescription(args + [

        LogInfo(msg='Starting perception stack (camera + board detector + piece detector)...'),

        Node(
            package='chess_perception',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'use_picamera2':    use_picam,
                'width':            1280,
                'height':           720,
                'fps':              5.0,
                'calibration_file': cal_file,
            }],
            output='screen',
        ),

        Node(
            package='chess_perception',
            executable='board_detector_node',
            name='board_detector_node',
            output='screen',
        ),

        Node(
            package='chess_perception',
            executable='piece_detector_node',
            name='piece_detector_node',
            parameters=[{
                'occupancy_diff_threshold': 25,
                'white_piece_brightness':   0.65,
            }],
            output='screen',
        ),

        LogInfo(msg='Perception stack ready. Run vision tests with: --test vision_corners / vision_board / vision_pieces / vision_squares / vision_fen'),
    ])
