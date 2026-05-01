"""
Perception Test Launch File.

Starts the full perception stack with debug-friendly settings:
  - camera_node       (Pi Camera, 5fps)
  - board_detector_node
  - piece_detector_node

After launching, verify with:
  ros2 topic hz /camera/image_raw              # should be ~5Hz
  ros2 topic echo /perception/board_geometry   # 4 non-zero corners
  ros2 service call /perception/capture_reference std_srvs/srv/Trigger {}
  ros2 topic echo /perception/board_state      # FEN string

To view debug images (if display is available):
  ros2 run image_view image_view --ros-args -r image:=/perception/board_debug
  ros2 run image_view image_view --ros-args -r image:=/perception/piece_debug

To save a debug frame to disk (headless):
  ros2 run image_transport republish raw compressed \
    --ros-args -r in:=/perception/board_debug -r out/compressed:=/perception/board_debug/compressed

Usage:
  ros2 launch chess_perception perception_test_launch.py
  ros2 launch chess_perception perception_test_launch.py use_picamera2:=False camera_id:=0
  ros2 launch chess_perception perception_test_launch.py calibration_file:=/home/ian/.chess/calibration.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'use_picamera2', default_value='True',
            description='True=Pi Camera CSI (picamera2), False=OpenCV V4L2'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='OpenCV device index (only used when use_picamera2=False)'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='Path to calibration.yaml; empty=skip undistortion'),
        DeclareLaunchArgument(
            'warp_size', default_value='480',
            description='Warped board image size in pixels (square)'),
        DeclareLaunchArgument(
            'occupancy_threshold', default_value='25',
            description='Pixel diff threshold for piece occupancy detection'),
        DeclareLaunchArgument(
            'white_threshold', default_value='0.65',
            description='Brightness fraction (0-1) above which piece is white'),
        DeclareLaunchArgument(
            'auto_capture_reference', default_value='True',
            description='Auto-capture empty-board reference at startup'),
    ]

    use_picam     = LaunchConfiguration('use_picamera2')
    camera_id     = LaunchConfiguration('camera_id')
    cal_file      = LaunchConfiguration('calibration_file')
    warp_size     = LaunchConfiguration('warp_size')
    occ_thresh    = LaunchConfiguration('occupancy_threshold')
    white_thresh  = LaunchConfiguration('white_threshold')
    auto_cap      = LaunchConfiguration('auto_capture_reference')

    return LaunchDescription(args + [

        LogInfo(msg='[perception_test] Starting perception test stack...'),
        LogInfo(msg='[perception_test] Topics to monitor:'),
        LogInfo(msg='[perception_test]   /camera/image_raw          (camera feed)'),
        LogInfo(msg='[perception_test]   /perception/board_geometry (board corners)'),
        LogInfo(msg='[perception_test]   /perception/board_debug    (annotated frame)'),
        LogInfo(msg='[perception_test]   /perception/board_state    (FEN output)'),
        LogInfo(msg='[perception_test]   /perception/piece_debug    (piece labels)'),

        Node(
            package='chess_perception',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'use_picamera2':    use_picam,
                'camera_id':        camera_id,
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
                'occupancy_diff_threshold': occ_thresh,
                'white_piece_brightness':   white_thresh,
                'warp_size':                warp_size,
                'auto_capture_reference':   auto_cap,
            }],
            output='screen',
        ),

        LogInfo(msg='[perception_test] All nodes started. Run verification:'),
        LogInfo(msg='[perception_test]   ros2 topic hz /camera/image_raw'),
    ])
