"""
Perception Test Launch File.

Same as perception_launch.py but with all tuning parameters exposed.

Camera backend (choose one):
  use_camera_ros:=True  (default) — ros-humble-camera-ros + libcamera (Ubuntu 22.04)
  use_camera_ros:=False           — chess_perception camera_node (needs python3-libcamera)

After launching, verify with:
  ros2 topic hz /camera/image_raw              # should be ~5Hz
  ros2 topic echo /perception/board_geometry   # 4 non-zero corners
  ros2 service call /perception/capture_reference std_srvs/srv/Trigger {}
  ros2 topic echo /perception/board_state      # FEN string

To view debug images (if display is available):
  ros2 run image_view image_view --ros-args -r image:=/perception/board_debug
  ros2 run image_view image_view --ros-args -r image:=/perception/piece_debug

To save debug frames headlessly (scp them to your Mac to view):
  watch -n 2 "ros2 topic echo /perception/board_debug --once > /dev/null && echo saved"
  # Frames are not saved automatically; use a script or rqt_image_view on X11

Usage:
  ros2 launch chess_perception perception_test_launch.py
  ros2 launch chess_perception perception_test_launch.py use_camera_ros:=True
  ros2 launch chess_perception perception_test_launch.py use_camera_ros:=False use_picamera2:=True
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            'use_camera_ros', default_value='True',
            description='True=ros-humble-camera-ros (Ubuntu 22.04 Pi), False=custom camera_node'),
        DeclareLaunchArgument(
            'use_picamera2', default_value='True',
            description='(use_camera_ros=False only) True=picamera2, False=OpenCV V4L2'),
        DeclareLaunchArgument(
            'camera_id', default_value='0',
            description='OpenCV device index (use_camera_ros=False, use_picamera2=False only)'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='Path to calibration.yaml'),
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
            description='Auto-capture empty-board reference frame at startup'),
    ]

    use_camera_ros = LaunchConfiguration('use_camera_ros')
    use_picam      = LaunchConfiguration('use_picamera2')
    camera_id      = LaunchConfiguration('camera_id')
    cal_file       = LaunchConfiguration('calibration_file')
    warp_size      = LaunchConfiguration('warp_size')
    occ_thresh     = LaunchConfiguration('occupancy_threshold')
    white_thresh   = LaunchConfiguration('white_threshold')
    auto_cap       = LaunchConfiguration('auto_capture_reference')

    return LaunchDescription(args + [

        LogInfo(msg='[perception_test] Starting perception test stack...'),
        LogInfo(msg='[perception_test] Monitor topics:'),
        LogInfo(msg='[perception_test]   ros2 topic hz /camera/image_raw'),
        LogInfo(msg='[perception_test]   ros2 topic echo /perception/board_geometry'),
        LogInfo(msg='[perception_test]   ros2 topic echo /perception/board_state'),

        # ── ros-humble-camera-ros backend (default, works on Ubuntu 22.04 Pi) ──
        # camera_ros publishes /camera_node/image_raw — remap to /camera/image_raw
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'width':  640,
                'height': 480,
                'format': 'BGR888',
            }],
            remappings=[
                ('/camera_node/image_raw',            '/camera/image_raw'),
                ('/camera_node/image_raw/compressed', '/camera/image_raw/compressed'),
                ('/camera_node/camera_info',          '/camera/camera_info'),
            ],
            condition=IfCondition(use_camera_ros),
            output='screen',
        ),

        # ── chess_perception camera_node (fallback, needs python3-libcamera) ──
        Node(
            package='chess_perception',
            executable='camera_node',
            name='chess_camera_node',
            parameters=[{
                'use_picamera2':    use_picam,
                'camera_id':        camera_id,
                'width':            1280,
                'height':           720,
                'fps':              5.0,
                'calibration_file': cal_file,
            }],
            condition=UnlessCondition(use_camera_ros),
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

        LogInfo(msg='[perception_test] All nodes started.'),
    ])
