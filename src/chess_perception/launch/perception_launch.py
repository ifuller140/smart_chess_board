"""
Perception Stack Launch File.

Starts camera + board detector + piece detector.

Camera backend selection:
  use_camera_ros:=True  (default) — uses ros-humble-camera-ros with libcamera.
                                    Works on Ubuntu 22.04 + Pi Camera v2 out of the box.
                                    Install: sudo apt install ros-humble-camera-ros
  use_camera_ros:=False           — uses our custom chess_perception camera_node
                                    (requires python3-libcamera for Pi CSI camera)

Usage:
  ros2 launch chess_perception perception_launch.py
  ros2 launch chess_perception perception_launch.py use_camera_ros:=False use_picamera2:=True
  ros2 launch chess_perception perception_launch.py calibration_file:=/home/ian/.chess/calibration.yaml
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
            description='True=use ros-humble-camera-ros (recommended on Ubuntu 22.04 Pi), '
                        'False=use chess_perception camera_node (requires python3-libcamera)'),
        DeclareLaunchArgument(
            'use_picamera2', default_value='True',
            description='(only when use_camera_ros=False) True=picamera2, False=OpenCV V4L2'),
        DeclareLaunchArgument(
            'calibration_file', default_value='',
            description='Path to calibration.yaml (only used by chess_perception camera_node)'),
        DeclareLaunchArgument(
            'width', default_value='1640',
            description='Camera capture width in pixels (1640 = Pi Cam v2 full-sensor 2x2-binned)'),
        DeclareLaunchArgument(
            'height', default_value='1232',
            description='Camera capture height in pixels (1232 = Pi Cam v2 full-sensor 2x2-binned)'),
        DeclareLaunchArgument(
            'fps', default_value='5.0',
            description='Camera frame rate'),
    ]

    use_camera_ros = LaunchConfiguration('use_camera_ros')
    use_picam      = LaunchConfiguration('use_picamera2')
    cal_file       = LaunchConfiguration('calibration_file')
    width          = LaunchConfiguration('width')
    height         = LaunchConfiguration('height')
    fps            = LaunchConfiguration('fps')

    return LaunchDescription(args + [

        LogInfo(msg='Starting perception stack...'),

        # ── Camera backend: ros-humble-camera-ros (preferred on Ubuntu 22.04) ──
        # camera_ros publishes to /camera_node/image_raw — remap to /camera/image_raw
        # using fully-qualified path since camera_ros doesn't honour relative remaps.
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'width':  width,
                'height': height,
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

        # ── Camera backend: chess_perception camera_node (fallback) ──
        # Used when use_camera_ros:=False.  Requires python3-libcamera for Pi CSI.
        Node(
            package='chess_perception',
            executable='camera_node',
            name='chess_camera_node',
            parameters=[{
                'use_picamera2':    use_picam,
                'width':            width,
                'height':           height,
                'fps':              fps,
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
                'occupancy_diff_threshold': 25,
                'white_piece_brightness':   0.65,
            }],
            output='screen',
        ),

        LogInfo(msg='Perception stack ready.'),
    ])
