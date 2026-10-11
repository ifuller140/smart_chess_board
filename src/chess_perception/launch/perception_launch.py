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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# How long after launch to wait before the first capture_premove call (seconds).
# The camera and piece_detector_node need time to fully initialize.
_CAPTURE_START_DELAY_S = 8.0

# Number of capture_premove calls to make, spaced 3s apart.
_CAPTURE_COUNT = 5
_CAPTURE_INTERVAL_S = 3.0


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
        # Auto-restart camera/board/piece nodes individually if one exits
        # (e.g. the documented camera_ros stale-subscriber-after-2h bug).
        # Deliberately scoped to this perception+UI layer only — NOT applied
        # to hardware/gantry nodes elsewhere, since respawning those alone
        # without a full re-home risks operating on stale position
        # assumptions after a crash mid-motion. Off by default for
        # interactive/manual launches (Ctrl+C during dev would otherwise
        # try to relaunch a node mid-shutdown); the production systemd unit
        # (setup/smart-chess.service) is expected to pass respawn:=True.
        DeclareLaunchArgument(
            'respawn', default_value='False',
            description='Auto-restart camera/board/piece nodes individually on exit'),
    ]

    use_camera_ros = LaunchConfiguration('use_camera_ros')
    use_picam      = LaunchConfiguration('use_picamera2')
    cal_file       = LaunchConfiguration('calibration_file')
    width          = LaunchConfiguration('width')
    height         = LaunchConfiguration('height')
    fps            = LaunchConfiguration('fps')
    respawn        = LaunchConfiguration('respawn')

    # ── Camera backend: ros-humble-camera-ros (preferred on Ubuntu 22.04) ──
    # camera_ros publishes to /camera_node/image_raw — remap to /camera/image_raw
    # using fully-qualified path since camera_ros doesn't honour relative remaps.
    camera_ros_node = Node(
            package='camera_ros',
            executable='camera_node',
            name='camera_node',
            parameters=[{
                'width':  width,
                'height': height,
                'format': 'BGR888',
                # Without this, libcamera's own mode-selection heuristic picks
                # a raw sensor mode independently of width/height above — for
                # this sensor+role+size combination it silently chose a
                # dead-centered ~39%x39% crop of the full array (confirmed via
                # `ros2 param get /camera_node ScalerCrop` defaulting to
                # {(1000,752)/1280x960} out of the full 3280x2464 array), i.e.
                # the whole board was never in frame, only the center ~4
                # squares. Pinning sensor_mode to the sensor's native 2x2-
                # binned full-FOV raw mode (1640x1232) forces the ISP to crop
                # the FULL array before scaling down to width x height, so the
                # entire board is back in frame regardless of the final
                # (lower-res, CPU-friendly) stream size. Confirmed live:
                # ScalerCrop becomes {(0,0)/3280x2460} (whole array) with this
                # set. This mirrors the identical fix already applied to our
                # own Python fallback camera_node.py's picamera2 backend
                # (see its `sensor={'output_size': (1640, 1232)}` config).
                'sensor_mode': '1640:1232',
            }],
            remappings=[
                ('/camera_node/image_raw',            '/camera/image_raw'),
                ('/camera_node/image_raw/compressed', '/camera/image_raw/compressed'),
                ('/camera_node/camera_info',          '/camera/camera_info'),
            ],
            condition=IfCondition(use_camera_ros),
            respawn=respawn,
            respawn_delay=2.0,
            output='screen',
        )

    # ── Camera backend: chess_perception camera_node (fallback) ──
    # Used when use_camera_ros:=False.  Requires python3-libcamera for Pi CSI.
    camera_fallback_node = Node(
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
        respawn=respawn,
        respawn_delay=2.0,
        output='screen',
    )

    board_detector_node = Node(
        package='chess_perception',
        executable='board_detector_node',
        name='board_detector_node',
        respawn=respawn,
        respawn_delay=2.0,
        output='screen',
    )

    piece_detector_node = Node(
        package='chess_perception',
        executable='piece_detector_node',
        name='piece_detector_node',
        respawn=respawn,
        respawn_delay=2.0,
        output='screen',
    )

    return LaunchDescription(args + [

        LogInfo(msg='Starting perception stack...'),

        camera_ros_node,
        camera_fallback_node,
        board_detector_node,
        piece_detector_node,

        LogInfo(msg='Perception stack ready.'),

        # ── Auto capture_premove ──────────────────────────────────────────────
        # Calls /perception/capture_premove once at startup to establish an
        # initial pre-move reference frame.  game_manager_node re-calls this
        # service before each player turn; this startup call just primes the
        # reference so the debug view is non-empty immediately.
        *[
            TimerAction(
                period=_CAPTURE_START_DELAY_S + i * _CAPTURE_INTERVAL_S,
                actions=[
                    LogInfo(msg=f'Auto capture_premove ({i + 1}/{_CAPTURE_COUNT})...'),
                    ExecuteProcess(
                        cmd=[
                            'ros2', 'service', 'call',
                            '/perception/capture_premove',
                            'std_srvs/srv/Trigger', '{}',
                        ],
                        output='screen',
                    ),
                ],
            )
            for i in range(_CAPTURE_COUNT)
        ],

        LogInfo(
            msg=f'Will auto-capture premove reference {_CAPTURE_COUNT}x '
                f'starting at {_CAPTURE_START_DELAY_S}s '
                f'(every {_CAPTURE_INTERVAL_S}s). '
                f'Board can be in any state during this window.'
        ),
    ])
