from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    hw_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chess_hw_interface'),
                'launch',
                'hw_interface_launch.py'
            ])
        ])
    )

    return LaunchDescription([
        hw_launch,
        
        # Perception
        Node(
            package='chess_perception',
            executable='camera_node',
            name='camera_node'
        ),
        Node(
            package='chess_perception',
            executable='board_detector_node',
            name='board_detector_node'
        ),
        Node(
            package='chess_perception',
            executable='piece_detector_node',
            name='piece_detector_node'
        ),
        
        # Logic
        Node(
            package='chess_logic',
            executable='chess_engine_node',
            name='chess_engine_node'
        ),
        Node(
            package='chess_logic',
            executable='game_manager_node',
            name='game_manager_node'
        ),
        
        # Gantry High-Level
        Node(
            package='gantry_control',
            executable='motion_planner_node',
            name='motion_planner_node'
        ),
        Node(
            package='gantry_control',
            executable='homing_node',
            name='homing_node'
        )
    ])
