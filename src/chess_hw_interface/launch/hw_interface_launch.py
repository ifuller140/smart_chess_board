from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get config file
    config_dir = os.path.join(get_package_share_directory('chess_hw_interface'), 'config')
    pins_config = os.path.join(config_dir, 'pins.yaml')

    return LaunchDescription([
        Node(
            package='chess_hw_interface',
            executable='stepper_driver_node',
            name='stepper_driver',
            parameters=[pins_config]
        ),
        Node(
            package='chess_hw_interface',
            executable='servo_node',
            name='servo_node',
            parameters=[pins_config]
        ),
        Node(
            package='chess_hw_interface',
            executable='limit_switch_node',
            name='limit_switch_node',
            parameters=[pins_config]
        ),
        # Gantry Kinematics (Logic layer, but often run with HW)
        Node(
            package='gantry_control',
            executable='gantry_kinematics_node',
            name='gantry_kinematics',
            parameters=[{'steps_per_mm': 5.0}]
        )
    ])
