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
        # ── Motor Drivers ──────────────────────────────────
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

        # ── Sensors ────────────────────────────────────────
        Node(
            package='chess_hw_interface',
            executable='limit_switch_node',
            name='limit_switch_node',
            parameters=[pins_config]
        ),

        # ── Safety ─────────────────────────────────────────
        Node(
            package='chess_hw_interface',
            executable='gpio_watchdog_node',
            name='gpio_watchdog_node',
        ),

        # ── Clock Hardware ─────────────────────────────────
        Node(
            package='chess_hw_interface',
            executable='clock_servo_node',
            name='clock_servo_node',
            parameters=[pins_config]
        ),
        Node(
            package='chess_hw_interface',
            executable='clock_display_node',
            name='clock_display_node',
            parameters=[pins_config]
        ),
        Node(
            package='chess_hw_interface',
            executable='chess_clock_node',
            name='chess_clock_node',
            output='screen',
        ),

        # ── Gantry Kinematics ──────────────────────────────
        Node(
            package='gantry_control',
            executable='gantry_kinematics_node',
            name='gantry_kinematics',
            parameters=[{'steps_per_mm': 5.0}]
        ),

        # ── Hardware Test Orchestration ─────────────────────
        Node(
            package='chess_hw_interface',
            executable='test_runner_node',
            name='test_runner_node',
            output='screen',
        ),
    ])
