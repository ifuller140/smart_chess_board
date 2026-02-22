from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chess_hw_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian',
    maintainer_email='ian@example.com',
    description='Hardware interface for Smart Chess Board',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stepper_driver_node = chess_hw_interface.nodes.stepper_driver_node:main',
            'servo_node = chess_hw_interface.nodes.servo_node:main',
            'limit_switch_node = chess_hw_interface.nodes.limit_switch_node:main',
            'clock_servo_node = chess_hw_interface.nodes.clock_servo_node:main',
            'clock_display_node = chess_hw_interface.nodes.clock_display_node:main',
            'gpio_watchdog_node = chess_hw_interface.nodes.gpio_watchdog_node:main',
            'chess_clock_node = chess_hw_interface.nodes.chess_clock_node:main',
        ],
    },
)
