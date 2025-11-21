from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gantry_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'action'), glob('action/*.action')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian',
    maintainer_email='ian@example.com',
    description='Gantry control for Smart Chess Board',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gantry_kinematics_node = gantry_control.nodes.gantry_kinematics_node:main',
            'motion_planner_node = gantry_control.nodes.motion_planner_node:main',
            'homing_node = gantry_control.nodes.homing_node:main',
        ],
    },
)
