from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chess_logic'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian',
    maintainer_email='ian@example.com',
    description='Logic for Smart Chess Board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chess_engine_node = chess_logic.nodes.chess_engine_node:main',
            'game_manager_node = chess_logic.nodes.game_manager_node:main',
        ],
    },
)
