from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'chess_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'templates'), glob('templates/*.html')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ian',
    maintainer_email='ian@example.com',
    description='Chess OS — web UI/control surface for the Smart Chess Board',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'chess_ui = chess_ui.app:main',
        ],
    },
)
