from setuptools import setup
import os
from glob import glob

package_name = 'rt1_assignment2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='endri',
    maintainer_email='endrigjinaj49@gmail.com',
    description='ROS 2 Adaptive Speed Controller and Start-to-End Navigation in Cluttered Environments.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_node = rt1_assignment2.ui_node:main',
            'distance_node = rt1_assignment2.distance_node:main',
            'waypoint_navigator = rt1_assignment2.waypoint_navigator:main',
            'clear_barrier = rt1_assignment2.clear_barrier:main',
            'reset_barrier = rt1_assignment2.clear_barrier:reset_main',
        ],
    },
)
