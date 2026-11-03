from setuptools import setup
import os
from glob import glob

package_name = 'adaptive_speed_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Endri Gjinaj',
    maintainer_email='endrigjinaj49@gmail.com',
    author='Vikash',
    author_email='vikash@iiitdwd.ac.in',
    description='ROS 2 Adaptive Speed Controller and Autonomous Navigation in Cluttered Environments.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Canonical production nodes
            'adaptive_speed_controller_node = adaptive_speed_control.adaptive_speed_controller_node:main',
            'teleop_ui_node = adaptive_speed_control.teleop_ui_node:main',
            'waypoint_navigator_node = adaptive_speed_control.waypoint_navigator_node:main',
            'odometry_monitor_node = adaptive_speed_control.odometry_monitor_node:main',
            'metrics_logger_node = adaptive_speed_control.metrics_logger_node:main',
            'moving_obstacle_node = adaptive_speed_control.moving_obstacle_node:main',
            'speed_dashboard_node = adaptive_speed_control.speed_dashboard_node:main',
            'keyboard_teleop_node = adaptive_speed_control.keyboard_teleop_node:main',
            'manual_dashboard_node = adaptive_speed_control.manual_dashboard_node:main',
            'gazebo_key_bridge_node = adaptive_speed_control.gazebo_key_bridge_node:main',
            'challenge_scoring_node = adaptive_speed_control.challenge_scoring_node:main',
            'clear_barrier = adaptive_speed_control.clear_barrier:main',
            'reset_barrier = adaptive_speed_control.clear_barrier:reset_main',
            'scan_fault_injector_node = adaptive_speed_control.scan_fault_injector_node:main',
            'collision_monitor_node = adaptive_speed_control.collision_monitor_node:main',
            'lifecycle_manager_node = adaptive_speed_control.lifecycle_manager_node:main',
            # Backward-compatible script aliases
            'distance_node = adaptive_speed_control.distance_node:main',
            'ui_node = adaptive_speed_control.ui_node:main',
            'waypoint_navigator = adaptive_speed_control.waypoint_navigator:main',
        ],
    },
)
