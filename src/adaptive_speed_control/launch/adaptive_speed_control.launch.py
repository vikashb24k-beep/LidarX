import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('adaptive_speed_control')
    default_config = os.path.join(pkg_share, 'config', 'adaptive_speed_params.yaml')

    config_arg = DeclareLaunchArgument(
        'params_file', default_value=default_config,
        description='Path to ROS 2 parameters YAML file for adaptive speed controller'
    )
    adaptive_enabled_arg = DeclareLaunchArgument(
        'adaptive_enabled', default_value='true',
        description='Enable dynamic adaptive speed control (false = fixed baseline)'
    )

    controller_node = Node(
        package='adaptive_speed_control',
        executable='adaptive_speed_controller_node',
        name='adaptive_speed_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'adaptive_enabled': LaunchConfiguration('adaptive_enabled')}
        ]
    )

    return LaunchDescription([
        config_arg,
        adaptive_enabled_arg,
        controller_node,
    ])
