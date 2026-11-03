import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('adaptive_speed_control')
    controller_config = os.path.join(pkg_share, 'config', 'adaptive_speed_params.yaml')
    navigator_config = os.path.join(pkg_share, 'config', 'navigator_params.yaml')

    controller_params_arg = DeclareLaunchArgument(
        'controller_params', default_value=controller_config,
        description='Path to controller parameter YAML file'
    )
    navigator_params_arg = DeclareLaunchArgument(
        'navigator_params', default_value=navigator_config,
        description='Path to navigator parameter YAML file'
    )
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='true',
        description='Automatically begin autonomous corridor navigation'
    )

    # 1. Sole actuator arbiter publishing to /cmd_vel
    controller_node = Node(
        package='adaptive_speed_control',
        executable='adaptive_speed_controller_node',
        name='adaptive_speed_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('controller_params')]
    )

    # 2. Autonomous corridor navigator publishing to /cmd_vel_in
    navigator_node = Node(
        package='adaptive_speed_control',
        executable='waypoint_navigator_node',
        name='waypoint_navigator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('navigator_params'),
            {'auto_start': LaunchConfiguration('auto_start')}
        ]
    )

    # 3. Odometry & HUD monitor publishing to /robot_status
    monitor_node = Node(
        package='adaptive_speed_control',
        executable='odometry_monitor_node',
        name='odometry_monitor_node',
        output='screen'
    )

    return LaunchDescription([
        controller_params_arg,
        navigator_params_arg,
        auto_start_arg,
        controller_node,
        navigator_node,
        monitor_node,
    ])
