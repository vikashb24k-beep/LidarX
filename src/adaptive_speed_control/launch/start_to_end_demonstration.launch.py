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
    adaptive_enabled_arg = DeclareLaunchArgument(
        'adaptive_enabled', default_value='true',
        description='Flag to enable adaptive speed policy (false for fixed baseline)'
    )
    fault_profile_arg = DeclareLaunchArgument(
        'fault_profile', default_value='none',
        description='LiDAR fault profile applied between /scan_raw and /scan'
    )

    # 1. Sole actuator arbiter publishing to /cmd_vel
    controller_node = Node(
        package='adaptive_speed_control',
        executable='adaptive_speed_controller_node',
        name='adaptive_speed_controller_node',
        output='screen',
        parameters=[
            LaunchConfiguration('controller_params'),
            {'adaptive_enabled': LaunchConfiguration('adaptive_enabled'), 'use_sim_time': True}
        ]
    )

    # 2. Autonomous corridor / waypoint navigator publishing to /cmd_vel_in
    navigator_node = Node(
        package='adaptive_speed_control',
        executable='waypoint_navigator_node',
        name='waypoint_navigator_node',
        output='screen',
        parameters=[
            LaunchConfiguration('navigator_params'),
            {'auto_start': LaunchConfiguration('auto_start'), 'use_sim_time': True}
        ]
    )

    # Keep velocity arbitration upstream of the final safety stage.  Teleop
    # has higher priority than navigation; /cmd_vel_in is twist_mux output.
    mux_node = Node(
        package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
        parameters=[os.path.join(pkg_share, 'config', 'twist_mux.yaml')]
    )

    fault_injector_node = Node(
        package='adaptive_speed_control', executable='scan_fault_injector_node',
        name='scan_fault_injector_node', output='screen',
        parameters=[{'fault_profile': LaunchConfiguration('fault_profile'), 'use_sim_time': True}]
    )

    collision_monitor_node = Node(
        package='adaptive_speed_control', executable='collision_monitor_node',
        name='collision_monitor_node', output='screen', parameters=[{'use_sim_time': True}]
    )

    # 3. Odometry & HUD monitor publishing to /robot_status and RViz markers
    monitor_node = Node(
        package='adaptive_speed_control',
        executable='odometry_monitor_node',
        name='odometry_monitor_node',
        output='screen', parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        controller_params_arg,
        navigator_params_arg,
        auto_start_arg,
        adaptive_enabled_arg,
        fault_profile_arg,
        mux_node,
        fault_injector_node,
        controller_node,
        navigator_node,
        monitor_node,
        collision_monitor_node,
    ])
