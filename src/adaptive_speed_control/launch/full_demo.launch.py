import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory('adaptive_speed_control')
    pkg_sim = get_package_share_directory('adaptive_speed_simulation')

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='1',
        description='Scenario preset: 1 (open corridor), 2 (moderate), 3 (cluttered), 4 (barrier)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 display'
    )
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically start waypoint navigation'
    )
    log_metrics_arg = DeclareLaunchArgument(
        'log_metrics', default_value='false',
        description='Log trajectory metrics to CSV'
    )
    csv_file_arg = DeclareLaunchArgument(
        'csv_filename', default_value='results/demo_metrics.csv',
        description='Path for metrics CSV output'
    )

    # 1. Simulation and robot spawn
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'scenario': LaunchConfiguration('scenario'),
            'headless': LaunchConfiguration('headless'),
            'rviz': LaunchConfiguration('rviz'),
        }.items()
    )

    # 2. Navigation, controller, and odometry monitor
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_control, 'launch', 'start_to_end_demonstration.launch.py')
        ),
        launch_arguments={
            'auto_start': LaunchConfiguration('autostart'),
        }.items()
    )

    # 3. Optional Metrics Logger
    logger_node = Node(
        package='adaptive_speed_control',
        executable='metrics_logger_node',
        name='metrics_logger_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('log_metrics')),
        parameters=[{
            'csv_filename': LaunchConfiguration('csv_filename'),
        }]
    )

    return LaunchDescription([
        scenario_arg,
        headless_arg,
        rviz_arg,
        autostart_arg,
        log_metrics_arg,
        csv_file_arg,
        sim_launch,
        demo_launch,
        logger_node,
    ])

