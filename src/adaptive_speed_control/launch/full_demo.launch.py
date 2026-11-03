import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory('adaptive_speed_control')
    pkg_sim = get_package_share_directory('adaptive_speed_simulation')

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='1',
        description='Scenario preset: 1 (open), 2 (moderate), 3 (cluttered), 4 (barrier), 5 (warehouse), 6 (hospital)'
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
    dashboard_arg = DeclareLaunchArgument(
        'dashboard', default_value='false',
        description='Launch live speed dashboard GUI'
    )
    adaptive_enabled_arg = DeclareLaunchArgument(
        'adaptive_enabled', default_value='true',
        description='Enable adaptive speed policy (false for fixed baseline)'
    )
    log_metrics_arg = DeclareLaunchArgument(
        'log_metrics', default_value='false',
        description='Log trajectory metrics to CSV'
    )
    csv_file_arg = DeclareLaunchArgument(
        'csv_filename', default_value='results/demo_metrics.csv',
        description='Path for metrics CSV output'
    )
    fault_profile_arg = DeclareLaunchArgument('fault_profile', default_value='none', description='LiDAR fault profile')
    bag_arg = DeclareLaunchArgument('bag', default_value='false', description='Record evaluation topics with ros2 bag')

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

    # 2. Dynamic components based on scenario preset
    def setup_scenario_components(context, *args, **kwargs):
        scenario = LaunchConfiguration('scenario').perform(context).strip().lower()

        actions = []

        # Determine navigator config and dynamic obstacle setup
        if scenario in ('5', 'warehouse'):
            nav_config = os.path.join(pkg_control, 'config', 'navigation_scenario_5.yaml')
            dyn_obs_config = os.path.join(pkg_control, 'config', 'dynamic_obstacles_scenario_5.yaml')
            actions.append(Node(
                package='adaptive_speed_control',
                executable='moving_obstacle_node',
                name='moving_obstacle_node',
                output='screen',
                parameters=[dyn_obs_config]
            ))
        elif scenario in ('6', 'hospital'):
            nav_config = os.path.join(pkg_control, 'config', 'navigation_scenario_6.yaml')
            dyn_obs_config = os.path.join(pkg_control, 'config', 'dynamic_obstacles_scenario_6.yaml')
            actions.append(Node(
                package='adaptive_speed_control',
                executable='moving_obstacle_node',
                name='moving_obstacle_node',
                output='screen',
                parameters=[dyn_obs_config]
            ))
        else:
            nav_config = os.path.join(pkg_control, 'config', 'navigator_params.yaml')

        demo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_control, 'launch', 'start_to_end_demonstration.launch.py')
            ),
            launch_arguments={
                'navigator_params': nav_config,
                'auto_start': LaunchConfiguration('autostart'),
                'adaptive_enabled': LaunchConfiguration('adaptive_enabled'),
                'fault_profile': LaunchConfiguration('fault_profile'),
            }.items()
        )
        actions.append(demo_launch)

        return actions

    # 3. Optional Speed Dashboard Node
    dashboard_node = Node(
        package='adaptive_speed_control',
        executable='speed_dashboard_node',
        name='speed_dashboard_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard'))
    )

    # 4. Optional Metrics Logger
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

    # ros2 bag respects the simulated clock when replayed with --clock.  This
    # deliberately records the raw and degraded scans for fault diagnosis.
    bag_recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', PythonExpression(["'bags/scenario_' + '", LaunchConfiguration('scenario'), "'"]),
             '/scan', '/scan_raw', '/odom', '/cmd_vel_in', '/cmd_vel', '/robot_status',
             '/obstacle_info', '/diagnostics', '/tf', '/clock', '/contacts'],
        condition=IfCondition(LaunchConfiguration('bag')), output='screen'
    )

    return LaunchDescription([
        scenario_arg,
        headless_arg,
        rviz_arg,
        autostart_arg,
        dashboard_arg,
        adaptive_enabled_arg,
        log_metrics_arg,
        csv_file_arg,
        fault_profile_arg,
        bag_arg,
        sim_launch,
        OpaqueFunction(function=setup_scenario_components),
        dashboard_node,
        logger_node,
        bag_recorder,
    ])
