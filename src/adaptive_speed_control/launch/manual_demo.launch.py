import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_control = get_package_share_directory('adaptive_speed_control')
    pkg_sim = get_package_share_directory('adaptive_speed_simulation')

    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='5',
        description='Scenario preset: 1 (open), 2 (moderate), 3 (cluttered), 4 (barrier), 5 (warehouse), 6 (hospital)'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 display'
    )
    adaptive_arg = DeclareLaunchArgument(
        'adaptive', default_value='true',
        description='Enable adaptive speed policy'
    )
    record_arg = DeclareLaunchArgument(
        'record', default_value='false',
        description='Log manual driving session metrics to CSV'
    )
    challenge_arg = DeclareLaunchArgument(
        'challenge', default_value='false',
        description='Enable manual drive challenge scoring'
    )
    csv_file_arg = DeclareLaunchArgument(
        'csv_filename', default_value='results/manual_metrics.csv',
        description='Path for metrics CSV output'
    )
    fault_profile_arg = DeclareLaunchArgument('fault_profile', default_value='none', description='LiDAR fault profile')
    dashboard_arg = DeclareLaunchArgument(
        'dashboard', default_value='true',
        description='Launch the live graphical speed dashboard when a display is available'
    )

    # 1. Gazebo Simulation & Robot Spawning
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

    def setup_manual_demo(context, *args, **kwargs):
        scenario = LaunchConfiguration('scenario').perform(context).strip().lower()
        actions = []

        # Goal coordinates for monitor & challenge node based on scenario
        goal_x, goal_y = 9.0, 0.0
        if scenario in ('5', 'warehouse'):
            goal_x, goal_y = 7.5, 4.5
            dyn_obs_config = os.path.join(pkg_control, 'config', 'dynamic_obstacles_scenario_5.yaml')
            actions.append(Node(
                package='adaptive_speed_control',
                executable='moving_obstacle_node',
                name='moving_obstacle_node',
                output='screen',
                parameters=[dyn_obs_config]
            ))
        elif scenario in ('6', 'hospital'):
            goal_x, goal_y = 6.0, 4.0
            dyn_obs_config = os.path.join(pkg_control, 'config', 'dynamic_obstacles_scenario_6.yaml')
            actions.append(Node(
                package='adaptive_speed_control',
                executable='moving_obstacle_node',
                name='moving_obstacle_node',
                output='screen',
                parameters=[dyn_obs_config]
            ))

        # Adaptive Speed Controller (Sole publisher to /cmd_vel)
        actions.append(Node(
            package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
            parameters=[os.path.join(pkg_control, 'config', 'twist_mux.yaml')],
            remappings=[('/cmd_vel_out', '/cmd_vel_in')]
        ))
        actions.append(Node(
            package='adaptive_speed_control', executable='scan_fault_injector_node',
            name='scan_fault_injector_node', output='screen',
            parameters=[{'fault_profile': LaunchConfiguration('fault_profile'), 'use_sim_time': True}]
        ))
        actions.append(Node(
            package='adaptive_speed_control',
            executable='adaptive_speed_controller_node',
            name='adaptive_speed_controller_node',
            output='screen',
            parameters=[{
                'adaptive_enabled': LaunchConfiguration('adaptive'),
                'threshold': 0.45,
                'slowdown_distance': 2.00,
                'density_radius': 1.30,
                'density_influence': 0.60,
                'max_linear_speed': 0.40,
                'max_acceleration': 0.60,
                'enable_corridor_stop': False,
                'use_sim_time': True,
            }]
        ))
        actions.append(Node(
            package='adaptive_speed_control', executable='collision_monitor_node',
            name='collision_monitor_node', output='screen', parameters=[{'use_sim_time': True}]
        ))

        # Odometry Monitor (HUD & RViz markers)
        actions.append(Node(
            package='adaptive_speed_control',
            executable='odometry_monitor_node',
            name='odometry_monitor_node',
            output='screen',
            parameters=[{
                'goal_x': goal_x,
                'goal_y': goal_y,
                'speed_limit': 0.40,
                'log_rate': 1.5,
                'use_sim_time': True,
            }]
        ))

        # Gazebo GUI Key Bridge (allows driving directly with Gazebo window focused)
        actions.append(Node(
            package='adaptive_speed_control',
            executable='gazebo_key_bridge_node',
            name='gazebo_key_bridge_node',
            output='screen',
        ))

        actions.append(Node(
            package='adaptive_speed_control',
            executable='speed_dashboard_node',
            name='speed_dashboard_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('dashboard'))
        ))

        # Optional Challenge Scoring Node
        actions.append(Node(
            package='adaptive_speed_control',
            executable='challenge_scoring_node',
            name='challenge_scoring_node',
            output='screen',
            parameters=[{
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': 0.8,
            }],
            condition=IfCondition(LaunchConfiguration('challenge'))
        ))

        # Optional Metrics Logger
        actions.append(Node(
            package='adaptive_speed_control',
            executable='metrics_logger_node',
            name='metrics_logger_node',
            output='screen',
            parameters=[{
                'csv_filename': LaunchConfiguration('csv_filename'),
                'log_rate': 5.0,
            }],
            condition=IfCondition(LaunchConfiguration('record'))
        ))

        # Instructions banner
        actions.append(LogInfo(
            msg="\n"
                "================================================================================\n"
                "  [MANUAL DRIVING READY]\n"
                "  To drive the robot with your keyboard, open a SECOND TERMINAL and run:\n"
                "      ros2 run adaptive_speed_control keyboard_teleop_node\n"
                "\n"
                "  Live speed dashboard: enabled by default (set dashboard:=false to disable).\n"
                "================================================================================\n"
        ))

        return actions

    return LaunchDescription([
        scenario_arg,
        headless_arg,
        rviz_arg,
        adaptive_arg,
        record_arg,
        challenge_arg,
        csv_file_arg,
        fault_profile_arg,
        dashboard_arg,
        sim_launch,
        OpaqueFunction(function=setup_manual_demo),
    ])
