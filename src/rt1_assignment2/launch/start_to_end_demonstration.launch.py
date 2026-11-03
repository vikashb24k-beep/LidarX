from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Controller Parameters
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.45',
        description='Safety distance threshold in meters'
    )
    slowdown_distance_arg = DeclareLaunchArgument(
        'slowdown_distance', default_value='2.00',
        description='Distance in meters where adaptive speed reduction starts'
    )
    density_radius_arg = DeclareLaunchArgument(
        'density_radius', default_value='1.30',
        description='Radius for surrounding obstacle density calculation'
    )
    density_influence_arg = DeclareLaunchArgument(
        'density_influence', default_value='0.60',
        description='Weighting influence of obstacle density'
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed', default_value='0.40',
        description='Maximum allowed linear speed in m/s'
    )

    # Navigator Parameters
    start_x_arg = DeclareLaunchArgument('start_x', default_value='-9.0')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='0.0')
    goal_x_arg = DeclareLaunchArgument('goal_x', default_value='9.0')
    goal_y_arg = DeclareLaunchArgument('goal_y', default_value='0.0')
    nominal_speed_arg = DeclareLaunchArgument('nominal_speed', default_value='0.40')
    auto_start_arg = DeclareLaunchArgument('auto_start', default_value='True')

    # Adaptive Speed Controller Node (The Sole Actuator Publisher to /cmd_vel)
    distance_node = Node(
        package='rt1_assignment2',
        executable='distance_node',
        name='distance_node',
        output='screen',
        parameters=[{
            'threshold': LaunchConfiguration('threshold'),
            'slowdown_distance': LaunchConfiguration('slowdown_distance'),
            'density_radius': LaunchConfiguration('density_radius'),
            'density_influence': LaunchConfiguration('density_influence'),
            'max_linear_speed': LaunchConfiguration('max_linear_speed'),
        }]
    )

    # Autonomous Corridor Waypoint Navigator (Publishes to /cmd_vel_in)
    waypoint_navigator = Node(
        package='rt1_assignment2',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'start_x': LaunchConfiguration('start_x'),
            'start_y': LaunchConfiguration('start_y'),
            'goal_x': LaunchConfiguration('goal_x'),
            'goal_y': LaunchConfiguration('goal_y'),
            'nominal_speed': LaunchConfiguration('nominal_speed'),
            'auto_start': LaunchConfiguration('auto_start'),
        }]
    )

    return LaunchDescription([
        threshold_arg,
        slowdown_distance_arg,
        density_radius_arg,
        density_influence_arg,
        max_linear_speed_arg,
        start_x_arg,
        start_y_arg,
        goal_x_arg,
        goal_y_arg,
        nominal_speed_arg,
        auto_start_arg,
        distance_node,
        waypoint_navigator,
    ])

