from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments for runtime configuration
    threshold_arg = DeclareLaunchArgument(
        'threshold', default_value='0.45',
        description='Minimum safe distance threshold in meters'
    )
    slowdown_distance_arg = DeclareLaunchArgument(
        'slowdown_distance', default_value='1.50',
        description='Distance in meters where adaptive speed reduction starts'
    )
    density_radius_arg = DeclareLaunchArgument(
        'density_radius', default_value='1.20',
        description='Obstacle detection radius for density calculation in meters'
    )
    density_influence_arg = DeclareLaunchArgument(
        'density_influence', default_value='0.50',
        description='Weighting influence of obstacle density in [0.0, 1.0]'
    )
    max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed', default_value='0.40',
        description='Maximum allowed linear speed in m/s'
    )
    max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed', default_value='1.00',
        description='Maximum allowed angular speed in rad/s'
    )
    front_angle_deg_arg = DeclareLaunchArgument(
        'front_angle_deg', default_value='60.0',
        description='Half-sector monitoring angle in degrees (+/-)'
    )

    # Adaptive speed controller node
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
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'front_angle_deg': LaunchConfiguration('front_angle_deg'),
        }]
    )

    return LaunchDescription([
        threshold_arg,
        slowdown_distance_arg,
        density_radius_arg,
        density_influence_arg,
        max_linear_speed_arg,
        max_angular_speed_arg,
        front_angle_deg_arg,
        distance_node,
    ])

