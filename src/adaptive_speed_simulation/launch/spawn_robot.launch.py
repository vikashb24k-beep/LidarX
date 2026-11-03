import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_simulation = get_package_share_directory('adaptive_speed_simulation')

    gazebo_models_path, _ = os.path.split(pkg_simulation)
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep + gazebo_models_path

    rviz_launch_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Open RViz2 GUI'
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config', default_value='rviz.rviz',
        description='RViz config file'
    )
    world_arg = DeclareLaunchArgument(
        'world', default_value='my.sdf',
        description='Name of the Gazebo world file to load'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo headless without GUI'
    )
    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )
    scenario_arg = DeclareLaunchArgument(
        'scenario', default_value='',
        description='Demonstration scenario preset: 1 (open), 2 (moderate), 3 (cluttered), 4 (barrier)'
    )
    x_arg = DeclareLaunchArgument(
        'x', default_value='-9.0',
        description='x coordinate of spawned robot'
    )
    y_arg = DeclareLaunchArgument(
        'y', default_value='0.0',
        description='y coordinate of spawned robot'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='yaw angle of spawned robot'
    )
    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    urdf_file_path = PathJoinSubstitution([
        pkg_simulation,
        "urdf",
        LaunchConfiguration('model')
    ])

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_simulation, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'headless': LaunchConfiguration('headless'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_simulation, 'rviz', LaunchConfiguration('rviz_config')])],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    def setup_spawn_node(context, *args, **kwargs):
        scenario = LaunchConfiguration('scenario').perform(context).strip().lower()
        x = LaunchConfiguration('x').perform(context)
        y = LaunchConfiguration('y').perform(context)
        yaw = LaunchConfiguration('yaw').perform(context)

        if scenario in ('1', 'open'):
            x, y, yaw = '-9.0', '0.0', '0.0'
        elif scenario in ('2', 'moderate'):
            x, y, yaw = '-4.0', '0.0', '0.0'
        elif scenario in ('3', 'cluttered'):
            x, y, yaw = '0.5', '0.0', '0.0'
        elif scenario in ('4', 'critical', 'barrier'):
            x, y, yaw = '5.0', '0.0', '0.0'

        spawn_urdf_node = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "mogi_bot",
                "-topic", "robot_description",
                "-x", x, "-y", y, "-z", "0.5", "-Y", yaw
            ],
            output="screen",
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ]
        )
        return [spawn_urdf_node]

    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry",
            "/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model",
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan"
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command(['xacro', ' ', urdf_file_path]),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    return LaunchDescription([
        rviz_launch_arg,
        rviz_config_arg,
        world_arg,
        headless_arg,
        model_arg,
        scenario_arg,
        x_arg,
        y_arg,
        yaw_arg,
        sim_time_arg,
        world_launch,
        rviz_node,
        OpaqueFunction(function=setup_spawn_node),
        gz_bridge_node,
        robot_state_publisher_node,
    ])
