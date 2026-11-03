import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='my.sdf',
        description='Name of the Gazebo world file to load'
    )
    headless_arg = DeclareLaunchArgument(
        'headless', default_value='false',
        description='Run Gazebo simulation headless without GUI'
    )

    pkg_adaptive_speed_simulation = get_package_share_directory('adaptive_speed_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    def launch_gz_sim(context, *args, **kwargs):
        world_name = LaunchConfiguration('world').perform(context)
        is_headless = LaunchConfiguration('headless').perform(context).lower() in ('true', '1')

        world_path = os.path.join(pkg_adaptive_speed_simulation, 'worlds', world_name)
        if is_headless:
            gz_args_val = f"-s -r --headless-rendering {world_path}"
        else:
            gz_args_val = f"-r -v 1 {world_path}"

        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'),
            ),
            launch_arguments={
                'gz_args': gz_args_val,
                'on_exit_shutdown': 'true'
            }.items()
        )
        return [gazebo_launch]

    return LaunchDescription([
        world_arg,
        headless_arg,
        OpaqueFunction(function=launch_gz_sim)
    ])
