"""Nav2 only, using FAST-LIO official frames and topics.

Start Livox driver + FAST-LIO separately first. This launch does not start
AMCL, map_server, or pointcloud_to_laserscan.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('navigation'),
            'config',
            'nav2_fastlio_odom.yaml',
        ]),
        description='Nav2 params for FAST-LIO camera_init/body odom navigation',
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('navigation'), 'launch', 'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'enable_task_bridge': 'false',
        }.items(),
    )

    return LaunchDescription([
        declare_sim,
        declare_params,
        navigation,
    ])
