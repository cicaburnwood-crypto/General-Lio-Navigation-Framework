import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('chassis_3d_localization')
    default_config = os.path.join(pkg_share, 'config', 'vgicp_localizer.yaml')
    default_pcd_map = PathJoinSubstitution([
        FindPackageShare('bringup'),
        'maps',
        'inner_office',
        '3D',
        'map.pcd',
    ])

    use_sim_time = LaunchConfiguration('use_sim_time')
    localizer_config = LaunchConfiguration('localizer_config')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    initialpose_topic = LaunchConfiguration('initialpose_topic')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'pcd_map_path',
            default_value=default_pcd_map,
            description='PCD map loaded on localizer startup',
        ),
        DeclareLaunchArgument(
            'initialpose_topic',
            default_value='/initialpose',
            description='PoseWithCovarianceStamped topic from RViz 2D Pose Estimate',
        ),
        DeclareLaunchArgument(
            'localizer_config',
            default_value=default_config,
            description='3D VGICP localizer parameter file',
        ),
        Node(
            package='chassis_3d_localization',
            executable='chassis_3d_localizer_node',
            namespace='localizer',
            name='chassis_3d_localizer_node',
            output='screen',
            parameters=[
                localizer_config,
                {
                    'use_sim_time': use_sim_time,
                    'pcd_map_path': pcd_map_path,
                    'initialpose_topic': initialpose_topic,
                },
            ],
        ),
    ])
