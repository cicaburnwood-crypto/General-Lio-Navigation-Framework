"""3D localization stack: FAST-LIO adapter + VGICP scan-to-map localizer.

Start Livox driver and FAST-LIO separately. This launch consumes FAST-LIO
/Odometry and /cloud_registered_body, normalizes them to /odom and
/sensors/cloud, then publishes map -> odom after /localizer/relocalize.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar = LaunchConfiguration('lidar')
    adapter_config = LaunchConfiguration('adapter_config')
    localizer_config = LaunchConfiguration('localizer_config')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    initialpose_topic = LaunchConfiguration('initialpose_topic')

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='front_118',
        description='LiDAR profile: front_118, right_rear_115, left_front_119',
    )
    declare_adapter_config = DeclareLaunchArgument(
        'adapter_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('nav_interface_adapter'),
            'config',
            'adapter_fastlio_body.yaml',
        ]),
        description='nav_interface_adapter params for FAST-LIO body cloud',
    )
    declare_localizer_config = DeclareLaunchArgument(
        'localizer_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('chassis_3d_localization'),
            'config',
            'vgicp_localizer.yaml',
        ]),
        description='3D VGICP localizer parameter file',
    )
    declare_pcd_map = DeclareLaunchArgument(
        'pcd_map_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('bringup'),
            'maps',
            'inner_office',
            '3D',
            'map.pcd',
        ]),
        description='PCD map loaded by the 3D localizer on startup',
    )
    declare_initialpose_topic = DeclareLaunchArgument(
        'initialpose_topic',
        default_value='/initialpose',
        description='PoseWithCovarianceStamped topic for RViz initial pose',
    )

    adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'adapter_fastlio_body.launch.py',
            ])
        ]),
        launch_arguments={
            'lidar': lidar,
            'config_file': adapter_config,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_3d_localization'),
                'launch',
                'localizer.launch.py',
            ])
        ]),
        launch_arguments={
            'localizer_config': localizer_config,
            'pcd_map_path': pcd_map_path,
            'initialpose_topic': initialpose_topic,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription([
        declare_sim,
        declare_lidar,
        declare_adapter_config,
        declare_localizer_config,
        declare_pcd_map,
        declare_initialpose_topic,
        adapter,
        localizer,
    ])
