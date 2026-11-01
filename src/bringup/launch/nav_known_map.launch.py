"""Known-map navigation with either 2D AMCL or 3D VGICP localization."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _validate_localization(context, *args, **kwargs):
    del args, kwargs

    mode = LaunchConfiguration('localization').perform(context)
    if mode not in ('2d', '3d'):
        raise RuntimeError("localization must be '2d' or '3d'")
    return []


def _is_mode(mode):
    return IfCondition(PythonExpression(["'", LaunchConfiguration('localization'), f"' == '{mode}'"]))


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    chassis_localization_share = get_package_share_directory('chassis_localization')
    chassis_3d_localization_share = get_package_share_directory('chassis_3d_localization')
    nav_interface_adapter_share = get_package_share_directory('nav_interface_adapter')
    navigation_share = get_package_share_directory('navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar = LaunchConfiguration('lidar')
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file')
    map_yaml = LaunchConfiguration('map')
    pcd_map_path = LaunchConfiguration('pcd_map_path')
    raw_adapter_config = LaunchConfiguration('raw_adapter_config')
    fastlio_adapter_config = LaunchConfiguration('fastlio_adapter_config')
    amcl_config = LaunchConfiguration('amcl_config')
    localizer_config = LaunchConfiguration('localizer_config')
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    enable_table_approach = LaunchConfiguration('enable_table_approach')

    sensor_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'sensor_stack.launch.py'])
        ]),
        launch_arguments={
            'lidar': lidar,
            'use_sim_time': use_sim_time,
            'fast_lio_config_file': fast_lio_config_file,
        }.items(),
    )

    raw_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'adapter_raw_livox.launch.py',
            ])
        ]),
        condition=_is_mode('2d'),
        launch_arguments={
            'config_file': raw_adapter_config,
            'use_sim_time': use_sim_time,
            'lidar': lidar,
        }.items(),
    )

    amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_localization'),
                'launch',
                'amcl.launch.py',
            ])
        ]),
        condition=_is_mode('2d'),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'amcl_config': amcl_config,
        }.items(),
    )

    fastlio_adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'adapter_fastlio_body.launch.py',
            ])
        ]),
        condition=_is_mode('3d'),
        launch_arguments={
            'lidar': lidar,
            'config_file': fastlio_adapter_config,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    localizer_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_3d_localization'),
                'launch',
                'localizer.launch.py',
            ])
        ]),
        condition=_is_mode('3d'),
        launch_arguments={
            'localizer_config': localizer_config,
            'pcd_map_path': pcd_map_path,
            'initialpose_topic': '/initialpose',
            'use_sim_time': use_sim_time,
        }.items(),
    )

    map_server_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('bringup'), 'launch', 'map_server.launch.py'])
        ]),
        condition=_is_mode('3d'),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('navigation'), 'launch', 'nav2.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'enable_table_approach': enable_table_approach,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'localization',
            default_value='2d',
            description="Known-map localization mode: '2d' for AMCL or '3d' for VGICP",
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'lidar',
            default_value='front_118',
            description='LiDAR profile: front_118, right_rear_115, left_front_119',
        ),
        DeclareLaunchArgument('fast_lio_config_file', default_value='mid360_filtered.yaml'),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                FindPackageShare('bringup'),
                'maps',
                'inner_office',
                '2D',
                'office_test_2d.yaml',
            ]),
            description='2D occupancy map yaml path',
        ),
        DeclareLaunchArgument(
            'pcd_map_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('bringup'),
                'maps',
                'inner_office',
                '3D',
                'map.pcd',
            ]),
            description='PCD map path used by 3D VGICP localization',
        ),
        DeclareLaunchArgument(
            'raw_adapter_config',
            default_value=f'{nav_interface_adapter_share}/config/adapter_raw_livox.yaml',
            description='Adapter params for filtered raw Livox data',
        ),
        DeclareLaunchArgument(
            'fastlio_adapter_config',
            default_value=f'{nav_interface_adapter_share}/config/adapter_fastlio_body.yaml',
            description='Adapter params for FAST-LIO body cloud',
        ),
        DeclareLaunchArgument(
            'amcl_config',
            default_value=f'{chassis_localization_share}/config/amcl.yaml',
        ),
        DeclareLaunchArgument(
            'localizer_config',
            default_value=f'{chassis_3d_localization_share}/config/vgicp_localizer.yaml',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=f'{navigation_share}/config/nav2_mppi.yaml',
        ),
        DeclareLaunchArgument(
            'enable_table_approach',
            default_value='false',
            description='Enable meeting-room table goal adaptation',
        ),
        OpaqueFunction(function=_validate_localization),
        sensor_stack,
        raw_adapter,
        amcl,
        fastlio_adapter,
        localizer_3d,
        map_server_3d,
        navigation,
    ])
