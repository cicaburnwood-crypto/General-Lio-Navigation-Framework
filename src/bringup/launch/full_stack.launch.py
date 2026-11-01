"""导航生产栈：interface adapter + chassis_localization + navigation。

FAST-LIO / 里程计源需在外部单独启动。nav_interface_adapter 会把外部
Odometry / PointCloud2 / Livox CustomMsg 规整成 /odom、odom→base_link、
/sensors/cloud 与 /scan。

不含 mapping 子图——那是 mapping.launch.py 的事。

参数：
  map           AMCL 加载的地图 yaml 路径（必填）
  use_sim_time  是否使用仿真时钟，默认 false
  adapter_config nav_interface_adapter 参数文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar = LaunchConfiguration('lidar')
    adapter_config = LaunchConfiguration('adapter_config')
    amcl_config = LaunchConfiguration('amcl_config')
    enable_table_approach = LaunchConfiguration('enable_table_approach')

    declare_map = DeclareLaunchArgument(
        'map', description='AMCL 用的地图 yaml 文件路径'
    )
    declare_sim = DeclareLaunchArgument(
        'use_sim_time', default_value='false'
    )
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
            'adapter_raw_livox.yaml',
        ]),
        description='nav_interface_adapter 参数文件',
    )
    declare_amcl_config = DeclareLaunchArgument(
        'amcl_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('chassis_localization'),
            'config',
            'amcl.yaml',
        ]),
        description='chassis_localization AMCL 参数文件',
    )
    declare_table_approach = DeclareLaunchArgument(
        'enable_table_approach',
        default_value='false',
        description='Enable meeting-room table goal adaptation',
    )

    adapter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'adapter_raw_livox.launch.py'
            ])
        ]),
        launch_arguments={
            'config_file': adapter_config,
            'use_sim_time': use_sim_time,
            'lidar': lidar,
        }.items(),
    )

    chassis_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('chassis_localization'), 'launch', 'amcl.launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'amcl_config': amcl_config,
        }.items(),
    )

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('navigation'), 'launch', 'nav2.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'enable_table_approach': enable_table_approach,
        }.items(),
    )

    return LaunchDescription([
        declare_map,
        declare_sim,
        declare_lidar,
        declare_adapter_config,
        declare_amcl_config,
        declare_table_approach,
        adapter,
        chassis_localization,
        navigation,
    ])
