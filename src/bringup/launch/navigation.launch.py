"""对外稳定的导航入口（README §5.2）。

当前等价于 full_stack.launch.py，独立保留是为了：
  - 命令行接口稳定（`ros2 launch bringup navigation.launch.py map:=...`）
  - 未来可能在这一层加 RViz、diagnostics 监控、健康看板等"运行时辅助"
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

    declare_map = DeclareLaunchArgument('map', description='地图 yaml 路径')
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

    full_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bringup'), 'launch', 'full_stack.launch.py'
            ])
        ]),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'lidar': lidar,
            'adapter_config': adapter_config,
            'amcl_config': amcl_config,
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
        full_stack,
    ])
