"""建图栈：nav_interface_adapter + slam_toolbox（online_async）。

FAST-LIO / 里程计源需在外部单独启动。nav_interface_adapter 会提供
/odom、odom→base_link 与 /scan。

输出：
  /map  ——后续用 `ros2 run nav2_map_server map_saver_cli -f office_map` 落盘

不启动 AMCL 与 Nav2。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    lidar = LaunchConfiguration('lidar')
    adapter_config = LaunchConfiguration('adapter_config')

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

    # slam_toolbox：online_async 模式。
    # TODO: 把 slam_toolbox 参数提到 bringup/config/slam_toolbox.yaml，
    #       这里直接挂 yaml，避免在 launch 里堆默认值。
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
        }],
    )

    return LaunchDescription([
        declare_sim,
        declare_lidar,
        declare_adapter_config,
        adapter,
        slam_toolbox,
    ])
