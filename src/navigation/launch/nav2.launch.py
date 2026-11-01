"""Nav2 启动封装。

策略：
  - 复用 nav2_bringup 的 navigation_launch.py（标准 lifecycle 管理 + 默认拓扑）
  - 用本包 nav2.yaml 覆盖参数
  - 保留 Nav2 Humble 默认速度链路：
    controller_server → cmd_vel_nav → velocity_smoother → /cmd_vel
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():
    pkg_share = get_package_share_directory('navigation')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    default_params = os.path.join(pkg_share, 'config', 'nav2_mppi.yaml')
    default_table_params = os.path.join(pkg_share, 'config', 'table_approach.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    table_params_file = LaunchConfiguration('table_params_file')
    enable_task_bridge = LaunchConfiguration('enable_task_bridge')
    enable_table_approach = LaunchConfiguration('enable_table_approach')

    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_params = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Nav2 configuration file path'
    )
    declare_table_params = DeclareLaunchArgument(
        'table_params_file', default_value=default_table_params,
        description='Table approach geometry and direct-motion parameters'
    )
    declare_task_bridge = DeclareLaunchArgument(
        'enable_task_bridge',
        default_value='true',
        description='Start the unified /goal_pose and /navigation/target_pose navigation bridge',
    )
    declare_table_approach = DeclareLaunchArgument(
        'enable_table_approach',
        default_value='true',
        description='Enable meeting-room table goal adaptation and direct 20 cm insertion',
    )

    # Keep Nav2's GoalPose subscription off the public /goal_pose topic.  The
    # task bridge below is the single owner of both public goal inputs and
    # calls NavigateToPose only after any table retreat has completed.
    nav2_with_bridge = GroupAction(
        condition=IfCondition(enable_task_bridge),
        actions=[
            SetRemap(src='/goal_pose', dst='/validated_goal_pose'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'true',
                }.items(),
            ),
        ],
    )

    # FAST-LIO-only and other callers may explicitly disable the unified
    # bridge.  In that mode keep Nav2's standard /goal_pose input untouched.
    nav2_without_bridge = IncludeLaunchDescription(
        condition=UnlessCondition(enable_task_bridge),
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    task_navigation_bridge = Node(
        package='nav_goal_regenerator',
        executable='task_navigation_bridge_node',
        name='task_navigation_bridge',
        output='screen',
        condition=IfCondition(enable_task_bridge),
        parameters=[table_params_file, {
            'use_sim_time': use_sim_time,
            'enable_table_approach': enable_table_approach,
        }],
    )

    # Preserve the old arrival signal for direct Nav2 mode. This node is not
    # started when the unified bridge owns the full task lifecycle.
    arrival_monitor = Node(
        package='nav_goal_regenerator',
        executable='arrival_monitor_node',
        name='arrival_monitor',
        output='screen',
        condition=UnlessCondition(enable_task_bridge),
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_sim,
        declare_params,
        declare_table_params,
        declare_task_bridge,
        declare_table_approach,
        task_navigation_bridge,
        arrival_monitor,
        nav2_with_bridge,
        nav2_without_bridge,
    ])
