"""Independent Nav2 door workflow. No hardware velocity driver is started here."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration as C
from launch_ros.actions import Node

ROOT = Path(__file__).resolve().parents[1]


def include(package, file, **args):
    # BL launches these components in separate processes. Preserve that launch
    # argument isolation here: FAST-LIO's config_file must not become the
    # adapter's config_file (which otherwise silently falls back to raw Livox).
    return GroupAction(scoped=True, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(share(package))/'launch'/file)),
            launch_arguments=args.items()),
    ])


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(ROOT/'src/bringup/maps/hkclr_office/2D/hkclr_office_2d.yaml')),
        DeclareLaunchArgument('pcd_map', default_value=str(ROOT/'src/bringup/maps/hkclr_office/3D/map.pcd')),
        DeclareLaunchArgument('lidar', default_value='front_118'),
        DeclareLaunchArgument('profile_file', default_value=str(ROOT/'src/bringup/config/lidar_profiles.yaml')),
        DeclareLaunchArgument('mission', default_value=str(ROOT/'src/patrol_manager/config/door_patrol.yaml')),
        DeclareLaunchArgument('image_topic', default_value='/camera/d435i/color/image_raw'),
        DeclareLaunchArgument('provider', default_value='cpu'),
        include('bringup','sensor_stack.launch.py', lidar=C('lidar'), profile_file=C('profile_file'),
                fast_lio_config_file=str(ROOT/'integration/fastlio_navigation.yaml')),
        include('bringup','adapter_fastlio_body.launch.py', lidar=C('lidar'), profile_file=C('profile_file'),
                config_file=str(ROOT/'src/nav_interface_adapter/config/adapter_fastlio_body.yaml')),
        include('chassis_3d_localization','localizer.launch.py', pcd_map_path=C('pcd_map'),
                localizer_config=str(ROOT/'src/chassis_3d_localization/config/vgicp_localizer.yaml')),
        include('bringup','map_server.launch.py', map=C('map')),
        # Preserve Nav2's native cmd_vel_nav -> smoother -> cmd_vel remaps.
        # An outer SetRemap, even /cmd_vel -> /cmd_vel, overrides those rules
        # and can connect the smoother to its own output.
        # Manager replaces only the table bridge as the sequential goal owner.
        include('navigation','nav2.launch.py', enable_task_bridge='false', enable_table_approach='false',
                params_file=str(ROOT/'src/navigation/config/nav2_mppi.yaml')),
        include('door_opening_server','door_opening.launch.py', image_topic=C('image_topic'), provider=C('provider')),
        Node(package='patrol_manager', executable='door_patrol_manager', name='patrol_manager',
             parameters=[{'mission_file':C('mission')}], output='screen'),
    ])
