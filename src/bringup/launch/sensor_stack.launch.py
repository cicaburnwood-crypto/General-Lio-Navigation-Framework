"""Selected MID360 + self filter + FAST-LIO sensor stack.

Usage:
  ros2 launch bringup sensor_stack.launch.py lidar:=front_118
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _flat_numbers(name, value):
    if value is None:
        return []
    if not isinstance(value, list):
        raise RuntimeError(f'{name} must be a list')
    numbers = []
    for item in value:
        if isinstance(item, list):
            numbers.extend(float(x) for x in item)
        else:
            numbers.append(float(item))
    return numbers


def _load_profiles(path):
    if not os.path.exists(path):
        raise RuntimeError(f'lidar profile file not found: {path}')
    with open(path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or 'profiles' not in data:
        raise RuntimeError(f'invalid lidar profile file: {path}')
    return data


def _resolve_config_path(profile_file, driver_config):
    if os.path.isabs(driver_config):
        return driver_config
    return os.path.join(os.path.dirname(profile_file), driver_config)


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    fast_lio_share = get_package_share_directory('fast_lio')

    profile_file = LaunchConfiguration('profile_file').perform(context)
    lidar_name = LaunchConfiguration('lidar').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    fast_lio_config_file = LaunchConfiguration('fast_lio_config_file').perform(context)

    profiles_data = _load_profiles(profile_file)
    default_lidar = profiles_data.get('default_lidar', 'front_118')
    selected_lidar = lidar_name or default_lidar
    profiles = profiles_data['profiles']
    if selected_lidar not in profiles:
        available = ', '.join(sorted(profiles.keys()))
        raise RuntimeError(
            f"unknown lidar profile '{selected_lidar}', available profiles: {available}")

    profile = profiles[selected_lidar]
    base_frame = profiles_data.get('base_frame', 'base_link')
    lidar_frame = profile['lidar_frame']
    input_livox_topic = profiles_data.get('input_livox_topic', '/livox/lidar')
    filtered_livox_topic = profiles_data.get('filtered_livox_topic', '/livox/lidar_filtered')
    driver_config_path = _resolve_config_path(profile_file, profile['driver_config'])
    if not os.path.exists(driver_config_path):
        raise RuntimeError(f'Livox driver config not found: {driver_config_path}')

    base_to_lidar_xyz = _flat_numbers(
        f'{selected_lidar}.base_to_lidar_xyz',
        profile.get('base_to_lidar_xyz', [0.0, 0.0, 0.0]))
    base_to_lidar_rpy = _flat_numbers(
        f'{selected_lidar}.base_to_lidar_rpy',
        profile.get('base_to_lidar_rpy', [0.0, 0.0, 0.0]))
    if len(base_to_lidar_xyz) != 3 or len(base_to_lidar_rpy) != 3:
        raise RuntimeError('base_to_lidar_xyz and base_to_lidar_rpy must have 3 values')

    common_boxes = _flat_numbers(
        'self_filter.boxes_base_link',
        profiles_data.get('self_filter', {}).get('boxes_base_link', []))
    profile_boxes = _flat_numbers(
        f'{selected_lidar}.boxes_base_link',
        profile.get('boxes_base_link', []))
    boxes_base_link = common_boxes + profile_boxes
    if len(boxes_base_link) % 6 != 0:
        raise RuntimeError('self-filter boxes must contain 6 values per box')

    livox_driver = Node(
        package='livox_ros_driver2',
        executable='livox_ros_driver2_node',
        name='livox_lidar_publisher',
        output='screen',
        parameters=[{
            'xfer_format': 1,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': lidar_frame,
            'lvx_file_path': '/home/livox/livox_test.lvx',
            'user_config_path': driver_config_path,
            'cmdline_input_bd_code': 'livox0000000001',
        }],
    )

    static_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'{selected_lidar}_static_tf',
        output='screen',
        arguments=[
            '--x', str(base_to_lidar_xyz[0]),
            '--y', str(base_to_lidar_xyz[1]),
            '--z', str(base_to_lidar_xyz[2]),
            '--roll', str(base_to_lidar_rpy[0]),
            '--pitch', str(base_to_lidar_rpy[1]),
            '--yaw', str(base_to_lidar_rpy[2]),
            '--frame-id', base_frame,
            '--child-frame-id', lidar_frame,
        ],
    )

    self_filter = Node(
        package='livox_self_filter',
        executable='livox_self_filter_node',
        name='livox_self_filter',
        output='screen',
        parameters=[{
            'input_topic': input_livox_topic,
            'output_topic': filtered_livox_topic,
            'base_to_lidar_xyz': base_to_lidar_xyz,
            'base_to_lidar_rpy': base_to_lidar_rpy,
            'boxes_base_link': boxes_base_link,
        }],
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'config_path': os.path.join(fast_lio_share, 'config'),
            'config_file': fast_lio_config_file,
        }.items(),
    )

    return [
        LogInfo(
            msg=(
                f"Starting sensor stack lidar={selected_lidar} "
                f"ip={profile.get('ip', 'unknown')} frame={lidar_frame}"
            )
        ),
        livox_driver,
        static_lidar_tf,
        self_filter,
        fast_lio,
    ]


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    default_profile_file = os.path.join(bringup_share, 'config', 'lidar_profiles.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar',
            default_value='front_118',
            description='LiDAR profile: front_118, right_rear_115, left_front_119',
        ),
        DeclareLaunchArgument(
            'profile_file',
            default_value=default_profile_file,
            description='YAML file containing LiDAR profiles',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'fast_lio_config_file',
            default_value='mid360_filtered.yaml',
            description='FAST-LIO config file under fast_lio/config',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
