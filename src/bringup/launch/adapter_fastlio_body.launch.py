"""Adapter for FAST-LIO body-frame data using the selected LiDAR profile."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _as_bool(value):
    return value.lower() in ('true', '1', 'yes', 'on')


def _load_profile(profile_file, lidar_name):
    if not os.path.exists(profile_file):
        raise RuntimeError(f'lidar profile file not found: {profile_file}')
    with open(profile_file, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    profiles = data.get('profiles', {}) if isinstance(data, dict) else {}
    selected = lidar_name or data.get('default_lidar', 'front_118')
    if selected not in profiles:
        available = ', '.join(sorted(profiles.keys()))
        raise RuntimeError(f"unknown lidar profile '{selected}', available profiles: {available}")
    return selected, profiles[selected]


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    lidar_name = LaunchConfiguration('lidar').perform(context)
    profile_file = LaunchConfiguration('profile_file').perform(context)
    config_file = LaunchConfiguration('config_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    cloud_frame = LaunchConfiguration('cloud_frame').perform(context)

    selected_lidar, profile = _load_profile(profile_file, lidar_name)
    base_to_lidar_xyz = _flat_numbers(
        f'{selected_lidar}.base_to_lidar_xyz',
        profile.get('base_to_lidar_xyz', [0.0, 0.0, 0.0]))
    base_to_lidar_rpy = _flat_numbers(
        f'{selected_lidar}.base_to_lidar_rpy',
        profile.get('base_to_lidar_rpy', [0.0, 0.0, 0.0]))
    if len(base_to_lidar_xyz) != 3 or len(base_to_lidar_rpy) != 3:
        raise RuntimeError('base_to_lidar_xyz and base_to_lidar_rpy must have 3 values')

    adapter = Node(
        package='nav_interface_adapter',
        executable='nav_interface_adapter_node',
        name='nav_interface_adapter',
        output='screen',
        parameters=[
            config_file,
            {
                'use_sim_time': _as_bool(use_sim_time),
                'cloud_frame': cloud_frame,
                'base_to_lidar_xyz': base_to_lidar_xyz,
                'base_to_lidar_rpy': base_to_lidar_rpy,
            },
        ],
    )

    return [adapter]


def generate_launch_description():
    bringup_share = get_package_share_directory('bringup')
    adapter_share = get_package_share_directory('nav_interface_adapter')

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='front_118'),
        DeclareLaunchArgument(
            'profile_file',
            default_value=os.path.join(bringup_share, 'config', 'lidar_profiles.yaml'),
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=os.path.join(adapter_share, 'config', 'adapter_fastlio_body.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('cloud_frame', default_value='fastlio_body'),
        OpaqueFunction(function=_launch_setup),
    ])
