"""AMCL + map_server + lifecycle_manager（chassis_localization 子集）。

输入：
  /scan         由 nav_interface_adapter 从 /sensors/cloud 投影
  /initialpose  RViz 或外部 service 给定
输出：
  TF map → odom
  /map
"""

import os
import re

from ament_index_python.packages import get_package_share_directory
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _map_root_from_yaml(map_yaml):
    directory = os.path.dirname(os.path.abspath(map_yaml))
    if os.path.basename(directory) in ('2D', '3D'):
        return os.path.dirname(directory)
    return directory


def _sanitize_node_name(name):
    sanitized = re.sub(r'[^A-Za-z0-9_]', '_', name).strip('_')
    return sanitized or 'map_static_tf'


def _number_list(name, value, expected_size):
    if not isinstance(value, list) or len(value) != expected_size:
        raise RuntimeError(f'{name} must be a list with {expected_size} numbers')
    return [float(item) for item in value]


def _load_map_static_tfs(context, *args, **kwargs):
    del args, kwargs

    map_yaml = LaunchConfiguration('map').perform(context)
    static_tfs_path = os.path.join(_map_root_from_yaml(map_yaml), 'static_tfs.yaml')
    if not os.path.exists(static_tfs_path):
        return []

    with open(static_tfs_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    transforms = data.get('transforms', [])
    if not isinstance(transforms, list):
        raise RuntimeError(f'{static_tfs_path}: transforms must be a list')

    nodes = []
    for index, transform in enumerate(transforms):
        if not isinstance(transform, dict):
            raise RuntimeError(f'{static_tfs_path}: transform #{index} must be a map')
        parent_frame = transform.get('parent_frame')
        child_frame = transform.get('child_frame')
        if not parent_frame or not child_frame:
            raise RuntimeError(
                f'{static_tfs_path}: transform #{index} needs parent_frame and child_frame')
        translation = _number_list(
            f'{static_tfs_path}: {child_frame}.translation',
            transform.get('translation'),
            3)
        rotation = _number_list(
            f'{static_tfs_path}: {child_frame}.rotation_xyzw',
            transform.get('rotation_xyzw'),
            4)
        node_name = transform.get('name') or f'{_sanitize_node_name(child_frame)}_static_tf'
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=node_name,
            output='screen',
            arguments=[
                '--x', str(translation[0]),
                '--y', str(translation[1]),
                '--z', str(translation[2]),
                '--qx', str(rotation[0]),
                '--qy', str(rotation[1]),
                '--qz', str(rotation[2]),
                '--qw', str(rotation[3]),
                '--frame-id', parent_frame,
                '--child-frame-id', child_frame,
            ],
        ))
    return nodes


def generate_launch_description():
    pkg_share = get_package_share_directory('chassis_localization')
    default_config = os.path.join(pkg_share, 'config', 'amcl.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    amcl_config = LaunchConfiguration('amcl_config')

    declare_map = DeclareLaunchArgument('map', description='地图 yaml 路径')
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_cfg = DeclareLaunchArgument('amcl_config', default_value=default_config)

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml,
        }],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {'use_sim_time': use_sim_time},
        ],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        declare_map, declare_sim, declare_cfg,
        map_server, amcl, lifecycle_mgr,
        OpaqueFunction(function=_load_map_static_tfs),
    ])
