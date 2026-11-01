"""Normalize external odom/cloud inputs for AMCL + Nav2.

Default parameters target filtered raw Livox data:
  /Odometry                  -> /odom + TF odom->base_link
  /livox/lidar_filtered CustomMsg -> /sensors/cloud + /scan
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_USE_CONFIG = '__use_config_file__'
_STRING_PARAMS = {
    'input_odom_topic',
    'output_odom_topic',
    'input_cloud_topic',
    'output_cloud_topic',
    'output_scan_topic',
    'cloud_input_type',
    'output_cloud_frame',
    'odom_frame',
    'base_frame',
    'cloud_frame',
    'scan_target_frame',
}
_BOOL_PARAMS = {'publish_tf', 'publish_lidar_tf', 'use_inf'}
_FLOAT_PARAMS = {
    'min_height',
    'max_height',
    'angle_min',
    'angle_max',
    'angle_increment',
    'scan_time',
    'range_min',
    'range_max',
    'inf_epsilon',
    'transform_tolerance',
}


def _as_bool(value):
    return value.lower() in ('true', '1', 'yes', 'on')


def _launch_setup(context, *args, **kwargs):
    del args, kwargs

    overrides = {
        'use_sim_time': _as_bool(LaunchConfiguration('use_sim_time').perform(context)),
    }

    for name in sorted(_STRING_PARAMS):
        value = LaunchConfiguration(name).perform(context)
        if value != _USE_CONFIG:
            overrides[name] = value

    for name in sorted(_BOOL_PARAMS):
        value = LaunchConfiguration(name).perform(context)
        if value != _USE_CONFIG:
            overrides[name] = _as_bool(value)

    for name in sorted(_FLOAT_PARAMS):
        value = LaunchConfiguration(name).perform(context)
        if value != _USE_CONFIG:
            overrides[name] = float(value)

    adapter = Node(
        package='nav_interface_adapter',
        executable='nav_interface_adapter_node',
        name='nav_interface_adapter',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            overrides,
        ],
    )

    return [adapter]


def generate_launch_description():
    pkg_share = get_package_share_directory('nav_interface_adapter')
    default_config = os.path.join(pkg_share, 'config', 'adapter_raw_livox.yaml')

    declare_config = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='nav_interface_adapter parameter file',
    )
    declare_sim = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_params = [
        *[
            DeclareLaunchArgument(name, default_value=_USE_CONFIG)
            for name in sorted(_STRING_PARAMS | _BOOL_PARAMS | _FLOAT_PARAMS)
        ],
    ]
    return LaunchDescription([
        declare_config,
        declare_sim,
        *declare_params,
        OpaqueFunction(function=_launch_setup),
    ])
