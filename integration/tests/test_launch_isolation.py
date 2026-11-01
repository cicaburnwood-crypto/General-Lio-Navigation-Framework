"""Expand the real AL launch graph without starting any ROS process/hardware."""
import importlib.util
import os
from pathlib import Path

import yaml
from launch import LaunchContext
from launch.actions import ExecuteProcess
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node


def expand_stack(path):
    spec = importlib.util.spec_from_file_location('al_door_stack', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.ROOT = Path(os.environ.get('AL_TEST_ROOT', '/home/hkclr/AL'))
    context = LaunchContext()
    nodes = []

    def visit(entity):
        condition = getattr(entity, 'condition', None)
        if condition is not None and not condition.evaluate(context):
            return
        if isinstance(entity, Node):
            # Only substitutions; deliberately NEVER call Node.execute/visit.
            entity._perform_substitutions(context)
            params, files = {}, []
            for value, is_file in entity._Node__expanded_parameter_arguments or []:
                if not is_file:
                    continue
                files.append(value)
                # Launch serializes numeric lists using !!python/tuple.
                doc = yaml.load(Path(value).read_text(), Loader=yaml.FullLoader)
                for block in (doc or {}).values():
                    if isinstance(block, dict):
                        params.update(block.get('ros__parameters', {}))
            nodes.append(dict(executable=perform_substitutions(
                                  context, normalize_to_list_of_substitutions(entity.node_executable)),
                              params=params, files=files))
            return
        if isinstance(entity, ExecuteProcess):
            raise AssertionError('Unexpected process action; never execute in this test')
        for child in entity.visit(context) or []:
            visit(child)

    visit(module.generate_launch_description())
    return nodes, context


def stack():
    path = os.environ.get('AL_TEST_LAUNCH', str(Path(__file__).parents[1]/'door_stack.launch.py'))
    return expand_stack(path)


def test_adapter_uses_bl_body_cloud_contract():
    nodes, context = stack()
    adapter = next(n for n in nodes if n['executable']=='nav_interface_adapter_node')
    p = adapter['params']
    assert p['input_cloud_topic']=='/cloud_registered_body'
    assert p['cloud_input_type']=='pointcloud2'
    assert p['output_cloud_frame']=='base_link'
    assert p['input_odom_topic']=='/Odometry'
    assert p['output_odom_topic']=='/odom'
    assert p['cloud_frame']=='fastlio_body'
    assert list(p['base_to_lidar_xyz'])==[.3, 0., .1]
    assert any(f.endswith('/adapter_fastlio_body.yaml') for f in adapter['files'])
    assert 'config_file' not in context.launch_configurations


def test_native_matching_and_nav2_are_unchanged():
    nodes, _ = stack()
    localizer = next(n for n in nodes if n['executable']=='chassis_3d_localizer_node')
    assert localizer['params']['update_hz']==2.
    assert localizer['params']['pcd_map_path'].endswith('/hkclr_office/3D/map.pcd')
    assert any(f.endswith('/vgicp_localizer.yaml') for f in localizer['files'])
    assert sum(n['executable']=='door_patrol_manager' for n in nodes)==1
    assert sum(n['executable']=='controller_server' for n in nodes)==1
    assert not any('agv2' in n['executable'] for n in nodes)


def test_only_unused_map_output_differs_from_bl_fastlio():
    nodes, _ = stack()
    fastlio = next(n for n in nodes if n['executable']=='fastlio_mapping')
    params = fastlio['params'].copy()
    params.pop('use_sim_time')
    baseline = yaml.safe_load(Path(
        '/home/hkclr/chassis_nav_ws/src/FAST_LIO/config/mid360_filtered.yaml'
    ).read_text())['/**']['ros__parameters']
    assert params['publish']['map_en'] is False
    params['publish']['map_en'] = True
    assert params == baseline
