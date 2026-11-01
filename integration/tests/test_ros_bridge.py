"""Instantiate the real GUI ROS adapter; no vehicle or velocity publisher."""
import gc
import math
import os
from pathlib import Path
import sys
import threading

from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
import yaml

from patrol_manager.door_patrol_node import DoorPatrolManager
from test_gate import config
from test_ros_workflow import Fixtures, until

integration = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(integration if (integration/'ros_bridge.py').exists() else Path('/integration')))
from ros_bridge import RosBridge


def test_real_gui_bridge_start_status_cancel_restart_and_open(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID') == '192'
    mission = tmp_path/'mission.yaml'
    mission.write_text(yaml.safe_dump(config()))
    bridge = RosBridge('/camera/color/image_raw')
    manager = fixture = executor = thread = None
    try:
        gc.collect()
        assert bridge.executor is bridge.spin_executor
        assert bridge.thread.is_alive()
        assert set(bridge.patrol_clients) == {'/patrol/start', '/patrol/cancel'}
        # Preserve the inherited introspection property, rather than shadow it.
        assert all(client in list(bridge.clients) for client in bridge.patrol_clients.values())
        manager = DoorPatrolManager(parameter_overrides=[Parameter('mission_file', value=str(mission))])
        fixture = Fixtures()
        fixture.speed = 0.; fixture.yaw = math.pi/2
        fixture.hold = True; fixture.opening = 100.
        executor = MultiThreadedExecutor(num_threads=5)
        executor.add_node(manager); executor.add_node(fixture)
        thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
        until(lambda: bridge.snapshot()['nav_ready'] and bridge.snapshot()['door_ready'])
        until(lambda: bridge.snapshot().get('patrol_status', {}).get('state') == 'IDLE')
        assert bridge.trigger('/patrol/start')['success']
        until(lambda: bridge.snapshot().get('patrol_status', {}).get('state') == 'NAVIGATING')
        until(lambda: len(fixture.goals) == 1)
        assert bridge.trigger('/patrol/cancel')['success']
        until(lambda: bridge.snapshot().get('patrol_status', {}).get('state') == 'CANCELED')
        assert fixture.cancel_seen and len(fixture.goals) == 1 and not fixture.calls
        fixture.hold = False
        assert bridge.trigger('/patrol/start')['success']
        until(lambda: bridge.snapshot().get('patrol_status', {}).get('state') == 'SUCCEEDED')
        assert len(fixture.goals) == 3
        assert fixture.goals[-1].pose.pose.position.y == 4.
        assert bridge.snapshot()['door_ready']
        publisher = fixture.create_publisher(Image, '/camera/color/image_raw', qos_profile_sensor_data)
        until(lambda: publisher.get_subscription_count() >= 1)
        message = Image(height=2, width=3, encoding='rgb8', step=9, data=bytes([127]*18))
        publisher.publish(message)
        until(lambda: bridge.jpeg() is not None)
        assert bridge.jpeg().startswith(b'\xff\xd8')
        assert bridge.snapshot()['rgb_age_s'] < 2
    finally:
        if fixture is not None:
            fixture.hold = False
        if executor is not None:
            executor.shutdown()
        if thread is not None:
            thread.join(timeout=5)
        if manager is not None:
            manager.destroy_node()
        if fixture is not None:
            fixture.destroy_node()
        bridge.close()
    assert not bridge.thread.is_alive()
