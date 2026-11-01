#!/usr/bin/env python3
"""Read-only AL acceptance: counts sensor data and reads lifecycle state."""
import json
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from lifecycle_msgs.srv import GetState
from sensor_msgs.msg import Image, Imu, PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from livox_ros_driver2.msg import CustomMsg


def main():
    rclpy.init()
    node = Node('al_readonly_acceptance')
    samples = {}
    nonzero_cmd = 0
    def capture(topic, msg):
        nonlocal nonzero_cmd
        stamp = getattr(getattr(msg, 'header', None), 'stamp', None)
        age = None if stamp is None else node.get_clock().now().nanoseconds/1e9-stamp.sec-stamp.nanosec/1e9
        size = getattr(msg, 'point_num', getattr(msg, 'width', None))
        samples.setdefault(topic, []).append((time.monotonic(), age, size))
        if isinstance(msg, Twist) and any(abs(v)>1e-6 for v in (msg.linear.x,msg.linear.y,msg.angular.z)):
            nonzero_cmd += 1
    for topic, cls in [('/livox/lidar',CustomMsg),('/livox/imu',Imu),('/sensors/cloud',PointCloud2),
                       ('/odom',Odometry),('/camera/d435i/color/image_raw',Image),('/cmd_vel',Twist)]:
        samples[topic] = []
        node.create_subscription(cls, topic, lambda msg,t=topic:capture(t,msg), qos_profile_sensor_data)
    deadline = time.monotonic()+12
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.1)
    result = {'topics':{},'nonzero_cmd_messages_observed':nonzero_cmd,'lifecycle':{}}
    result['velocity_graph'] = {topic: {
        'publishers': [e.node_name for e in node.get_publishers_info_by_topic(topic)],
        'subscribers': [e.node_name for e in node.get_subscriptions_info_by_topic(topic)],
    } for topic in ('/cmd_vel','/cmd_vel_nav')}
    final = result['velocity_graph']['/cmd_vel']
    raw = result['velocity_graph']['/cmd_vel_nav']
    # Humble's stock navigation_launch.py remaps controller_server through
    # the smoother, while behavior_server retains direct /cmd_vel output.
    # Match that native graph instead of imposing a new deployment policy.
    result['native_velocity_chain_ok'] = ('velocity_smoother' in final['publishers']
        and set(final['publishers']) <= {'velocity_smoother', 'behavior_server'}
        and 'velocity_smoother' not in final['subscribers']
        and 'agv2_control_node' in final['subscribers']
        and 'controller_server' in raw['publishers']
        and 'velocity_smoother' in raw['subscribers'])
    for topic, records in samples.items():
        ages = [r[1] for r in records if r[1] is not None]
        result['topics'][topic] = {'count':len(records),
            'hz':round((len(records)-1)/(records[-1][0]-records[0][0]),2) if len(records)>1 else 0,
            'median_age_s':round(statistics.median(ages),4) if ages else None,
            'last_size':records[-1][2] if records else None}
    for name in ('map_server','controller_server','planner_server','bt_navigator','behavior_server','velocity_smoother'):
        client = node.create_client(GetState,f'/{name}/get_state')
        if client.wait_for_service(timeout_sec=1):
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node,future,timeout_sec=2)
            result['lifecycle'][name] = future.result().current_state.label if future.done() else 'timeout'
        else:
            result['lifecycle'][name] = 'unavailable'
    out = Path(__file__).resolve().parents[1]/'evidence/live_topics.json'
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
