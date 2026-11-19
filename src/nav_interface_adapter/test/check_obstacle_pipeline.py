"""Isolated ROS smoke check; pass the built adapter executable as argv[1].

Run with ROS_DOMAIN_ID reserved for testing and ROS_LOCALHOST_ONLY=1.
No odometry, navigation goals or velocity commands are published.
"""
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import TransformBroadcaster


def main():
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '61', '76')
    rclpy.init()
    node = rclpy.create_node('obstacle_pipeline_check')
    full, obstacles = {}, {}
    subscriptions = [
        node.create_subscription(PointCloud2, '/height_test/full',
                                 lambda m: full.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m),
                                 qos_profile_sensor_data),
        node.create_subscription(PointCloud2, '/height_test/obstacles',
                                 lambda m: obstacles.__setitem__((m.header.stamp.sec, m.header.stamp.nanosec), m),
                                 qos_profile_sensor_data),
    ]
    publisher = node.create_publisher(PointCloud2, '/height_test/input', qos_profile_sensor_data)
    broadcaster = TransformBroadcaster(node)
    config = '''nav_interface_adapter:
  ros__parameters:
    input_cloud_topic: /height_test/input
    input_odom_topic: /height_test/unused_odom
    output_odom_topic: /height_test/odom
    output_cloud_topic: /height_test/full
    output_obstacle_cloud_topic: /height_test/obstacles
    output_scan_topic: /height_test/scan
    cloud_input_type: pointcloud2
    cloud_frame: height_test_base
    base_frame: height_test_base
    output_cloud_frame: height_test_base
    scan_target_frame: height_test_base
    obstacle_height_frame: height_test_map
    obstacle_min_height: 0.2
    obstacle_max_height: 0.6
    publish_tf: false
    publish_lidar_tf: false
'''
    process = None
    with tempfile.TemporaryDirectory(prefix='obstacle_pipeline_') as tmp:
        config_file = Path(tmp) / 'params.yaml'
        config_file.write_text(config)
        with (Path(tmp) / 'node.log').open('w+') as log:
            try:
                process = subprocess.Popen([sys.argv[1], '--ros-args', '--params-file', str(config_file)],
                                           stdout=log, stderr=subprocess.STDOUT)

                def spin(duration, action=None):
                    end = time.monotonic() + duration
                    while time.monotonic() < end:
                        assert process.poll() is None, 'adapter exited'
                        if action:
                            action()
                        rclpy.spin_once(node, timeout_sec=0.05)

                def cloud_at(stamp, points):
                    message = PointCloud2()
                    message.header.frame_id = 'height_test_base'
                    message.header.stamp = stamp
                    message.height, message.width = 1, len(points)
                    message.fields = [PointField(name=n, offset=i * 8, datatype=PointField.FLOAT64, count=1)
                                      for i, n in enumerate(('x', 'y', 'z'))]
                    message.point_step, message.row_step = 24, 24 * len(points)
                    message.data = b''.join(struct.pack('<ddd', *p) for p in points)
                    return message

                spin(1.5)
                missing = cloud_at(node.get_clock().now().to_msg(), [(0.3, 0., 0.3)])
                spin(0.6, lambda: publisher.publish(missing))
                assert full, 'complete cloud must publish without map TF'
                assert not obstacles, 'must not silently fall back to body-Z without map TF'

                now = node.get_clock().now().nanoseconds
                stamps = [rclpy.time.Time(nanoseconds=now - offset).to_msg()
                          for offset in (2_000_000_000, 1_000_000_000)]
                transforms = []
                for index, stamp in enumerate(stamps):
                    transform = TransformStamped()
                    transform.header.frame_id = 'height_test_map'
                    transform.child_frame_id = 'height_test_base'
                    transform.header.stamp = stamp
                    transform.transform.translation.z = 17.0 if index == 0 else -30.0
                    angle = 0.0 if index == 0 else -math.pi / 2
                    transform.transform.rotation.y = math.sin(angle / 2)
                    transform.transform.rotation.w = math.cos(angle / 2)
                    transforms.append(transform)
                spin(0.5, lambda: broadcaster.sendTransform(transforms))

                points = [(0.3, 0., 4.), (0.7, 0., 0.3), (-0.1, 0., 0.3),
                          (0.1, 0., 0.1), (float('nan'), 0., 0.3)]
                for index, stamp in enumerate(stamps):
                    message = cloud_at(stamp, points)
                    key = (stamp.sec, stamp.nanosec)
                    spin(0.5, lambda: publisher.publish(message))
                    assert key in obstacles, 'filtered cloud was not published'
                    expected = points[1:3] if index == 0 else points[:1]
                    result = obstacles[key]
                    assert result.header == message.header
                    assert bytes(result.data) == b''.join(struct.pack('<ddd', *p) for p in expected), (
                        'wrong points for cloud timestamp', index, result.width)
                    assert bytes(full[key].data) == bytes(message.data), 'localization cloud was modified'

                empty = cloud_at(stamps[1], [(2., 0., 0.3)])
                spin(0.4, lambda: publisher.publish(empty))
                key = (stamps[1].sec, stamps[1].nanosec)
                assert obstacles[key].width == 0, 'all-rejected frames must publish an empty cloud'
                print('PASS: configurable bounds, timestamped map direction, translation independence, '
                      'missing TF, complete localization cloud, and empty output')
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read(), file=sys.stderr)
                raise
            finally:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
