#!/usr/bin/env python3
"""AL's RGB-only UVC publisher. No depth, navigation or velocity interfaces."""
from array import array
from pathlib import Path
import threading
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from rgb_recording import NativeRecorder


class RGBPublisher(Node):
    def __init__(self):
        super().__init__('al_d435i_rgb')
        self.device = self.declare_parameter('device', '/dev/v4l/by-path/pci-0000:00:14.0-usb-0:1.1:1.3-video-index0').value
        topic = self.declare_parameter('image_topic', '/camera/d435i/color/image_raw').value
        self.publisher = self.create_publisher(Image, topic, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.sent = None
        self.recorder = NativeRecorder(Path(__file__).resolve().parents[1]/'recordings/rgb')
        self.worker = threading.Thread(target=self.capture, daemon=True)
        self.worker.start()
        self.create_timer(1/15, self.publish_latest)

    def capture(self):
        while not self.stop.is_set():
            cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
            try:
                if not cap.isOpened():
                    raise RuntimeError('Cannot open '+self.device)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'YUYV'))
                # Match the native YUYV recording mode used by TR1..8 / VAL1..4.
                # The detector performs the training resize to 640x360 and
                # centered 640x640 letterbox; publish the full RGB frame here.
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
                cap.set(cv2.CAP_PROP_FPS, 30)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
                pixel_format = ''.join(chr((fourcc >> (8*i)) & 0xff) for i in range(4))
                self.get_logger().info(
                    f'D435i RGB capture opened: {self.device}; '
                    f'actual={int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x'
                    f'{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} '
                    f'@ {cap.get(cv2.CAP_PROP_FPS):g} FPS; format={pixel_format}')
                failed = 0
                while not self.stop.is_set():
                    ok, frame = cap.read()
                    stamp = self.get_clock().now().to_msg()
                    if not ok or frame is None:
                        failed += 1
                        if failed >= 5:
                            raise RuntimeError('D435i capture interrupted; reopening')
                        self.stop.wait(.1)
                        continue
                    failed = 0
                    self.recorder.offer(frame, time.monotonic())
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    with self.lock:
                        self.latest = (stamp, rgb)
            except Exception as exc:
                self.get_logger().warning(str(exc))
            finally:
                cap.release()
                with self.lock:
                    self.latest = None
            self.stop.wait(1)

    def publish_latest(self):
        with self.lock:
            latest = self.latest
        if latest is None:
            return
        stamp, frame = latest
        identity = (stamp.sec, stamp.nanosec)
        if identity == self.sent:
            return
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = 'd435i_color_optical_frame'
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'rgb8'
        msg.step = msg.width*3
        # A typed byte array avoids ROS's two Python per-byte validation loops
        # on each native 1080p image, without changing any pixel values.
        msg.data = array('B', frame.tobytes())
        self.publisher.publish(msg)
        self.sent = identity


def main():
    rclpy.init()
    node = RGBPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.worker.join(timeout=3)
        node.recorder.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
