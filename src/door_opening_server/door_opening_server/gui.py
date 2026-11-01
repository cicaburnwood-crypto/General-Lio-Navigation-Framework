"""Local browser GUI: RGB topic preview plus a real empty ROS service request."""
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from PIL import Image as PILImage
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_service
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backend import image_to_rgb
from .ros_node import SERVICE_NAME, SERVICE_TYPE


class DoorOpeningGui(Node):
    def __init__(self, **kwargs):
        super().__init__('door_opening_gui', **kwargs)
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('gui_host', '127.0.0.1')
        self.declare_parameter('gui_port', 8878)
        self.topic = self.get_parameter('image_topic').value
        self._guard = threading.Lock()
        self._request_guard = threading.Lock()
        self._preview = False
        self._jpeg = None
        self._last_image_time = 0.0
        self._last_encode_time = 0.0
        self._last_result = None
        self._details = None
        self._details_event = threading.Event()
        self._busy = False
        self._image_error = ''
        self._camera_source = 'live'
        self.service_type = get_service(SERVICE_TYPE)
        self.client = self.create_client(self.service_type, SERVICE_NAME)
        self.usb_publisher = self.create_publisher(Image, self.topic, 1)
        self.details_subscription = self.create_subscription(String, SERVICE_NAME + "/details", self.on_details, 10)
        self.subscription = self.create_subscription(Image, self.topic, self.on_image, qos_profile_sensor_data)

    def on_details(self, message):
        try:
            details = json.loads(message.data)
            with self._guard:
                self._details = details
                self._details_event.set()
        except (ValueError, TypeError):
            pass

    def publish_usb_frame(self, data, width, height, stamp_ns):
        if not (0 < width <= 1920 and 0 < height <= 1080) or len(data) != width*height*3:
            raise ValueError('Invalid USB RGB frame')
        message=Image();message.width=width;message.height=height;message.step=width*3
        message.encoding='rgb8';message.data=data;message.header.frame_id='usb_rgb_camera'
        message.header.stamp.sec=stamp_ns//1_000_000_000
        message.header.stamp.nanosec=stamp_ns%1_000_000_000
        self.usb_publisher.publish(message)

    def set_preview(self, enabled):
        with self._guard:
            self._preview = bool(enabled)
            self._jpeg = None
            self._last_image_time = 0.0
            self._last_result = None
            self._image_error = ''

    def on_image(self, message):
        now = time.monotonic()
        with self._guard:
            if not self._preview or now - self._last_encode_time < .10:
                return
            self._last_encode_time = now
        try:
            rgb = image_to_rgb(message)
            image = PILImage.fromarray(rgb)
            image.thumbnail((1280,720))
            output = io.BytesIO(); image.save(output, format='JPEG', quality=82)
            with self._guard:
                if self._preview:
                    self._jpeg = output.getvalue()
                    self._last_image_time = now
                    self._image_error = ''
                    self._camera_source = 'recorded_test_frame_only' if message.header.frame_id == 'recorded_test_frame_only' else 'live'
        except Exception as error:
            with self._guard:
                self._image_error = str(error)

    def status(self):
        with self._guard:
            return dict(camera_open=self._preview,
                        camera_ready=self._preview and self._jpeg is not None and time.monotonic()-self._last_image_time<1.0,
                        service_ready=self.client.service_is_ready(), image_topic=self.topic,
                        busy=self._busy, last_result=self._last_result, image_error=self._image_error,
                        camera_source=self._camera_source)

    def jpeg(self):
        with self._guard:
            return self._jpeg

    def detect(self):
        if not self._request_guard.acquire(blocking=False):
            raise RuntimeError('已有检测请求正在处理')
        try:
            state = self.status()
            if not state['camera_ready']:
                raise RuntimeError('尚未收到实时 RGB 图像，请先打开 Camera 并检查相机 topic')
            if not state['service_ready']:
                raise RuntimeError('门状态检测服务尚未就绪')
            with self._guard:
                self._busy = True
                self._last_result = None
            with self._guard:
                self._details = None
                self._details_event.clear()
            request_ns = self.get_clock().now().nanoseconds
            started = time.monotonic()
            future = self.client.call_async(self.service_type.Request())
            completed = threading.Event()
            future.add_done_callback(lambda _:completed.set())
            if not completed.wait(20.0):
                future.cancel()
                raise RuntimeError('服务请求超时，请检查检测节点日志')
            response = future.result()
            value = float(response.opening_percent)
            if not math.isfinite(value) or value not in (0.0,100.0):
                raise RuntimeError('服务返回了约定之外的值')
            result = dict(opening_percent=value, duration_s=round(time.monotonic()-started,3),
                          completed_at=datetime.now().astimezone().isoformat(timespec='seconds'))
            self._details_event.wait(0.5)
            with self._guard:
                details = self._details
                if (details and details.get('started_ns', 0) >= request_ns
                        and details.get('opening_percent') == value):
                    result.update(probabilities=details['probabilities'],
                                  threshold=details['threshold'], reason=details['reason'],
                                  sample_count=details.get('sample_count',len(details['probabilities'])),
                                  mean_probability=details.get('mean_probability'),
                                  rule=details.get('rule'))
                else:
                    result.update(probabilities=[], reason='本次逐帧概率尚未收到')
                self._last_result = result
            return result
        finally:
            with self._guard:
                self._busy = False
            self._request_guard.release()


def create_http_server(node, host, port):
    document = (Path(get_package_share_directory('door_opening_server'))/'web/index.html').read_bytes()
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def reply(self, code, value, mime='application/json; charset=utf-8'):
            body = json.dumps(value,ensure_ascii=False).encode() if isinstance(value,dict) else value
            self.send_response(code)
            self.send_header('Content-Type',mime)
            self.send_header('Content-Length',str(len(body)))
            self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):pass

        def do_GET(self):
            path = self.path.split('?',1)[0]
            if path in ('/','/index.html'):
                self.reply(200,document,'text/html; charset=utf-8')
            elif path=='/api/status':
                self.reply(200,node.status())
            elif path=='/api/frame.jpg':
                image=node.jpeg()
                if image is None:self.reply(404,dict(error='等待相机图像'))
                else:self.reply(200,image,'image/jpeg')
            else:self.reply(404,dict(error='Not found'))

        def do_POST(self):
            # No cross-origin browser controls; the GUI is served from this same origin.
            origin=self.headers.get('Origin')
            if origin and origin != 'http://' + self.headers.get('Host',''):
                self.reply(403,dict(error='Cross-origin request rejected'));return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if self.path=='/api/usb_frame':
                    if not 0 < length <= 1920*1080*3:raise ValueError('Invalid RGB payload size')
                    node.publish_usb_frame(self.rfile.read(length),int(self.headers['X-Width']),
                                           int(self.headers['X-Height']),int(self.headers['X-Capture-Stamp-Ns']))
                    self.reply(200,dict(ok=True));return
                if not 0<=length<=1024:raise ValueError('Invalid request size')
                payload=json.loads(self.rfile.read(length) or b'{}')
                if self.path=='/api/camera':
                    if type(payload.get('open')) is not bool:raise ValueError('open must be boolean')
                    node.set_preview(payload['open']);self.reply(200,node.status())
                elif self.path=='/api/detect':
                    self.reply(200,node.detect())
                else:self.reply(404,dict(error='Not found'))
            except (ValueError, RuntimeError) as error:
                self.reply(409,dict(error=str(error)))
            except Exception as error:
                self.reply(500,dict(error='服务调用失败：'+str(error)))
    return ThreadingHTTPServer((host,port),Handler)


def main(args=None):
    rclpy.init(args=args)
    node=DoorOpeningGui()
    host=node.get_parameter('gui_host').value
    port=int(node.get_parameter('gui_port').value)
    executor=MultiThreadedExecutor(num_threads=2);executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    server=create_http_server(node,host,port)
    node.get_logger().info(f'Open GUI: http://{host}:{server.server_port}/ ; camera topic={node.topic}')
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:
        server.server_close();executor.shutdown();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        thread.join(timeout=3)
