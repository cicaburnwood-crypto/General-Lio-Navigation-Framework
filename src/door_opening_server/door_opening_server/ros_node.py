"""/inspection/detection_door_opening: empty request -> 0.0 or 100.0."""
from pathlib import Path
import json
import math
import threading
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_service
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .backend import OnnxDoorClassifier, image_to_rgb
from .policy import SamplingWindow, WINDOW_SECONDS, opening_percent

SERVICE_NAME = '/inspection/detection_door_opening'
SERVICE_TYPE = 'scan_planner_msgs/srv/DetectDoorOpening'


class DoorOpeningServer(Node):
    def __init__(self, *, predictor=None, **kwargs):
        super().__init__('door_opening_server', **kwargs)
        share = Path(get_package_share_directory('door_opening_server'))
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('model_path', str(share / 'models/door_epoch11.onnx'))
        self.declare_parameter('provider', 'auto')
        self.declare_parameter('intra_op_threads', 4)
        self.declare_parameter('max_frame_age_sec', .25)
        self.declare_parameter('image_reliability', 'best_effort')
        topic = self.get_parameter('image_topic').value
        max_age = float(self.get_parameter('max_frame_age_sec').value)
        if not topic or not 0 < max_age <= 1:
            raise ValueError('image_topic must be set and max_frame_age_sec must be in (0,1]')
        self.max_age_ns = round(max_age * 1e9)
        service_class = get_service(SERVICE_TYPE)
        if service_class.Request().get_fields_and_field_types():
            raise ValueError('DetectDoorOpening request must be empty')
        fields = service_class.Response().get_fields_and_field_types()
        if set(fields) != {'opening_percent'} or fields['opening_percent'] not in ('float', 'float32'):
            raise ValueError('DetectDoorOpening response must be float32 opening_percent')
        self.predictor = predictor if predictor is not None else OnnxDoorClassifier(
            self.get_parameter('model_path').value,
            self.get_parameter('provider').value,
            int(self.get_parameter('intra_op_threads').value))
        self.declare_parameter('opening_threshold', float(self.predictor.threshold))
        self.predictor.threshold = float(self.get_parameter('opening_threshold').value)
        if not 0 < self.predictor.threshold < 1:
            raise ValueError('opening_threshold must be in (0,1)')
        self._request_lock = threading.Lock()
        self._window_lock = threading.Lock()
        self._window = None
        self._stopping = threading.Event()
        self._service_group = ReentrantCallbackGroup()
        self._image_group = MutuallyExclusiveCallbackGroup()
        reliability=self.get_parameter('image_reliability').value
        if reliability not in ('best_effort','reliable'):raise ValueError('Invalid image_reliability')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE if reliability=='reliable' else ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(Image, topic, self.on_image, qos,
                                                     callback_group=self._image_group)
        self.details_publisher = self.create_publisher(String, SERVICE_NAME + "/details", 10)
        self.service = self.create_service(service_class, SERVICE_NAME, self.on_request,
                                           callback_group=self._service_group)
        self.get_logger().info(f'Ready: {SERVICE_NAME} [{SERVICE_TYPE}]; RGB={topic}; '
                               f'1 second / 1..5 frames / mean > threshold; threshold={self.predictor.threshold}; '
                               f'provider={getattr(self.predictor,"provider","injected test backend")}')

    def on_image(self, message):
        received = time.monotonic()
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        with self._window_lock:
            if self._window is not None:
                self._window.offer(message, stamp, received, now)

    def on_request(self, request, response):
        del request
        response.opening_percent = 0.0
        if not self._request_lock.acquire(blocking=False):
            self.get_logger().warning('Inspection already active; returning 0.0 to concurrent request')
            return response
        details = dict(started_ns=self.get_clock().now().nanoseconds,
                       probabilities=[], threshold=float(self.predictor.threshold),
                       sample_count=0, mean_probability=None, rule='mean_gt_threshold',
                       reason='检测未完成')
        try:
            with self._window_lock:
                window = SamplingWindow(time.monotonic(), self.get_clock().now().nanoseconds,
                                        self.max_age_ns)
                self._window = window
            remaining = max(0.0, window.start_monotonic + WINDOW_SECONDS - time.monotonic())
            interrupted = self._stopping.wait(remaining)
            with self._window_lock:
                self._window = None
                samples = window.selected()
            if interrupted or not window.complete():
                details['reason'] = '检测已终止，未执行模型推理' if interrupted else '本次没有有效帧，未执行模型推理'
                self.get_logger().warning(f'Capture interrupted or empty: {len(samples)} frames; returning 0.0')
                return response
            # Freshness was checked when each sample arrived. Sparse valid
            # samples must not expire merely while awaiting the 1-second window.
            probabilities = [self.predictor.score(image_to_rgb(sample.image)) for sample in samples]
            if any(not math.isfinite(float(p)) or not 0 <= p <= 1 for p in probabilities):
                raise ValueError('Invalid model probability')
            details.update(probabilities=[float(p) for p in probabilities],
                           sample_count=len(probabilities),
                           mean_probability=math.fsum(float(p) for p in probabilities)/len(probabilities),
                           stamp_ns=[sample.stamp_ns for sample in samples], reason='')
            response.opening_percent = opening_percent(probabilities, self.predictor.threshold)
            self.get_logger().info(f'opening_percent={response.opening_percent:.1f}; '
                                   f'samples={len(probabilities)}; mean={details["mean_probability"]:.5f}; '
                                   f'p_open={[round(p,5) for p in probabilities]}; '
                                   f'stamp_ns={[sample.stamp_ns for sample in samples]}')
            return response
        except Exception as error:
            details['reason'] = '推理异常：' + str(error)
            response.opening_percent = 0.0
            self.get_logger().error(f'Inspection failed; returning 0.0: {error}')
            return response
        finally:
            details['opening_percent'] = float(response.opening_percent)
            message = String()
            message.data = json.dumps(details, allow_nan=False)
            self.details_publisher.publish(message)
            with self._window_lock:
                self._window = None
            self._request_lock.release()

    def stop(self):
        self._stopping.set()


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=3)
    try:
        node = DoorOpeningServer()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
