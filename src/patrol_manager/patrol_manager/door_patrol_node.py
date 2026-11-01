"""Nav2 adaptation of patrol_manager: this manager alone dispatches the queue.

No velocity publisher. Nav2 owns stopping and goal orientation; measured odometry
and map TF confirm the stop before the existing door service is invoked.
"""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scan_planner_msgs.srv import DetectDoorOpening
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .door_mission import load_mission, parse_mission
from .door_gate import DoorGate
from .odom_motion import OdomMotion

TERMINAL = {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED'}


class DoorPatrolManager(Node):
    def __init__(self, **kwargs):
        super().__init__('patrol_manager', **kwargs)
        self.declare_parameter('mission_file', '')
        # Staged through ROS's existing atomic-parameter service, consumed by
        # /patrol/start once. Never written to mission_file or another YAML.
        self.declare_parameter('runtime_mission_json', '')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('nav2_xy_tolerance', .05)
        self.declare_parameter('nav2_yaw_tolerance', .087)
        self.declare_parameter('blue_tolerance_multiplier', 2.0)
        self.tolerance_client = self.create_client(SetParametersAtomically,
            '/controller_server/set_parameters_atomically')
        self.tolerance_future = None
        name = self.declare_parameter('navigation_action', '/navigate_to_pose').value
        self.client = ActionClient(self, NavigateToPose, name)
        self.door_client = self.create_client(DetectDoorOpening, '/inspection/detection_door_opening')
        self.publisher = self.create_publisher(String, '/patrol/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/patrol/start', self.start)
        self.create_service(Trigger, '/patrol/cancel', self.cancel)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Odometry, '/odom', self.on_odom, qos_profile_sensor_data)
        self.state, self.reason, self.index = 'IDLE', 'Configure red / blue waypoints', 0
        self.task_mode = 'patrol'
        self.door_calls_completed = 0
        self.stop_diagnostics = {}
        self.mission = self.gate = None
        self.odom_motion = OdomMotion()
        self.goal_future = self.goal_handle = self.result_future = self.cancel_future = None
        self.door_future = None
        self.cancel_requested = False
        self.timed_out = False
        self.goal_started = 0.0
        self.navigation_feedback = {}
        self.add_on_set_parameters_callback(self.validate_runtime_mission)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def publish_status(self):
        gate = self.gate
        value = dict(state=self.state, reason=self.reason, point_index=self.index,
            task_mode=self.task_mode,
            points=[dict(name=p.name, type=p.type, position=p.position, yaw=p.yaw)
                    for p in self.mission.points] if self.mission else [],
            navigation=self.navigation_feedback,
            door=dict(calls=self.door_calls_completed + (gate.calls if gate else 0),
                      in_flight=bool(gate and gate.in_flight),
                      opening_percent=gate.last_opening if gate else None,
                      position_latched=bool(gate and gate.position_reached),
                      settling=self.stop_diagnostics),
            point_count=len(self.mission.points) if self.mission else 0)
        self.publisher.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

    def validate_runtime_mission(self, parameters):
        for parameter in parameters:
            if parameter.name != 'runtime_mission_json':
                continue
            if self.state not in TERMINAL or self.door_future is not None:
                return SetParametersResult(successful=False, reason='Manager already has an active task')
            if not isinstance(parameter.value, str):
                return SetParametersResult(successful=False, reason='runtime_mission_json must be a string')
            try:
                if parameter.value:
                    parse_mission(json.loads(parameter.value))
            except (ValueError, TypeError, KeyError) as exc:
                return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def require_localization(self):
        try:
            transform = self.tf.lookup_transform('map', self.get_parameter('base_frame').value, Time())
            stamp = transform.header.stamp.sec + transform.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9 - stamp
            if stamp == 0 or not 0 <= age <= 1.0:
                raise ValueError('map → base_link is not current')
        except Exception as exc:
            raise ValueError(f'Real-time mode requires localization: {exc}') from exc

    def start(self, request, response):
        del request
        if self.state not in TERMINAL or self.door_future is not None or self.tolerance_future is not None:
            response.success = False
            response.message = 'Task active or previous detector request still completing'
            return response
        try:
            staged = self.get_parameter('runtime_mission_json').value
            if staged:
                # Consume even on a rejected start: a later /patrol/start must
                # never accidentally execute an old transient queue.
                self.set_parameters([Parameter('runtime_mission_json', value='')])
                mission = parse_mission(json.loads(staged))
                self.require_localization()
            else:
                mission = load_mission(self.get_parameter('mission_file').value)
        except Exception as exc:
            response.success, response.message = False, str(exc)
            return response
        self.mission, self.gate = mission, None
        self.task_mode = 'realtime' if staged else 'patrol'
        self.door_calls_completed = 0
        self.stop_diagnostics = {}
        self.odom_motion = OdomMotion()
        self.goal_future = self.goal_handle = self.result_future = self.cancel_future = None
        self.index, self.cancel_requested, self.timed_out = 0, False, False
        self.navigation_feedback = {}
        self.state, self.reason = 'WAITING_FOR_SERVER', 'Waiting for Nav2'
        response.success, response.message = True, self.reason
        self.publish_status()
        return response

    def cancel(self, request, response):
        del request
        if self.state in TERMINAL:
            response.success, response.message = False, 'No active task'
            return response
        self.cancel_requested = True
        if self.goal_future is None and self.goal_handle is None:
            self.state, self.reason = 'CANCELED', 'Queue canceled; no next goal will be sent'
        else:
            self.state, self.reason = 'CANCELING', 'Waiting for Nav2 terminal cancellation result'
        response.success, response.message = True, self.reason
        self.publish_status()
        return response

    def feedback(self, message):
        f = message.feedback
        distance = float(f.distance_remaining)
        self.navigation_feedback = dict(distance_remaining=distance if math.isfinite(distance) else None,
                                        number_of_recoveries=int(f.number_of_recoveries))

    def on_odom(self, message):
        if self.state not in {'SETTLING', 'WAITING_DOOR', 'DETECTING'} or not self.gate:
            return
        # Receipt alone is not enough: reject buffered/old odometry and map TF.
        now = self.get_clock().now().nanoseconds
        stamp = message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if stamp == 0 or not 0 <= (now-stamp)/1e9 <= self.mission.observation_max_age_sec:
            return
        try:
            transform = self.tf.lookup_transform('map', self.get_parameter('base_frame').value, Time())
        except Exception:
            return
        ts = transform.header.stamp.sec*10**9+transform.header.stamp.nanosec
        if ts == 0 or not 0 <= (now-ts)/1e9 <= self.mission.observation_max_age_sec:
            return
        p, q = transform.transform.translation, transform.transform.rotation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        v = message.twist.twist
        op, oq = message.pose.pose.position, message.pose.pose.orientation
        norm = math.sqrt(sum(value*value for value in (oq.x, oq.y, oq.z, oq.w)))
        if not math.isfinite(norm) or norm < .5:
            return
        x, y, z, w = (value/norm for value in (oq.x, oq.y, oq.z, oq.w))
        oyaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        measured = self.odom_motion.observe(stamp/1e9,
            (message.header.frame_id, message.child_frame_id), (op.x, op.y, oyaw),
            (v.linear.x, v.linear.y, v.angular.z), self.mission.observation_max_age_sec)
        if measured is not None:
            self.gate.observe(time.monotonic(), (p.x, p.y, yaw), measured)
            target_yaw = self.mission.points[self.index].yaw
            yaw_error = abs(math.atan2(math.sin(yaw-target_yaw), math.cos(yaw-target_yaw)))
            self.stop_diagnostics = dict(source='odom_pose_1s_window_and_reported_twist',
                linear_mps=measured[0], angular_radps=measured[2], yaw_error_rad=yaw_error)
        else:
            self.stop_diagnostics = dict(source='odom_pose_1s_window_and_reported_twist',
                waiting_for_window=True)

    def dispatch(self):
        p = self.mission.points[self.index]
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.mission.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y, goal.pose.pose.position.z = p.position
        goal.pose.pose.orientation.z = math.sin(p.yaw/2)
        goal.pose.pose.orientation.w = math.cos(p.yaw/2)
        self.goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        self.goal_started = time.monotonic()
        self.state, self.reason = 'WAITING_FOR_ACCEPTANCE', f'Goal sent by manager: {p.name}'

    def configure_goal_tolerance(self):
        """Use Nav2's existing checker and parameter service, not custom arrival."""
        p = self.mission.points[self.index]
        scale = self.get_parameter('blue_tolerance_multiplier').value if p.type == 'blue' else 1.0
        request = SetParametersAtomically.Request()
        request.parameters = [Parameter('general_goal_checker.' + name, value=float(
            self.get_parameter(base).value * scale)).to_parameter_msg()
            for name, base in [('xy_goal_tolerance','nav2_xy_tolerance'),
                               ('yaw_goal_tolerance','nav2_yaw_tolerance')]]
        self.tolerance_future = self.tolerance_client.call_async(request)
        self.state, self.reason = 'CONFIGURING_GOAL', f'Applying native Nav2 tolerance for {p.type} point'

    def advance(self):
        if self.gate:
            self.door_calls_completed += self.gate.calls
            self.gate = None
        self.stop_diagnostics = {}
        self.index += 1
        if self.index < len(self.mission.points):
            self.state, self.reason = 'WAITING_FOR_SERVER', 'Preparing next waypoint'
        else:
            self.state, self.reason = 'SUCCEEDED', 'All points completed'

    def tick(self):
        now = time.monotonic()
        # A canceled service cannot be remotely canceled: drain, never reuse it.
        if self.state in TERMINAL and self.door_future is not None and self.door_future.done():
            self.door_future = None
            if self.gate:
                self.gate.in_flight = False
        if (self.state == 'WAITING_FOR_SERVER' and self.client.server_is_ready()
                and self.tolerance_client.service_is_ready()):
            self.configure_goal_tolerance()
        if self.tolerance_future is not None and self.tolerance_future.done():
            future, self.tolerance_future = self.tolerance_future, None
            # A parameter response from a canceled task must never send a goal.
            if self.state == 'CONFIGURING_GOAL' and not self.cancel_requested:
                try:
                    result = future.result().result
                    if not result.successful:
                        raise RuntimeError(result.reason)
                    self.dispatch()
                except Exception as exc:
                    self.state, self.reason = 'FAILED', f'Nav2 tolerance update failed: {exc}'
        if self.goal_future is not None and self.goal_future.done():
            try:
                self.goal_handle = self.goal_future.result()
            except Exception as exc:
                self.state, self.reason = 'FAILED', str(exc)
            self.goal_future = None
            if self.goal_handle is not None:
                if not self.goal_handle.accepted:
                    self.goal_handle = None
                    self.state = 'CANCELED' if self.cancel_requested else 'FAILED'
                    self.reason = 'Nav2 rejected goal'
                else:
                    self.result_future = self.goal_handle.get_result_async()
                    self.state = 'CANCELING' if self.cancel_requested else 'NAVIGATING'
                    self.reason = 'Nav2 goal accepted'
        if self.state in {'NAVIGATING', 'WAITING_FOR_ACCEPTANCE'} and now-self.goal_started > self.mission.timeout_sec:
            self.timed_out = self.cancel_requested = True
            self.state, self.reason = 'CANCELING', 'Navigation timeout; waiting for confirmed stop'
        if self.state == 'CANCELING' and self.goal_handle is not None and self.cancel_future is None:
            self.cancel_future = self.goal_handle.cancel_goal_async()
        if self.result_future is not None and self.result_future.done():
            try:
                status = self.result_future.result().status
            except Exception as exc:
                self.state, self.reason = 'BLOCKED', f'Nav2 final result unavailable: {exc}'
                self.publish_status()
                return
            if status not in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED):
                self.state, self.reason = 'BLOCKED', f'Nav2 result is not terminal: status={status}'
                self.publish_status()
                return
            self.goal_handle = self.result_future = self.cancel_future = None
            if self.timed_out:
                self.state, self.reason = 'FAILED', 'Navigation timeout; Nav2 terminated'
            elif self.cancel_requested or status == GoalStatus.STATUS_CANCELED:
                self.state, self.reason = 'CANCELED', 'Nav2 terminated; queue canceled'
            elif status != GoalStatus.STATUS_SUCCEEDED:
                self.state, self.reason = 'FAILED', f'Nav2 failed: status={status}'
            elif self.mission.points[self.index].type == 'red':
                self.gate = DoorGate(self.mission, self.index, navigation_succeeded=True)
                self.odom_motion = OdomMotion()
                self.state, self.reason = 'SETTLING', 'Door-front goal reached; waiting for measured standstill and heading'
            else:
                # Blue waypoints do not wait, settle, or call the detector.
                self.advance()
        if self.state in {'SETTLING', 'WAITING_DOOR', 'DETECTING'}:
            settled = self.gate.settled(now)
            if self.door_future is not None and self.door_future.done():
                try:
                    opening = float(self.door_future.result().opening_percent)
                except Exception as exc:
                    opening = 0.0
                    self.reason = f'Detector error; staying stopped: {exc}'
                self.door_future = None
                if self.gate.finish(now, opening):
                    self.reason = 'Door fully open (100); manager releases next waypoint'
                    self.advance()
                else:
                    self.state = 'WAITING_DOOR' if settled else 'SETTLING'
                    self.reason = 'Door detection did not succeed; waiting for next detection'
            elif self.door_future is None:
                self.state = 'WAITING_DOOR' if settled else 'SETTLING'
                if self.door_client.service_is_ready() and self.gate.begin(now):
                    self.door_future = self.door_client.call_async(DetectDoorOpening.Request())
                    self.state, self.reason = 'DETECTING', 'Capturing 1 s / averaging 1..5 fresh RGB frames'
                elif settled and not self.door_client.service_is_ready():
                    self.reason = 'Waiting stopped for door detection service'
        self.publish_status()


def main(args=None):
    rclpy.init(args=args)
    node = DoorPatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.goal_handle is not None:
            node.goal_handle.cancel_goal_async()
            # Let the cancel request leave DDS; final cancellation is observed
            # by /patrol/cancel during normal operation, not inferred here.
            rclpy.spin_once(node, timeout_sec=.2)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
