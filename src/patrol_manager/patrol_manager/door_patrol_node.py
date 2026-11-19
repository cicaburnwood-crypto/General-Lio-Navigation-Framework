"""AL Nav2 patrol manager with a timed stop at red waypoints."""
import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener

from .door_mission import load_mission, parse_mission

RED_WAIT_SECONDS = 10.0
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
        self.publisher = self.create_publisher(String, '/patrol/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/patrol/start', self.start)
        self.create_service(Trigger, '/patrol/cancel', self.cancel)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.state, self.reason, self.index = 'IDLE', 'Configure red / blue waypoints', 0
        self.task_mode = 'patrol'
        self.mission = None
        self.red_wait_until = None
        self.goal_future = self.goal_handle = self.result_future = self.cancel_future = None
        self.cancel_requested = False
        self.timed_out = False
        self.goal_started = 0.0
        self.navigation_feedback = {}
        self.add_on_set_parameters_callback(self.validate_runtime_mission)
        self.create_timer(.05, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def publish_status(self):
        value = dict(state=self.state, reason=self.reason, point_index=self.index,
            task_mode=self.task_mode,
            points=[dict(name=p.name, type=p.type, position=p.position, yaw=p.yaw)
                    for p in self.mission.points] if self.mission else [],
            navigation=self.navigation_feedback,
            wait_remaining_sec=round(max(0.0, self.red_wait_until - time.monotonic()), 2)
                if self.red_wait_until is not None else None,
            point_count=len(self.mission.points) if self.mission else 0)
        self.publisher.publish(String(data=json.dumps(value, ensure_ascii=False, allow_nan=False)))

    def validate_runtime_mission(self, parameters):
        for parameter in parameters:
            if parameter.name != 'runtime_mission_json':
                continue
            if self.state not in TERMINAL:
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
        if self.state not in TERMINAL or self.tolerance_future is not None:
            response.success = False
            response.message = 'Task active or previous Nav2 request still completing'
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
        self.mission = mission
        self.red_wait_until = None
        self.task_mode = 'realtime' if staged else 'patrol'
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
        self.red_wait_until = None
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
        self.red_wait_until = None
        self.index += 1
        if self.index < len(self.mission.points):
            self.state, self.reason = 'WAITING_FOR_SERVER', 'Preparing next waypoint'
        else:
            self.state, self.reason = 'SUCCEEDED', 'All points completed'

    def tick(self):
        now = time.monotonic()
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
                self.red_wait_until = time.monotonic() + RED_WAIT_SECONDS
                self.state, self.reason = 'WAITING_RED', 'Red waypoint reached'
            else:
                self.advance()
        if self.state == 'WAITING_RED' and now >= self.red_wait_until:
            self.advance()
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
