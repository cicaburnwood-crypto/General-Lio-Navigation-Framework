"""Execute patrol points sequentially through the navigation Action."""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from scan_planner_msgs.action import NavigateTo
from std_msgs.msg import String
from std_srvs.srv import Trigger

from patrol_manager.mission import load_mission


class PatrolManager(Node):
    """Navigate, wait for placeholder inspection, then continue to the next point."""

    def __init__(self, **kwargs):
        super().__init__('patrol_manager', **kwargs)
        self.declare_parameter('mission_file', '')
        action_name = self.declare_parameter('navigation_action', '/navigation/navigate_to').value
        self.client = ActionClient(self, NavigateTo, action_name)
        self.publisher = self.create_publisher(
            String, '/patrol/status',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(Trigger, '/patrol/start', self.start)
        self.create_service(Trigger, '/patrol/cancel', self.cancel)

        self.state = 'IDLE'  # Patrol Manager 自定义管理巡检流程的状态变量，包含IDLE, WAITING_FOR_SERVER, WAITING_FOR_ACCEPTANCE, NAVIGATING, INSPECTING
        self.reason = 'Ready to start patrol'
        self.mission = None
        self.index = 0
        self.goal_future = None
        self.goal_handle = None
        self.result_future = None
        self.cancel_future = None
        self.cancel_requested = False
        self.navigation_feedback = {}
        self.inspection_deadline = None
        # Call tick() every 0.2s
        self.timer = self.create_timer(
            0.2, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.publish_status()

    def publish_status(self):
        """Publish a small JSON heartbeat for CLI and a future UI."""
        payload = dict(state=self.state, reason=self.reason, point_index=self.index,
                       navigation=self.navigation_feedback)
        self.publisher.publish(String(data=json.dumps(
            payload, ensure_ascii=False, allow_nan=False)))

    def start(self, request, response):
        """Load the mission and prepare its first point without dispatching navigation."""
        response.success = False
        if self.state not in {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED'}:
            response.message = 'Patrol active or navigation stop unconfirmed'
            return response
        filename = self.get_parameter('mission_file').value
        if not filename:
            response.message = 'Set mission_file before starting'
            return response
        try:
            loaded_mission = load_mission(filename)
        except Exception as error:
            response.message = 'Invalid mission: ' + str(error)
            return response

        # Commit state only after loading succeeds.
        self.mission = loaded_mission
        self.index = 0
        self.goal_future = self.goal_handle = self.result_future = self.cancel_future = None
        self.cancel_requested = False
        self.navigation_feedback = {}
        self.inspection_deadline = None
        self.state = 'WAITING_FOR_SERVER'
        self.reason = 'Mission loaded; waiting for navigation server'
        self.publish_status()
        response.success = True
        response.message = self.reason
        return response

    def build_goal(self, point):
        """Pass through mission coordinates; target Z is already base Z - 0.3."""
        # 不再对目标减高度；manager 仅在计算当前位姿到目标的 Z 距离时使用同一约定。
        goal_msg = NavigateTo.Goal()
        goal_msg.target.header.frame_id = self.mission.frame_id
        goal_msg.target.header.stamp = self.get_clock().now().to_msg()
        goal_msg.target.point.x = point.position[0]
        goal_msg.target.point.y = point.position[1]
        goal_msg.target.point.z = point.position[2]
        goal_msg.xy_tolerance = self.mission.xy_tolerance
        goal_msg.z_tolerance = self.mission.z_tolerance
        goal_msg.timeout_sec = self.mission.timeout_sec
        return goal_msg

    def send_goal(self, point):
        """Submit once without blocking, keeping the acceptance Future."""
        if self.goal_future is not None or self.goal_handle is not None:
            return False
        if not self.client.server_is_ready():
            return False
        # Send goal
        self.goal_future = self.client.send_goal_async(
            self.build_goal(point),
            feedback_callback=self.feedback,
        )
        return True

    def feedback(self, msg):
        """Copy Action feedback into JSON status; unknown distances become null."""
        def json_distance(value):
            return value if math.isfinite(value) else None

        feedback = msg.feedback
        self.navigation_feedback = {
            "task_id": bytes(msg.goal_id.uuid).hex(),
            "phase": feedback.phase,  # 下层导航阶段，区别于patrol state
            "xy_distance": json_distance(feedback.xy_distance),
            "z_distance": json_distance(feedback.z_distance),
            "remaining_path_length": json_distance(feedback.remaining_path_length),
            "path_progress_valid": feedback.path_progress_valid,
            "diagnostic_message": feedback.diagnostic_message,
        }


    def handle_result(self, response):
        """Read the Action status and NavigateTo.Result, then update patrol state."""
        # response 是 ROS 的外层响应；result 是 NavigateTo.action 中定义的结果。
        status = response.status
        result = response.result
        error_code = result.error_code
        error_reason = result.error_reason
        self.navigation_feedback.update(
            result_status=status, error_code=error_code, error_reason=error_reason)
        terminal = {
            GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED,
        }
        if error_code == NavigateTo.Result.ERROR_STOP_FAILED or status not in terminal:
            # Preserve handles when stop cannot be confirmed; no new submission is allowed.
            self.state = 'BLOCKED'
            self.reason = error_reason or 'Navigation stop is unconfirmed'
        else:
            self.goal_future = self.goal_handle = self.result_future = self.cancel_future = None
            if self.cancel_requested or status == GoalStatus.STATUS_CANCELED:
                self.state = 'CANCELED'
                self.reason = error_reason or 'Patrol canceled after navigation terminated'
            elif (status != GoalStatus.STATUS_SUCCEEDED
                  or error_code != NavigateTo.Result.ERROR_NONE):
                self.state = 'FAILED'
                self.reason = (
                    error_reason or f'Navigation failed: status={status}, error_code={error_code}')
            elif self.mission.points[self.index].is_home:
                self.state = 'SUCCEEDED'
                self.reason = 'Return home completed'
            else:
                self.state = 'INSPECTING'
                self.reason = 'Point reached; awaiting placeholder inspection start'
        self.publish_status()

    def inspect(self, point):
        """Start a placeholder wait; tick will check its deadline without blocking."""
        self.inspection_deadline = time.monotonic() + point.inspection_seconds
        self.state = 'INSPECTING'
        self.reason = (
            f'Placeholder inspection at {point.name}: wait {point.inspection_seconds:g} s; '
            'no sensor inspection performed')
        self.publish_status()

    def cancel(self, request, response):
        """Stop the queue; an accepted navigation goal must finish before cancellation ends."""
        if self.state in {'IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED', 'BLOCKED'}:
            response.success = False
            response.message = 'No cancelable patrol is running'
            return response

        self.cancel_requested = True
        if self.state in {'WAITING_FOR_SERVER', 'INSPECTING'}:
            # 没有导航在执行，可以直接结束队列。
            self.inspection_deadline = None
            self.state = 'CANCELED'
            self.reason = 'Patrol canceled'
        elif self.state == 'WAITING_FOR_ACCEPTANCE':
            # 目标已经发出，先等接受答复；接受后再请求取消。
            self.reason = 'Cancellation requested; waiting for goal acceptance'
        else:
            self.state = 'CANCELING'
            self.reason = 'Cancellation requested; waiting for navigation to finish'

        self.publish_status()
        response.success = True
        response.message = self.reason
        return response

    def tick(self):
        """Advance one stage without blocking; each Future is read only when done."""
        if self.state == 'WAITING_FOR_SERVER':
            point = self.mission.points[self.index]
            if self.send_goal(point):
                self.navigation_feedback = {}
                self.state = 'WAITING_FOR_ACCEPTANCE'
                self.reason = f'Goal sent: {point.name}; waiting for acceptance'

        elif self.state == 'WAITING_FOR_ACCEPTANCE':
            if self.goal_future.done():
                self.goal_handle = self.goal_future.result()
                self.goal_future = None
                if not self.goal_handle.accepted:
                    self.goal_handle = None
                    self.state = 'CANCELED' if self.cancel_requested else 'FAILED'
                    self.reason = 'Navigation goal rejected'
                else:
                    self.result_future = self.goal_handle.get_result_async()
                    self.state = 'CANCELING' if self.cancel_requested else 'NAVIGATING'
                    point = self.mission.points[self.index]
                    self.reason = f'Navigation accepted: {point.name}'

        elif self.state in {'NAVIGATING', 'CANCELING'}:
            # 最终结果才代表导航结束；取消服务的答复不能替代它。
            if self.result_future.done():
                self.handle_result(self.result_future.result())
            elif self.state == 'CANCELING':
                if self.cancel_future is None:
                    self.cancel_future = self.goal_handle.cancel_goal_async()
                    self.reason = 'Cancel sent; waiting for navigation to finish'
                elif self.cancel_future.done():
                    reply = self.cancel_future.result()
                    self.reason = (
                        'Cancel accepted; waiting for navigation to finish'
                        if reply.goals_canceling else
                        'Cancel not accepted; still waiting for navigation to finish')

        elif self.state == 'INSPECTING':
            if self.inspection_deadline is None:
                self.inspect(self.mission.points[self.index])
            elif time.monotonic() >= self.inspection_deadline:
                self.inspection_deadline = None
                self.index += 1
                if self.index < len(self.mission.points):
                    self.state = 'WAITING_FOR_SERVER'
                    self.reason = 'Inspection wait completed; preparing next point'
                else:
                    self.state = 'SUCCEEDED'
                    self.reason = 'All patrol points completed'

        self.publish_status()


def main(args=None):
    """Run the patrol node."""
    rclpy.init(args=args)
    node = PatrolManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
