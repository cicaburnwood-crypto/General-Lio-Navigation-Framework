"""Real DDS and Nav2 action protocol, simulated endpoints, no physical driver."""
import asyncio
import math
import json
import os
import tempfile
import threading
import time
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest
import rclpy
from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import SetParametersResult
from scan_planner_msgs.srv import DetectDoorOpening
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
import yaml

from patrol_manager.door_patrol_node import DoorPatrolManager
from test_gate import config


def until(predicate, seconds=10):
    deadline=time.monotonic()+seconds
    while not predicate():
        assert time.monotonic()<deadline, 'Timed out'
        time.sleep(.02)


class Fixtures(Node):
    def __init__(self):
        super().__init__('door_workflow_test_fixture')
        self.goals=[];self.calls=[];self.opening=0.;self.speed=.1;self.yaw=0.
        self.hold=False;self.cancel_seen=False
        self.odom_x=1.;self.odom_step=0.
        self.odom_jitter=0.;self.sample_index=0
        self.pose_x=1.;self.pose_y=2.;self.observing=True
        group=ReentrantCallbackGroup()
        self.server=ActionServer(self,NavigateToPose,'/navigate_to_pose',self.execute,
            cancel_callback=lambda _:CancelResponse.ACCEPT,callback_group=group)
        self.create_service(DetectDoorOpening,'/inspection/detection_door_opening',self.detect,callback_group=group)
        self.tolerances=[]
        self.create_service(SetParametersAtomically,'/controller_server/set_parameters_atomically',
                            self.set_tolerance,callback_group=group)
        self.odom=self.create_publisher(Odometry,'/odom',10)
        self.tf=TransformBroadcaster(self)
        self.create_timer(.05,self.observe,callback_group=group)
    def set_tolerance(self, request, response):
        self.tolerances.append({p.name:p.value.double_value for p in request.parameters})
        response.result=SetParametersResult(successful=True)
        return response
    def execute(self,handle):
        self.goals.append(handle.request)
        while self.hold:
            if handle.is_cancel_requested:
                self.cancel_seen=True;handle.canceled();return NavigateToPose.Result()
            time.sleep(.02)
        handle.succeed();return NavigateToPose.Result()
    def detect(self,req,response):
        self.calls.append(time.monotonic())
        time.sleep(.3)
        response.opening_percent=self.opening
        return response
    def observe(self):
        if not self.observing:return
        stamp=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id='map';t.child_frame_id='base_link'
        t.transform.translation.x=self.pose_x;t.transform.translation.y=self.pose_y
        t.transform.rotation.z=math.sin(self.yaw/2);t.transform.rotation.w=math.cos(self.yaw/2)
        self.tf.sendTransform(t)
        msg=Odometry();msg.header.stamp=stamp;msg.header.frame_id='odom';msg.child_frame_id='base_link'
        self.odom_x+=self.odom_step
        self.sample_index+=1
        noise=self.odom_jitter*(-1 if self.sample_index%2 else 1)
        msg.pose.pose.position.x=self.odom_x+noise;msg.pose.pose.position.y=2.-noise
        msg.pose.pose.orientation.z=math.sin((self.yaw+noise)/2)
        msg.pose.pose.orientation.w=math.cos((self.yaw+noise)/2)
        msg.twist.twist.linear.x=self.speed
        self.odom.publish(msg)


@pytest.fixture
def system(tmp_path):
    assert os.environ.get('ROS_DOMAIN_ID')=='192', 'Integration tests require isolated domain 192'
    assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init()
    path=tmp_path/'mission.yaml';path.write_text(yaml.safe_dump(config()))
    manager=DoorPatrolManager(parameter_overrides=[Parameter('mission_file',value=str(path))])
    fixture=Fixtures();executor=MultiThreadedExecutor(num_threads=5)
    executor.add_node(manager);executor.add_node(fixture)
    thread=threading.Thread(target=executor.spin);thread.start()
    start=fixture.create_client(Trigger,'/patrol/start');cancel=fixture.create_client(Trigger,'/patrol/cancel')
    assert start.wait_for_service(timeout_sec=5)
    def call(client):
        f=client.call_async(Trigger.Request());until(f.done);return f.result()
    yield manager,fixture,lambda:call(start),lambda:call(cancel)
    fixture.hold=False
    executor.shutdown();thread.join(timeout=5)
    manager.destroy_node();fixture.destroy_node();rclpy.shutdown()


def test_stop_face_poll_then_manager_sends_next(system):
    manager,fixture,start,cancel=system
    assert start().success
    until(lambda:manager.state=='SETTLING')
    time.sleep(1.2);assert not fixture.calls and len(fixture.goals)==1
    fixture.speed=0.;time.sleep(1.2);assert not fixture.calls # wrong yaw
    fixture.yaw=math.pi/2
    until(lambda:len(fixture.calls)>=2)
    assert len(fixture.goals)==1
    assert fixture.calls[1]-fixture.calls[0]>=.98
    first=fixture.goals[0].pose.pose
    assert first.position.x==1 and first.orientation.z==pytest.approx(math.sin(math.pi/4))
    fixture.opening=100.
    until(lambda:len(fixture.goals)==2)
    assert fixture.goals[1].pose.pose.position.y==4.
    assert fixture.tolerances[0]['general_goal_checker.xy_goal_tolerance']==.05
    assert fixture.tolerances[1]['general_goal_checker.xy_goal_tolerance']==.1
    until(lambda:manager.state=='SUCCEEDED')


def test_nav2_success_with_xy_drift_still_detects_and_releases_next(system):
    manager,fixture,start,cancel=system
    fixture.pose_x=1.1 # Outside 5 cm, as observed after real Nav2 success.
    fixture.speed=0.;fixture.yaw=math.pi/2
    assert start().success
    until(lambda:len(fixture.calls)>=1)
    assert manager.gate.position_reached
    assert len(fixture.goals)==1 # Closed door still prevents the next goal.
    fixture.opening=100.
    until(lambda:manager.state=='SUCCEEDED')
    assert len(fixture.goals)==2


def test_cancel_during_detection_discards_late_open(system):
    manager,fixture,start,cancel=system
    fixture.speed=0.;fixture.yaw=math.pi/2;fixture.opening=100.
    assert start().success
    until(lambda:manager.state=='DETECTING')
    assert cancel().success
    time.sleep(.6)
    assert manager.state=='CANCELED' and len(fixture.goals)==1
    assert manager.door_future is None


def test_cancel_active_nav_waits_native_result(system):
    manager,fixture,start,cancel=system
    fixture.hold=True
    assert start().success
    until(lambda:manager.state=='NAVIGATING')
    assert cancel().success
    until(lambda:manager.state=='CANCELED')
    assert fixture.cancel_seen and not fixture.calls and len(fixture.goals)==1


def test_fastlio_zero_twist_while_pose_moves_is_not_stopped(system):
    manager,fixture,start,cancel=system
    fixture.speed=0.;fixture.yaw=math.pi/2;fixture.odom_step=.01
    assert start().success
    until(lambda:manager.state=='SETTLING')
    time.sleep(1.4)
    assert not fixture.calls and len(fixture.goals)==1
    fixture.odom_step=0.
    until(lambda:len(fixture.calls)>=1)
    assert len(fixture.goals)==1
    assert cancel().success


def test_stationary_pose_jitter_reaches_detector_and_next_goal(system):
    manager,fixture,start,cancel=system
    fixture.speed=0.;fixture.yaw=math.pi/2;fixture.odom_jitter=.005
    assert start().success
    until(lambda:len(fixture.calls)>=1)
    assert len(fixture.goals)==1 and manager.gate.position_reached
    assert manager.stop_diagnostics['linear_mps']<manager.mission.linear_stopped_mps
    fixture.opening=100.
    until(lambda:manager.state=='SUCCEEDED')
    assert len(fixture.goals)==2


def stage(fixture, value):
    """The GUI uses this existing native ROS parameter service, not a new srv."""
    client=fixture.create_client(SetParametersAtomically,'/patrol_manager/set_parameters_atomically')
    assert client.wait_for_service(timeout_sec=3)
    request=SetParametersAtomically.Request(parameters=[Parameter('runtime_mission_json',value=json.dumps(value)).to_parameter_msg()])
    future=client.call_async(request);until(future.done)
    reply=future.result().result;fixture.destroy_client(client)
    return reply


def localized(manager):
    try:manager.require_localization();return True
    except ValueError:return False


def test_runtime_blue_points_never_settle_or_detect_and_do_not_write_yaml(system):
    manager,fixture,start,cancel=system
    path=Path(manager.get_parameter('mission_file').value);before=path.read_bytes();mtime=path.stat().st_mtime_ns
    c=config()
    for p in c['points']:p.update(type='blue',inspection_duration_sec=99)
    until(lambda:localized(manager))
    assert stage(fixture,c).successful and start().success
    until(lambda:manager.state=='SUCCEEDED')
    assert len(fixture.goals)==2 and not fixture.calls
    assert manager.task_mode=='realtime'
    assert manager.get_parameter('runtime_mission_json').value==''
    assert path.read_bytes()==before and path.stat().st_mtime_ns==mtime
    assert list(path.parent.iterdir())==[path]
    # No stale transient queue: next ordinary /patrol/start uses saved YAML.
    assert start().success
    until(lambda:manager.state=='SETTLING')
    assert manager.task_mode=='patrol' and manager.mission.points[0].type=='red'
    assert cancel().success


def test_runtime_blue_red_blue_red_requires_each_door(system):
    manager,fixture,start,cancel=system
    fixture.speed=0.;fixture.yaw=math.pi/2
    c=config();c['points']=[dict(name=f'P{i+1}',position=[1.,float(i+1),0.],yaw=math.pi/2,type=kind)
                          for i,kind in enumerate(['blue','red','blue','red'])]
    c['points'][3]['yaw']=0.
    until(lambda:localized(manager))
    assert stage(fixture,c).successful and start().success
    until(lambda:len(fixture.calls)>=1)
    assert len(fixture.goals)==2 and manager.index==1
    fixture.opening=100.
    until(lambda:len(fixture.goals)==4)
    fixture.opening=0.;old_calls=len(fixture.calls)
    time.sleep(1.2)
    assert manager.index==3 and len(fixture.calls)==old_calls # Still at previous red heading.
    fixture.pose_y=4.;fixture.yaw=0.
    until(lambda:len(fixture.calls)>old_calls)
    assert manager.state!='SUCCEEDED'
    fixture.opening=100.
    until(lambda:manager.state=='SUCCEEDED')
    assert [g.pose.pose.position.y for g in fixture.goals]==[1.,2.,3.,4.]


def test_runtime_single_blue_and_active_queue_cannot_be_replaced(system):
    manager,fixture,start,cancel=system
    fixture.hold=True
    c=config();c['points']=c['points'][:1];c['points'][0]['type']='blue'
    until(lambda:localized(manager))
    assert stage(fixture,c).successful and start().success
    until(lambda:manager.state=='NAVIGATING')
    assert not stage(fixture,config()).successful
    assert not start().success and len(fixture.goals)==1
    assert cancel().success
    until(lambda:manager.state=='CANCELED')
    assert not fixture.calls


def test_runtime_without_localization_does_not_send_goal_or_leave_staged_queue(system):
    manager,fixture,start,cancel=system
    fixture.observing=False
    time.sleep(1.2)
    assert stage(fixture,config()).successful
    reply=start()
    assert not reply.success and 'localization' in reply.message
    assert not fixture.goals
    assert manager.get_parameter('runtime_mission_json').value==''


def test_persisted_blue_red_task_uses_types_too(system):
    manager,fixture,start,cancel=system
    c=config();c['points'][0]['type']='blue';c['points'][1]['type']='red'
    Path(manager.get_parameter('mission_file').value).write_text(yaml.safe_dump(c))
    fixture.pose_y=4.;fixture.speed=0.;fixture.yaw=math.pi/2;fixture.opening=100.
    assert start().success
    until(lambda:manager.state=='SUCCEEDED')
    assert len(fixture.goals)==2 and len(fixture.calls)==1
    assert manager.task_mode=='patrol'


def test_actual_gui_bridge_parameter_then_trigger_path(system):
    manager,fixture,start,cancel=system
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
    from ros_bridge import RosBridge
    bridge=SimpleNamespace(start_lock=threading.Lock(),wait=RosBridge.wait,
        mission_parameters=fixture.create_client(SetParametersAtomically,'/patrol_manager/set_parameters_atomically'),
        trigger=lambda service:dict(success=start().success,message='fixture reply'))
    until(lambda:localized(manager))
    c=config();c['points']=c['points'][:1];c['points'][0]['type']='blue'
    assert RosBridge.start_patrol(bridge,c)['success']
    until(lambda:manager.state=='SUCCEEDED')
    assert len(fixture.goals)==1 and not fixture.calls and manager.task_mode=='realtime'
    fixture.destroy_client(bridge.mission_parameters)
