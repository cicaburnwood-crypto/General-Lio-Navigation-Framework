"""Original localization services; fake matches only, isolated ROS domain."""
import os
import threading
import time

import pytest
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from chassis_localization_msgs.srv import Relocalize, IsValid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from test_ros_bridge import RosBridge
from test_ros_workflow import Fixtures, until


@pytest.fixture
def localization_system():
    assert os.environ['ROS_DOMAIN_ID']=='192' and os.environ['ROS_LOCALHOST_ONLY']=='1'
    state=dict(valid=False,accept=True,requests=[],codes=[],restarts=0,stream=False)
    def restart():
        state['restarts'] += 1
        if state.get('restart_error'):
            raise RuntimeError('restart failed')
        state['stream'] = True
        return dict(previous_pid=100,pid=101)
    bridge=RosBridge('/camera/test/image', '/maps/selected/map.pcd', fastlio_restart=restart)
    fixture=Fixtures()
    odom=fixture.create_publisher(Odometry,'/Odometry',10)
    cloud=fixture.create_publisher(PointCloud2,'/sensors/cloud',10)
    def observe_lio():
        if not state['stream']:
            return
        stamp=fixture.get_clock().now().to_msg()
        if state.get('old_stamps'):
            stamp.sec -= 60
        o=Odometry();o.header.stamp=stamp;o.pose.pose.orientation.w=1.
        c=PointCloud2(width=1,height=1,point_step=12,row_step=12,data=bytes(12))
        c.header.stamp=stamp
        odom.publish(o);cloud.publish(c)
    fixture.create_timer(.05,observe_lio)
    def relocate(request,response):
        assert state['restarts'] == 1
        assert bridge.lio_odom_stamp > 0 and bridge.lio_cloud_stamp > 0
        state['requests'].append(request)
        state['valid']=False
        response.success=state['accept']
        response.message='accepted' if state['accept'] else 'map load failed'
        return response
    def validate(request,response):
        state['codes'].append(request.code)
        response.valid=state['valid']
        return response
    fixture.create_service(Relocalize,'/localizer/relocalize',relocate)
    fixture.create_service(IsValid,'/localizer/relocalize_check',validate)
    log=fixture.create_publisher(Log,'/rosout',10)
    executor=MultiThreadedExecutor(num_threads=4)
    executor.add_node(fixture)
    thread=threading.Thread(target=executor.spin);thread.start()
    try:
        until(lambda:bridge.relocalizer.service_is_ready() and bridge.localization_validator.service_is_ready())
        yield bridge,fixture,state,log
    finally:
        fixture.hold=False
        executor.shutdown();thread.join(timeout=5);fixture.destroy_node();bridge.close()


def test_reference_requires_native_match_and_does_not_become_robot_pose(localization_system):
    bridge,fixture,state,log=localization_system
    until(lambda:bridge.snapshot()['localization']['native_valid'] is False)
    reply=bridge.initial_pose(dict(x=10.,y=20.,z=.1,yaw=.8))
    assert reply['success'] and '尚不代表定位成功' in reply['message']
    request=state['requests'][-1]
    assert request.pcd_path=='/maps/selected/map.pcd'
    assert [request.x,request.y,request.z,request.yaw]==pytest.approx([10,20,.1,.8])
    assert request.pitch==request.roll==0
    time.sleep(.7)
    d=bridge.snapshot()
    assert d['localization']['state']=='MATCHING' and d['pose'] is None
    assert not fixture.goals # No navigation or motion dispatched.
    log.publish(Log(name='localizer.chassis_3d_localizer_node',
        msg='[align] seq=1 rough_rms=0.250 refine_rms=-1.000 jump_t=0.0 decision=not_converged_refine'))
    until(lambda:bridge.snapshot()['localization'].get('alignment',{}).get('decision')=='not_converged_refine')
    state['valid']=True
    until(lambda:bridge.snapshot()['localization']['state']=='LOCALIZED' and bridge.snapshot().get('pose') is not None)
    d=bridge.snapshot()
    assert d['pose'][:2]==[1.,2.] # Fixture's matched TF, never the supplied 10,20.
    assert state['codes'] and set(state['codes'])=={0} # Never use the code=1 bypass.
    assert not fixture.goals


def test_old_tf_is_not_a_current_localization_result(localization_system):
    bridge,fixture,state,log=localization_system
    state['valid']=True
    until(lambda:bridge.snapshot().get('pose') is not None)
    fixture.observing=False
    until(lambda:bridge.snapshot().get('pose') is None)
    assert bridge.snapshot()['localization']['pose_fresh'] is False


def test_rejected_seed_and_active_task_are_not_reported_as_localized(localization_system):
    bridge,fixture,state,log=localization_system
    with bridge.lock:bridge.data['patrol_status']=dict(state='NAVIGATING')
    with pytest.raises(ValueError,match='Cancel'):
        bridge.initial_pose(dict(x=10,y=20,z=0,yaw=0))
    assert not state['requests']
    with bridge.lock:bridge.data['patrol_status']=dict(state='IDLE')
    with pytest.raises(ValueError,match='finite'):
        bridge.initial_pose(dict(x=float('nan')))
    assert not state['requests']
    state['accept']=False
    with pytest.raises(ValueError,match='map load failed'):
        bridge.initial_pose(dict(x=10,y=20,z=0,yaw=0))
    time.sleep(.6)
    d=bridge.snapshot()
    assert d['localization']['state']=='ERROR' and d['pose'] is None
    assert len(state['requests'])==1 and not fixture.goals


def test_restart_failure_never_calls_native_matching(localization_system):
    bridge,fixture,state,log=localization_system
    state['restart_error']=True
    with pytest.raises(RuntimeError,match='restart failed'):
        bridge.initial_pose(dict(x=1,y=2,z=0,yaw=0))
    assert state['restarts']==1 and not state['requests']
    assert bridge.snapshot()['localization']['state']=='ERROR'
    assert bridge.snapshot()['pose'] is None and not fixture.goals


def test_queued_old_session_messages_do_not_start_matching(localization_system):
    bridge,fixture,state,log=localization_system
    state['old_stamps']=True
    results=[]
    thread=threading.Thread(target=lambda:results.append(bridge.initial_pose(dict(x=1,y=2))))
    thread.start()
    try:
        until(lambda:bridge.snapshot()['localization']['state']=='WAITING_LIO')
        time.sleep(.8)
        assert not state['requests'] and not results
        assert bridge.snapshot()['pose'] is None
    finally:
        state['old_stamps']=False
        thread.join(timeout=10)
    assert not thread.is_alive() and results[0]['success']
    assert len(state['requests'])==1 and not fixture.goals
