import math
import json
from pathlib import Path
import pytest
from patrol_manager.odom_motion import OdomMotion
from patrol_manager.door_gate import DoorGate
from patrol_manager.door_mission import parse_mission


def test_zero_twist_does_not_mean_stationary():
    m=OdomMotion()
    assert m.observe(1.,'odom',(0.,0.,0.),(0.,0.,0.),.5) is None
    for i in range(1,11):
        result=m.observe(1.+i*.1,'odom',(.02*i,0.,.04*i),(0.,0.,0.),.5)
        if i<10:assert result is None
    linear,_,angular=result
    assert linear==pytest.approx(.2) and angular==pytest.approx(.4)


def test_stop_still_requires_real_samples_and_preserves_reported_twist():
    m=OdomMotion()
    for i in range(11):m.observe(1.+i*.1,'odom',(0.,0.,0.),(0.,0.,0.),.5)
    assert m.observe(2.1,'odom',(0.,0.,0.),(.1,0.,-.2),.5)==(.1,0.,.2)
    assert m.observe(2.2,'odom',(0.,0.,0.),(0.,0.,0.),.5)==(0.,0.,0.)


def test_wrap_and_invalid_time_or_frame():
    m=OdomMotion()
    for i in range(11):
        yaw=math.pi-.01+i*.02
        yaw=math.atan2(math.sin(yaw),math.cos(yaw))
        result=m.observe(1.+i*.1,'odom',(0.,0.,yaw),(0.,0.,0.),.5)
    assert result[2]==pytest.approx(.2)
    assert m.observe(2.,'odom',(0.,0.,0.),(0.,0.,0.),.5) is None
    assert m.observe(4.,'odom',(0.,0.,0.),(0.,0.,0.),.5) is None
    assert m.observe(4.1,'new',(0.,0.,0.),(0.,0.,0.),.5) is None
    assert m.observe(4.2,'new',(float('nan'),0.,0.),(0.,0.,0.),.5) is None


def test_stationary_jitter_is_not_instantaneous_motion():
    m=OdomMotion()
    for i in range(100):
        jitter=.005*(-1 if i%2 else 1)
        result=m.observe(i*.1,'odom',(jitter,-jitter,jitter),(0.,0.,0.),.5)
        if i>=11:
            assert result is not None and result[0]<.02 and result[2]<.03


def test_real_movement_with_zero_twist_and_jitter_still_blocks():
    m=OdomMotion()
    for i in range(60):
        t=i*.1;jitter=.005*(-1 if i%2 else 1)
        result=m.observe(t,'odom',(.04*t+jitter,-.03*t-jitter,.06*t+jitter),(0.,0.,0.),.5)
        if i>=11:
            assert result[0]>.02 and result[2]>.03


def test_gap_requires_a_complete_new_window():
    m=OdomMotion()
    for i in range(11):m.observe(i*.1,'odom',(0.,0.,0.),(0.,0.,0.),.5)
    assert m.observe(3.,'odom',(0.,0.,0.),(0.,0.,0.),.5) is None
    assert m.observe(3.1,'odom',(0.,0.,0.),(0.,0.,0.),.5) is None
    for i in range(2,11):result=m.observe(3.+i*.1,'odom',(0.,0.,0.),(0.,0.,0.),.5)
    assert result==(0.,0.,0.)


def test_confirmed_stationary_real_capture_reaches_door_detection():
    rows=json.loads((Path(__file__).parent/'fixtures/stationary_20261028.json').read_text())['rows']
    mission=parse_mission({'frame_id':'map','points':[dict(name='red',type='red',
        position=[-7.161,-37.752,0.],yaw=0.)]})
    new=DoorGate(mission,navigation_succeeded=True)
    old=DoorGate(mission,navigation_succeeded=True)
    motion=OdomMotion();previous=None;new_ready=old_ready=False
    for r in rows:
        current=motion.observe(r['t'],'odom',r['pose'],r['twist'],.5)
        if current is not None:
            new.observe(r['t'],r['map_pose'],current)
            new_ready |= new.ready(r['t'])
        if previous is not None:
            dt=r['t']-previous['t']
            if 0<dt<=.5:
                speed=math.hypot(r['pose'][0]-previous['pose'][0],r['pose'][1]-previous['pose'][1])/dt
                angle=r['pose'][2]-previous['pose'][2]
                turn=abs(math.atan2(math.sin(angle),math.cos(angle)))/dt
                old.observe(r['t'],r['map_pose'],(speed,0.,turn))
                old_ready |= old.ready(r['t'])
        previous=r
    assert not old_ready # Reproduces the exact field failure.
    assert new_ready # The same data reaches the existing detector, no threshold change.
