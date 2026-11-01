import copy
import math
import pytest
from patrol_manager.door_mission import parse_mission
from patrol_manager.door_gate import DoorGate


def config():
    return {'frame_id':'map','points':[
        {'name':'door','position':[1.,2.,0.],'yaw':math.pi/2},
        {'name':'after','position':[1.,4.,0.],'yaw':math.pi/2}]}


def settled():
    g=DoorGate(parse_mission(config()))
    for i in range(12):
        g.observe(i*.1,(1.,2.,math.pi/2),(0.,0.,0.))
    return g


def test_requires_heading_and_actual_stop():
    g=DoorGate(parse_mission(config()))
    for i in range(30):g.observe(i*.1,(1.,2.,0.),(0.,0.,0.))
    assert not g.ready(2.9)
    for i in range(30,60):g.observe(i*.1,(1.,2.,math.pi/2),(.1,0.,0.))
    assert not g.ready(5.9)


def test_not_timer_only():
    g=DoorGate(parse_mission(config()))
    g.observe(0,(1.,2.,math.pi/2),(0.,0.,0.))
    assert not g.ready(10)


def test_no_overlap_and_one_second_spacing():
    g=settled()
    assert g.begin(1.1)
    assert not g.begin(1.2)
    assert not g.finish(1.2,0.)
    assert not g.begin(1.5)
    for i in range(13,23):g.observe(i*.1,(1.,2.,math.pi/2),(0.,0.,0.))
    assert g.begin(2.2)


@pytest.mark.parametrize('value',[0,50,99.9,float('nan'),float('inf'),-1,101])
def test_only_exact_hundred_releases(value):
    g=settled();assert g.begin(1.1)
    assert not g.finish(1.2,value)


def test_hundred_releases_when_still():
    g=settled();assert g.begin(1.1)
    assert g.finish(1.2,100.)


def test_door_success_overrides_motion_during_capture():
    g=settled();assert g.begin(1.1)
    g.observe(1.2,(1.,2.,math.pi/2),(0.,.2,0.))
    for i in range(13,26):g.observe(i*.1,(1.,2.,math.pi/2),(0.,0.,0.))
    assert g.finish(2.5,100.)


def test_door_success_overrides_stale_pose():
    g=settled();assert g.begin(1.1)
    assert g.finish(3,100.)


def test_door_success_overrides_heading_boundary_and_invalidated_flag():
    g=settled();assert g.begin(1.1)
    g.observe(1.2,(1.,2.,math.pi/2+.101),(0.,0.,0.))
    assert g.invalidated
    assert g.finish(1.3,100.)
    assert not g.finish(1.4,100.) # No duplicate release without a new request.


def test_success_without_current_request_does_not_release():
    assert not settled().finish(1.2,100.)


def test_incomplete_coordinates_and_invalid_numeric_rejected():
    for value in (None,float('nan'),float('inf')):
        c=config();c['points'][0]['position'][0]=value
        with pytest.raises((ValueError,TypeError)):parse_mission(c)
    c=config();c['points']=[]
    with pytest.raises(ValueError):parse_mission(c)


def test_types_legacy_single_point_and_current_red_gate():
    legacy=parse_mission(config())
    assert [p.type for p in legacy.points]==['red','blue']
    c=config();c['points'][0]['type']='blue';c['points'][1]['type']='red'
    mission=parse_mission(c)
    assert [p.type for p in mission.points]==['blue','red']
    g=DoorGate(mission,1)
    for i in range(12):g.observe(i*.1,(1.,2.,math.pi/2),(0.,0.,0.))
    assert not g.ready(1.1) # Must not check the first (blue) waypoint.
    for i in range(12,25):g.observe(i*.1,(1.,4.,math.pi/2),(0.,0.,0.))
    assert g.ready(2.4)
    c['points']=c['points'][:1]
    assert len(parse_mission(c).points)==1
    c['points'][0]['type']='green'
    with pytest.raises(ValueError):parse_mission(c)


def test_yaw_wrap():
    c=config();c['points'][0]['yaw']=math.pi
    g=DoorGate(parse_mission(c))
    for i in range(12):g.observe(i*.1,(1.,2.,-math.pi),(0.,0.,0.))
    assert g.ready(1.1)


def test_nav2_success_latches_xy_before_first_observation():
    g=DoorGate(parse_mission(config()),navigation_succeeded=True)
    # Reproduce the real red point: Nav2 succeeded, then map pose is 10 cm away.
    for i in range(12):g.observe(i*.1,(1.1,2.,math.pi/2),(0.,0.,0.))
    assert g.position_reached and g.begin(1.1)
    g.observe(1.2,(1.12,2.,math.pi/2),(0.,0.,0.))
    assert g.finish(1.2,100.)


def test_xy_latches_independently_of_heading_and_stop():
    g=DoorGate(parse_mission(config()))
    g.observe(0.,(1.,2.,0.),(.1,0.,0.))
    assert g.position_reached and not g.ready(0.)
    for i in range(1,15):g.observe(i*.1,(1.1,2.,math.pi/2),(0.,0.,0.))
    assert g.ready(1.4)


def test_position_latch_does_not_bypass_heading_motion_or_freshness():
    g=DoorGate(parse_mission(config()),navigation_succeeded=True)
    for i in range(12):g.observe(i*.1,(1.1,2.,0.),(0.,0.,0.))
    assert not g.ready(1.1)
    for i in range(12,25):g.observe(i*.1,(1.1,2.,math.pi/2),(.1,0.,0.))
    assert not g.ready(2.4)
    for i in range(25,39):g.observe(i*.1,(1.1,2.,math.pi/2),(0.,0.,0.))
    assert g.ready(3.8)
    assert not g.ready(5.)
    assert g.position_reached


def test_new_gate_does_not_inherit_previous_point_position_latch():
    mission=parse_mission(config())
    previous=DoorGate(mission,navigation_succeeded=True)
    current=DoorGate(mission,1)
    assert previous.position_reached and not current.position_reached
    for i in range(12):current.observe(i*.1,(1.,2.,math.pi/2),(0.,0.,0.))
    assert not current.ready(1.1)
