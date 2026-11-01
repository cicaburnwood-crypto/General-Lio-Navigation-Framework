"""Use the original MPPI marker layout, including its odom->map transform."""
import math
from pathlib import Path
import sys
import threading
from types import MethodType, SimpleNamespace

import pytest
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

integration=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(integration if (integration/'ros_bridge.py').exists() else Path('/integration')))
from ros_bridge import RosBridge


def test_selected_mppi_trajectory_not_candidate_or_reference():
    t=TransformStamped()
    t.transform.translation.x=2.
    t.transform.rotation.z=math.sin(math.pi/4)
    t.transform.rotation.w=math.cos(math.pi/4)
    fake=SimpleNamespace(data={},lock=threading.RLock(),tf=SimpleNamespace(lookup_transform=lambda *args:t))
    fake.map_transform=MethodType(RosBridge.map_transform,fake)
    fake.store_path=MethodType(RosBridge.store_path,fake)
    msg=MarkerArray()
    for namespace,identifier,x in [('Candidate Trajectories',0,999.),('Optimal Trajectory',22,2.),('Optimal Trajectory',21,1.)]:
        marker=Marker();marker.header.frame_id='odom';marker.ns=namespace;marker.id=identifier
        marker.pose.position.x=x;marker.action=Marker.ADD
        msg.markers.append(marker)
    RosBridge.on_trajectory(fake,msg)
    assert len(fake.data['local_path'])==2
    assert fake.data['local_path'][0]==pytest.approx([2.,1.,0.])
    assert fake.data['local_path'][1]==pytest.approx([2.,2.,0.])
    assert fake.data['local_revision']==1
    RosBridge.on_trajectory(fake,MarkerArray())
    assert fake.data['local_path']==[]
