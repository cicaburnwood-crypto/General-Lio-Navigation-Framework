"""Check start-state gating and atomic mission loading without ROS navigation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
import yaml
from std_srvs.srv import Trigger

from patrol_manager.patrol_node import PatrolManager


def node(filename, state='IDLE'):
    return SimpleNamespace(
        state=state, reason='previous', mission=object(), index=2,
        goal_future=object(), goal_handle=object(), result_future=object(),
        cancel_future=object(), cancel_requested=True,
        navigation_feedback={'phase': 4}, publish_status=Mock(),
        get_parameter=lambda _: SimpleNamespace(value=str(filename)))


def mission_file(tmp_path):
    filename = tmp_path / 'mission.yaml'
    filename.write_text(yaml.safe_dump({
        'frame_id': 'world',
        'navigation': {'xy_tolerance': 0.3, 'z_tolerance': 0.3, 'timeout_sec': 120},
        'points': [{'name': 'P1', 'position': [1, 2, 3], 'inspection_duration_sec': 1}],
    }))
    return filename


@pytest.mark.parametrize('state', ['IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED'])
def test_start_reloads_and_resets_only_after_valid_loading(tmp_path, state):
    n = node(mission_file(tmp_path), state)
    response = PatrolManager.start(n, Trigger.Request(), Trigger.Response())
    assert response.success
    assert n.state == 'WAITING_FOR_SERVER' and n.index == 0
    assert n.mission.points[0].name == 'P1'
    assert n.goal_future is n.goal_handle is n.result_future is n.cancel_future is None
    assert not n.cancel_requested and n.navigation_feedback == {}
    n.publish_status.assert_called_once()


@pytest.mark.parametrize('state', [
    'WAITING_FOR_SERVER', 'NAVIGATING', 'RETURNING', 'INSPECTING', 'CANCELING', 'BLOCKED',
])
def test_active_and_blocked_states_reject_before_loading(state):
    n = node('/does/not/exist.yaml', state)
    old_mission = n.mission
    response = PatrolManager.start(n, Trigger.Request(), Trigger.Response())
    assert not response.success
    assert 'active or navigation stop unconfirmed' in response.message
    assert n.state == state and n.mission is old_mission and n.index == 2
    n.publish_status.assert_not_called()


@pytest.mark.parametrize('contents', [None, '', 'points: []'])
def test_loading_failure_preserves_previous_state(tmp_path, contents):
    filename = tmp_path / 'invalid.yaml'
    if contents is not None:
        filename.write_text(contents)
    n = node(filename)
    previous = dict(vars(n))
    response = PatrolManager.start(n, Trigger.Request(), Trigger.Response())
    assert not response.success and 'Invalid mission' in response.message
    assert vars(n) == previous


def test_missing_mission_parameter_is_rejected():
    n = node('')
    response = PatrolManager.start(n, Trigger.Request(), Trigger.Response())
    assert not response.success and 'mission_file' in response.message
    assert n.state == 'IDLE'


def test_duplicate_start_does_not_replace_loaded_mission(tmp_path):
    n = node(mission_file(tmp_path))
    assert PatrolManager.start(n, Trigger.Request(), Trigger.Response()).success
    loaded = n.mission
    assert not PatrolManager.start(n, Trigger.Request(), Trigger.Response()).success
    assert n.mission is loaded and n.state == 'WAITING_FOR_SERVER'
