"""Check terminal navigation decisions without executing inspection or another goal."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from action_msgs.msg import GoalStatus
from scan_planner_msgs.action import NavigateTo

from patrol_manager.mission import PatrolPoint
from patrol_manager.patrol_node import PatrolManager


def node(home=False, cancel=False):
    return SimpleNamespace(
        state='NAVIGATING', reason='', index=0,
        mission=SimpleNamespace(points=(PatrolPoint('P1', (1.0, 2.0, 3.0), 1.0, home),)),
        navigation_feedback={}, cancel_requested=cancel,
        goal_future=object(), goal_handle=object(), result_future=object(),
        cancel_future=object(), publish_status=Mock(), send_goal=Mock(), inspect=Mock())


def response(status, code=NavigateTo.Result.ERROR_NONE, reason=''):
    return SimpleNamespace(
        status=status, result=NavigateTo.Result(error_code=code, error_reason=reason))


@pytest.mark.parametrize('home,expected', [(False, 'INSPECTING'), (True, 'SUCCEEDED')])
def test_success_enters_inspection_or_finishes_home(home, expected):
    n = node(home=home)
    PatrolManager.handle_result(n, response(GoalStatus.STATUS_SUCCEEDED))
    assert n.state == expected and n.index == 0
    assert n.goal_future is n.goal_handle is n.result_future is n.cancel_future is None
    n.inspect.assert_not_called()  # Inspection starts later in tick.
    n.send_goal.assert_not_called()
    n.publish_status.assert_called_once()


@pytest.mark.parametrize('status,code', [
    (GoalStatus.STATUS_ABORTED, NavigateTo.Result.ERROR_NO_PROGRESS),
    (GoalStatus.STATUS_SUCCEEDED, NavigateTo.Result.ERROR_GLOBAL_PLAN_FAILED),
    (GoalStatus.STATUS_ABORTED, NavigateTo.Result.ERROR_NONE),
])
def test_failure_never_advances_even_if_one_success_indicator_matches(status, code):
    n = node()
    PatrolManager.handle_result(n, response(status, code, 'test failure'))
    assert n.state == 'FAILED' and n.reason == 'test failure' and n.index == 0
    assert n.navigation_feedback['error_code'] == code
    n.send_goal.assert_not_called()


@pytest.mark.parametrize('cancel,status', [
    (False, GoalStatus.STATUS_CANCELED), (True, GoalStatus.STATUS_SUCCEEDED),
])
def test_cancellation_stops_sequence_even_when_navigation_finished_at_same_time(cancel, status):
    n = node(cancel=cancel)
    PatrolManager.handle_result(n, response(status))
    assert n.state == 'CANCELED'
    assert n.goal_handle is None
    n.inspect.assert_not_called()
    n.send_goal.assert_not_called()


@pytest.mark.parametrize('status,code', [
    (GoalStatus.STATUS_ABORTED, NavigateTo.Result.ERROR_STOP_FAILED),
    (GoalStatus.STATUS_CANCELED, NavigateTo.Result.ERROR_STOP_FAILED),
    (GoalStatus.STATUS_EXECUTING, NavigateTo.Result.ERROR_NONE),
    (GoalStatus.STATUS_UNKNOWN, NavigateTo.Result.ERROR_NONE),
])
def test_unconfirmed_stop_blocks_restart_and_retains_handles(status, code):
    n = node(cancel=True)
    old_handle = n.goal_handle
    PatrolManager.handle_result(n, response(status, code))
    assert n.state == 'BLOCKED'
    assert n.goal_handle is old_handle
    n.send_goal.assert_not_called()
