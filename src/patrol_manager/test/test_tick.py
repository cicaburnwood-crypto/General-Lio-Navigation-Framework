"""Exercise complete patrol sequences with controllable Action replies."""

from concurrent.futures import Future
from types import MethodType, SimpleNamespace
from unittest.mock import Mock, patch

import pytest
from action_msgs.msg import GoalStatus
from scan_planner_msgs.action import NavigateTo
from std_srvs.srv import Trigger

from patrol_manager.mission import PatrolPoint
from patrol_manager.patrol_node import PatrolManager


def node(home=False):
    points = [PatrolPoint('P1', (1., 2., 3.), 1.), PatrolPoint('P2', (4., 5., 6.), 1.)]
    if home:
        points.append(PatrolPoint('Home', (0., 0., 0.), 0., True))
    n = SimpleNamespace(
        state='WAITING_FOR_SERVER', reason='', index=0,
        mission=SimpleNamespace(points=tuple(points)), client=Mock(),
        goal_future=None, goal_handle=None, result_future=None, cancel_future=None,
        cancel_requested=False, navigation_feedback={}, inspection_deadline=None,
        build_goal=lambda point: point, feedback=Mock(), publish_status=Mock())
    for method in ('send_goal', 'handle_result', 'inspect', 'tick', 'cancel'):
        setattr(n, method, MethodType(getattr(PatrolManager, method), n))
    n.client.server_is_ready.return_value = True
    n.client.send_goal_async.side_effect = lambda *args, **kwargs: Future()
    return n


def accept(n, accepted=True):
    handle = Mock(accepted=accepted)
    handle.get_result_async.return_value = Future()
    handle.cancel_goal_async.return_value = Future()
    n.goal_future.set_result(handle)
    n.tick()
    return handle


def finish(n, status=GoalStatus.STATUS_SUCCEEDED, code=NavigateTo.Result.ERROR_NONE):
    n.result_future.set_result(SimpleNamespace(
        status=status, result=NavigateTo.Result(error_code=code)))
    n.tick()


@pytest.mark.parametrize('home', [False, True])
def test_sequence_waits_for_result_and_inspection_before_next_point(home):
    n = node(home)
    with patch('patrol_manager.patrol_node.time.monotonic') as clock:
        for index, point in enumerate(n.mission.points):
            clock.return_value = index * 10.
            n.tick()
            assert n.state == 'WAITING_FOR_ACCEPTANCE'
            n.tick()  # No acceptance yet: don't resubmit.
            assert n.client.send_goal_async.call_count == index + 1
            accept(n)
            assert n.state == 'NAVIGATING' and n.index == index
            n.tick()  # No final result yet: don't advance.
            assert n.index == index
            finish(n)
            if point.is_home:
                assert n.state == 'SUCCEEDED'
                break
            assert n.state == 'INSPECTING'
            n.tick()  # Start inspection once.
            deadline = n.inspection_deadline
            clock.return_value += .5
            n.tick()
            assert n.inspection_deadline == deadline and n.index == index
            clock.return_value += .5
            n.tick()
            assert n.inspection_deadline is None
        assert n.state == 'SUCCEEDED'
        n.tick()
        assert n.client.send_goal_async.call_count == len(n.mission.points)


@pytest.mark.parametrize('state', ['WAITING_FOR_SERVER', 'INSPECTING'])
def test_cancel_without_navigation_ends_queue_immediately(state):
    n = node()
    n.state = state
    reply = n.cancel(Trigger.Request(), Trigger.Response())
    assert reply.success and n.state == 'CANCELED'
    n.tick()
    n.client.send_goal_async.assert_not_called()


def test_cancel_pending_acceptance_waits_then_cancels_once_and_waits_for_result():
    n = node()
    n.tick()
    n.cancel(Trigger.Request(), Trigger.Response())
    n.tick()
    assert n.state == 'WAITING_FOR_ACCEPTANCE'
    handle = accept(n)
    assert n.state == 'CANCELING'
    n.tick()
    handle.cancel_goal_async.return_value.set_result(SimpleNamespace(goals_canceling=[object()]))
    n.tick()
    n.tick()
    assert n.state == 'CANCELING'  # Cancel reply is not completion.
    handle.cancel_goal_async.assert_called_once()
    finish(n, GoalStatus.STATUS_CANCELED)
    assert n.state == 'CANCELED' and n.index == 0
    n.tick()
    assert n.client.send_goal_async.call_count == 1


def test_cancel_rejected_still_waits_for_final_result_and_does_not_advance():
    n = node()
    n.tick()
    handle = accept(n)
    n.cancel(Trigger.Request(), Trigger.Response())
    n.tick()
    handle.cancel_goal_async.return_value.set_result(SimpleNamespace(goals_canceling=[]))
    n.tick()
    assert n.state == 'CANCELING' and 'not accepted' in n.reason
    finish(n)  # Navigation can finish successfully before cancellation takes effect.
    assert n.state == 'CANCELED'
    n.tick()
    assert n.client.send_goal_async.call_count == 1


@pytest.mark.parametrize('cancel,expected', [(False, 'FAILED'), (True, 'CANCELED')])
def test_goal_rejection_stops_queue(cancel, expected):
    n = node()
    n.tick()
    if cancel:
        n.cancel(Trigger.Request(), Trigger.Response())
    accept(n, accepted=False)
    assert n.state == expected and n.goal_handle is None
    n.tick()
    assert n.client.send_goal_async.call_count == 1


@pytest.mark.parametrize('code,expected', [
    (NavigateTo.Result.ERROR_LOCAL_PLAN_FAILED, 'FAILED'),
    (NavigateTo.Result.ERROR_STOP_FAILED, 'BLOCKED'),
])
def test_navigation_failure_never_sends_next_goal(code, expected):
    n = node()
    n.tick()
    accept(n)
    finish(n, GoalStatus.STATUS_ABORTED, code)
    assert n.state == expected
    n.tick()
    assert n.client.send_goal_async.call_count == 1
