"""Check asynchronous submission gating without contacting navigation."""

from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from patrol_manager.patrol_node import PatrolManager


def node(ready=True):
    client = Mock()
    client.server_is_ready.return_value = ready
    return SimpleNamespace(
        client=client, goal_future=None, goal_handle=None,
        build_goal=Mock(return_value=object()), feedback=Mock())


def test_success_returns_true_and_preserves_the_acceptance_future():
    n = node()
    point = object()
    assert PatrolManager.send_goal(n, point)
    assert n.goal_future is n.client.send_goal_async.return_value
    n.build_goal.assert_called_once_with(point)
    n.client.send_goal_async.assert_called_once_with(
        n.build_goal.return_value, feedback_callback=n.feedback)
    assert not PatrolManager.send_goal(n, point)
    assert n.client.send_goal_async.call_count == 1


def test_unavailable_server_does_not_submit_or_construct_goal():
    n = node(ready=False)
    assert not PatrolManager.send_goal(n, object())
    assert n.goal_future is None
    n.client.send_goal_async.assert_not_called()
    n.build_goal.assert_not_called()


@pytest.mark.parametrize('field', ['goal_future', 'goal_handle'])
def test_existing_navigation_is_not_overwritten(field):
    n = node()
    existing = object()
    setattr(n, field, existing)
    assert not PatrolManager.send_goal(n, object())
    assert getattr(n, field) is existing
    n.client.send_goal_async.assert_not_called()
