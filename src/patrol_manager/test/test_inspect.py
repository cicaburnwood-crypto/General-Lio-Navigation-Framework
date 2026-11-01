"""Placeholder inspection schedules a wait without dispatching navigation."""

from types import SimpleNamespace
from unittest.mock import Mock, patch

import pytest

from patrol_manager.mission import PatrolPoint
from patrol_manager.patrol_node import PatrolManager


@pytest.mark.parametrize('duration', [0.0, 1.5])
def test_inspection_sets_deadline_without_waiting_or_advancing(duration):
    node = SimpleNamespace(
        state='INSPECTING', reason='', index=0, inspection_deadline=None,
        publish_status=Mock(), send_goal=Mock())
    point = PatrolPoint('P1', (1.0, 2.0, 3.0), duration)
    with patch('patrol_manager.patrol_node.time.monotonic', return_value=100.0), \
            patch('patrol_manager.patrol_node.time.sleep') as sleep:
        PatrolManager.inspect(node, point)
    assert node.inspection_deadline == 100.0 + duration
    assert node.state == 'INSPECTING' and node.index == 0
    assert 'Placeholder' in node.reason and 'P1' in node.reason
    node.publish_status.assert_called_once()
    node.send_goal.assert_not_called()
    sleep.assert_not_called()
