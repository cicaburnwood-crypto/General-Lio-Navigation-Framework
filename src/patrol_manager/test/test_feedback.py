"""Check that Action feedback produces standard JSON for patrol status."""

import json
from types import SimpleNamespace
from unittest.mock import Mock

import pytest
from scan_planner_msgs.action import NavigateTo

from patrol_manager.patrol_node import PatrolManager


@pytest.mark.parametrize('distance,expected', [
    (0.0, 0.0), (1.2, 1.2), (float('nan'), None),
    (float('inf'), None), (-float('inf'), None),
])
def test_feedback_publishes_uuid_and_distances_as_json(distance, expected):
    msg = NavigateTo.Impl.FeedbackMessage()
    msg.goal_id.uuid = list(range(16))
    msg.feedback.phase = NavigateTo.Feedback.PHASE_EXECUTING
    msg.feedback.xy_distance = distance
    msg.feedback.z_distance = distance
    msg.feedback.remaining_path_length = distance
    msg.feedback.path_progress_valid = False
    msg.feedback.diagnostic_message = '测试反馈'
    node = SimpleNamespace(
        navigation_feedback={}, publisher=Mock(),
        state='NAVIGATING', reason='P1', index=0)
    PatrolManager.feedback(node, msg)
    PatrolManager.publish_status(node)
    text = node.publisher.publish.call_args.args[0].data
    parsed = json.loads(text)['navigation']
    assert parsed['task_id'] == bytes(range(16)).hex()
    assert parsed['phase'] == NavigateTo.Feedback.PHASE_EXECUTING
    assert parsed['xy_distance'] == expected
    assert parsed['z_distance'] == expected
    assert parsed['remaining_path_length'] == expected
    assert parsed['path_progress_valid'] is False
    assert parsed['diagnostic_message'] == '测试反馈'
    json.dumps(parsed, allow_nan=False)
