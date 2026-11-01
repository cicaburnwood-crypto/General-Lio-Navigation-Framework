"""Validate the mission-file contract without starting ROS or navigation."""

from copy import deepcopy

import pytest
import yaml

from patrol_manager.mission import Mission, load_mission


def example():
    return {
        "frame_id": "world",
        "navigation": {"xy_tolerance": 0.3, "z_tolerance": 0.3, "timeout_sec": 120},
        "points": [
            {"name": "P1", "position": [1, 2, 3], "inspection_duration_sec": 1},
            {"name": "P2", "position": [4, 5, 6], "inspection_duration_sec": 0},
        ],
        "home": {"name": "Home", "position": [0, 0, 0]},
    }


def read(tmp_path, data):
    filename = tmp_path / "mission.yaml"
    filename.write_text(yaml.safe_dump(data))
    return load_mission(filename)


def test_valid_mission_has_independent_points_and_float_values(tmp_path):
    mission = read(tmp_path, example())
    assert isinstance(mission, Mission)
    assert [p.name for p in mission.points] == ["P1", "P2", "Home"]
    assert len({id(p) for p in mission.points}) == 3
    assert all(type(v) is float for p in mission.points for v in p.position)
    assert [p.inspection_seconds for p in mission.points] == [1.0, 0.0, 0.0]
    assert [p.is_home for p in mission.points] == [False, False, True]
    assert type(mission.timeout_sec) is float


@pytest.mark.parametrize("home", ["missing", None])
def test_home_is_optional(tmp_path, home):
    data = example()
    if home == "missing":
        del data["home"]
    else:
        data["home"] = None
    assert len(read(tmp_path, data).points) == 2


def test_empty_points_are_rejected_even_with_home(tmp_path):
    data = example()
    data["points"] = []
    with pytest.raises(ValueError, match="nonempty"):
        read(tmp_path, data)


@pytest.mark.parametrize("position", [
    [1, 2], [1, 2, 3, 4], [float("nan"), 0, 0], [0, float("inf"), 0],
])
@pytest.mark.parametrize("target", ["point", "home"])
def test_positions_require_three_finite_numbers(tmp_path, position, target):
    data = example()
    value = data["points"][0] if target == "point" else data["home"]
    value["position"] = position
    with pytest.raises(ValueError, match="three finite"):
        read(tmp_path, data)


@pytest.mark.parametrize("key", ["xy_tolerance", "z_tolerance", "timeout_sec"])
@pytest.mark.parametrize("value", [0, -1, float("nan"), float("inf")])
def test_navigation_limits_must_be_positive_and_finite(tmp_path, key, value):
    data = example()
    data["navigation"][key] = value
    with pytest.raises(ValueError, match=key):
        read(tmp_path, data)


@pytest.mark.parametrize("duration", [-1, float("nan"), float("inf")])
def test_inspection_duration_must_be_nonnegative_and_finite(tmp_path, duration):
    data = example()
    data["points"][0]["inspection_duration_sec"] = duration
    with pytest.raises(ValueError, match="inspection_duration_sec"):
        read(tmp_path, data)


def test_loading_another_file_does_not_mutate_previous_mission(tmp_path):
    data = example()
    first = read(tmp_path, data)
    second_data = deepcopy(data)
    second_data["points"][0]["name"] = "New"
    second = read(tmp_path, second_data)
    assert first.points[0].name == "P1"
    assert second.points[0].name == "New"
