"""Door patrol schema. No ROS dependency and no guessed real-world goals."""
from dataclasses import dataclass
import math
from pathlib import Path
import yaml


@dataclass(frozen=True)
class DoorPoint:
    name: str
    position: tuple
    yaw: float
    inspection_seconds: float = 0.0
    type: str = 'blue'


@dataclass(frozen=True)
class DoorMission:
    frame_id: str
    points: tuple
    settle_seconds: float
    linear_stopped_mps: float
    angular_stopped_radps: float
    observation_max_age_sec: float
    xy_tolerance: float
    yaw_tolerance: float
    timeout_sec: float


def parse_mission(data):
    if not isinstance(data, dict) or data.get('frame_id') != 'map':
        raise ValueError('Nav2 door mission frame_id must be map')
    if not isinstance(data.get('points'), list) or not data['points']:
        raise ValueError('Configure at least one waypoint')
    points = []
    for index, p in enumerate(data['points']):
        if not isinstance(p, dict):
            raise ValueError('Each waypoint must be an object')
        # Legacy missions had exactly one implicit door at index zero.
        kind = p.get('type', 'red' if index == 0 else 'blue')
        if kind not in ('red', 'blue'):
            raise ValueError('Waypoint type must be red or blue')
        xyz = p.get('position', [])
        if len(xyz) != 3 or any(v is None for v in xyz) or p.get('yaw') is None:
            raise ValueError('Configure XYZ and yaw for every point before starting')
        values = tuple(float(v) for v in xyz) + (float(p['yaw']),)
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Point coordinates and yaw must be finite')
        seconds = float(p.get('inspection_duration_sec', 0))
        if not math.isfinite(seconds) or seconds < 0:
            raise ValueError('Inspection duration must be nonnegative')
        if not isinstance(p.get('name'), str) or not p['name'].strip():
            raise ValueError('Point name is required')
        points.append(DoorPoint(p['name'], values[:3], values[3], seconds, kind))
    settings = dict(settle_seconds=1.0, linear_stopped_mps=.02,
                    angular_stopped_radps=.03, observation_max_age_sec=.5,
                    xy_tolerance=.05, yaw_tolerance=.087, timeout_sec=120.0)
    settings.update({k: v for k, v in data.get('navigation', {}).items() if k in settings})
    settings.update({k: v for k, v in data.get('door', {}).items() if k in settings})
    settings = {k: float(v) for k, v in settings.items()}
    if not all(math.isfinite(v) and v > 0 for v in settings.values()):
        raise ValueError('Tolerances, durations and timeouts must be positive')
    return DoorMission('map', tuple(points), **settings)


def load_mission(path):
    return parse_mission(yaml.safe_load(Path(path).read_text()))
