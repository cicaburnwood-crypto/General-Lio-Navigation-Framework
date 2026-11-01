"""Load a small YAML patrol mission without ROS dependencies."""

from dataclasses import dataclass
import math
from pathlib import Path

import yaml


@dataclass(frozen=True)
class PatrolPoint:
    """Represent a position and optional placeholder inspection."""

    name: str
    position: tuple
    inspection_seconds: float
    is_home: bool = False


@dataclass(frozen=True)
class Mission:
    """Contain ordered patrol points and common navigation settings."""

    frame_id: str
    points: tuple
    xy_tolerance: float
    z_tolerance: float
    timeout_sec: float


def load_mission(filename):
    """Read and validate patrol points, navigation limits, and optional home."""
    data = yaml.safe_load(Path(filename).read_text())
    if not isinstance(data, dict):
        raise ValueError("Mission YAML must contain a mapping")
    if not isinstance(data.get("points"), list) or not data["points"]:
        raise ValueError("points must be a nonempty list")
    frame_id = data["frame_id"]
    if not isinstance(frame_id, str) or not frame_id.strip():
        raise ValueError("frame_id must be a nonempty string")

    limits = {}
    for key in ("xy_tolerance", "z_tolerance", "timeout_sec"):
        value = float(data["navigation"][key])
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be finite and positive")
        limits[key] = value

    def read_point(value, is_home=False):
        name = value["name"]
        position = tuple(float(v) for v in value["position"])
        if len(position) != 3 or not all(math.isfinite(v) for v in position):
            raise ValueError(f"{name}: position must contain three finite numbers")
        duration = 0.0 if is_home else float(value["inspection_duration_sec"])
        if not math.isfinite(duration) or duration < 0:
            raise ValueError(f"{name}: inspection_duration_sec must be finite and nonnegative")
        return PatrolPoint(
            name=name, position=position, inspection_seconds=duration, is_home=is_home)

    patrol_points = [read_point(value) for value in data["points"]]
    if data.get("home") is not None:
        patrol_points.append(read_point(data["home"], is_home=True))
    return Mission(frame_id=frame_id, points=tuple(patrol_points), **limits)


if __name__ == "__main__":
    filename = Path(__file__).resolve().parents[1] / "config" / "patrol.example.yaml"
    print("start loading mission ...")
    print(load_mission(filename))
