"""Sample up to five fresh frames and decide using their mean probability."""
from dataclasses import dataclass
from collections import deque
import math
from typing import Any, List, Optional, Sequence

SAMPLE_COUNT = 5
WINDOW_SECONDS = 1.0
WINDOW_NS = 1_000_000_000


@dataclass(frozen=True)
class Sample:
    stamp_ns: int
    image: Any


class SamplingWindow:
    """Select one to five fresh acquisitions across the request window.

    A request creates a new window. Nothing from a previous request is reused.
    The caller holds its own lock while accessing this object.
    """
    def __init__(self, start_monotonic: float, start_ros_ns: int, max_age_ns: int):
        self.start_monotonic = start_monotonic
        self.start_ros_ns = start_ros_ns
        self.max_age_ns = max_age_ns
        self.samples = deque(maxlen=60)
        self.latest_stamp_ns = 0

    def offer(self, image: Any, stamp_ns: int, received_monotonic: float, now_ros_ns: int) -> bool:
        elapsed = received_monotonic - self.start_monotonic
        source_elapsed = stamp_ns - self.start_ros_ns
        age = now_ros_ns - stamp_ns
        if not 0 <= elapsed < WINDOW_SECONDS:
            return False
        if stamp_ns <= 0 or not 0 <= source_elapsed < WINDOW_NS:
            return False
        if not 0 <= age <= self.max_age_ns or stamp_ns <= self.latest_stamp_ns:
            return False
        self.samples.append(Sample(stamp_ns, image))
        self.latest_stamp_ns = stamp_ns
        return True

    def complete(self) -> bool:
        return bool(self.samples)

    def selected(self) -> List[Sample]:
        samples=list(self.samples)
        if len(samples)<=SAMPLE_COUNT:return samples
        return [samples[round(i*(len(samples)-1)/(SAMPLE_COUNT-1))] for i in range(SAMPLE_COUNT)]


def opening_percent(probabilities: Sequence[float], threshold: float) -> float:
    """Return 100 only if the mean of 1..5 valid scores exceeds threshold."""
    if not 1 <= len(probabilities) <= SAMPLE_COUNT or not 0 < threshold < 1:
        return 0.0
    if any(not math.isfinite(p) or not 0 <= p <= 1 for p in probabilities):
        return 0.0
    return 100.0 if math.fsum(probabilities) / len(probabilities) > threshold else 0.0
