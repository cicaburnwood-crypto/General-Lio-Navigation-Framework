"""Estimate motion over one second, without amplifying FAST-LIO pose jitter."""
from collections import deque
import math


class OdomMotion:
    def __init__(self):
        self.previous = None
        self.samples = deque()
        self.window_seconds = 1.0

    def observe(self, stamp, frame, pose, twist, max_gap):
        previous = self.previous
        if not all(math.isfinite(v) for v in (stamp, *pose, *twist)):
            self.previous = None
            self.samples.clear()
            return None
        self.previous = (stamp, frame, pose)
        if previous is None or frame != previous[1] or not 0 < stamp-previous[0] <= max_gap:
            self.samples.clear()
            self.samples.append((stamp, *pose))
            return None
        angle = pose[2]-previous[2][2]
        yaw = self.samples[-1][3]+math.atan2(math.sin(angle), math.cos(angle))
        self.samples.append((stamp, pose[0], pose[1], yaw))
        # Keep the sample bracketing the one-second boundary. Require an actual
        # full window; a new frame, timestamp reset or data gap must warm up again.
        while len(self.samples) > 2 and stamp-self.samples[1][0] >= self.window_seconds:
            self.samples.popleft()
        if stamp-self.samples[0][0] < self.window_seconds or len(self.samples) < 3:
            return None
        times = [s[0]-stamp for s in self.samples]
        mean_t = sum(times)/len(times)
        centered = [t-mean_t for t in times]
        denom = sum(t*t for t in centered)
        slopes = []
        for axis in (1, 2, 3):
            # Subtract a local origin for numerical stability at large map coords.
            origin = self.samples[0][axis]
            slopes.append(sum(t*(s[axis]-origin) for t,s in zip(centered,self.samples))/denom)
        speed = math.hypot(slopes[0],slopes[1])
        turn = abs(slopes[2])
        # Neither a zero-filled twist nor an unchanged pose can erase motion
        # reported by the other measurement. A reported nonzero twist is still
        # taken immediately, without smoothing. Never use commanded velocity.
        return max(speed, math.hypot(twist[0], twist[1])), 0.0, max(turn, abs(twist[2]))
