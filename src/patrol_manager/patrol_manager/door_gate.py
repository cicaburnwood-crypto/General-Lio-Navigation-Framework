"""Stop-and-face gate driven by measured odometry, not a fixed sleep."""
import math


class DoorGate:
    def __init__(self, mission, point_index=0, *, navigation_succeeded=False):
        self.mission = mission
        self.point_index = point_index
        # Nav2's stateful goal checker already latched XY before returning
        # SUCCEEDED. Preserve that result across turn-in-place/localization
        # drift; do not require a second XY arrival after navigation stopped.
        self.position_reached = bool(navigation_succeeded)
        self.stationary_since = None
        self.last_observation = None
        self.last_call = -math.inf
        self.in_flight = False
        self.invalidated = False
        self.calls = 0
        self.last_opening = None

    def observe(self, now, pose, velocity):
        p = self.mission.points[self.point_index]
        finite = all(math.isfinite(v) for v in (*pose, *velocity))
        if finite and math.hypot(pose[0]-p.position[0], pose[1]-p.position[1]) <= self.mission.xy_tolerance:
            self.position_reached = True
        correct = (finite and self.position_reached and
                   abs(math.atan2(math.sin(pose[2]-p.yaw), math.cos(pose[2]-p.yaw)))
                   <= self.mission.yaw_tolerance and
                   math.hypot(velocity[0], velocity[1]) <= self.mission.linear_stopped_mps and
                   abs(velocity[2]) <= self.mission.angular_stopped_radps)
        gap = self.last_observation is not None and now-self.last_observation > self.mission.observation_max_age_sec
        self.last_observation = now
        if not correct or gap:
            self.stationary_since = None
            if self.in_flight:
                self.invalidated = True
        if correct and self.stationary_since is None:
            self.stationary_since = now

    def settled(self, now):
        fresh = self.last_observation is not None and now-self.last_observation <= self.mission.observation_max_age_sec
        if not fresh:
            self.stationary_since = None
            if self.in_flight:
                self.invalidated = True
        return fresh and self.stationary_since is not None and now-self.stationary_since >= self.mission.settle_seconds

    def ready(self, now):
        return self.settled(now) and not self.in_flight and now-self.last_call >= 1.0

    def begin(self, now):
        if not self.ready(now):
            return False
        self.in_flight = True
        self.invalidated = False
        self.last_call = now
        self.calls += 1
        return True

    def finish(self, now, opening):
        # A successful response for this point's outstanding request is final.
        # Heading, motion, or observation-age changes during inference must not
        # veto it. Task cancellation is handled by the manager before finish().
        valid = self.in_flight
        self.in_flight = False
        self.last_opening = opening if math.isfinite(opening) else None
        return valid and opening == 100.0
