"""Evidence-based triggers. Wheel and steering error namespaces stay separate."""
import time

class Detector:
    def __init__(self, cfg, trigger, now=None):
        self.cfg, self.trigger = cfg, trigger
        self.started = time.monotonic() if now is None else now
        self.latched, self.clearing, self.multi_since = set(), {}, {}
        self.telemetry, self.telemetry_at, self.joy_at = {}, None, None
        self.ever_enabled = set()
        self.graph, self.query, self.external = {}, {}, {}

    def condition(self, key, bad, now, detail=None):
        if bad:
            self.clearing.pop(key, None)
            if key not in self.latched:
                self.latched.add(key)
                self.trigger(key, detail or {})
        elif key in self.latched:
            self.clearing.setdefault(key, now)
            if now - self.clearing[key] >= self.cfg["rearm_seconds"]:
                self.latched.remove(key); self.clearing.pop(key, None)

    def feed(self, source, payload, now=None):
        now = time.monotonic() if now is None else now
        if source == "/joy": self.joy_at = now
        if source == "/agv2/chassis_telemetry":
            self.telemetry, self.telemetry_at = payload, now
        elif source == "ros_graph": self.graph = payload
        elif source == "query":
            ident = payload.get("id")
            self.query[ident] = dict(self.query.get(ident, {}), **payload)
        elif source == "can_health": self.external["can"] = payload

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        grace = now - self.started >= self.cfg["startup_grace_seconds"]
        t = self.telemetry
        elapsed = (now - self.telemetry_at) * 1000 if self.telemetry_at is not None else 0
        for end in ("front", "rear"):
            key = end + "_wheel"
            sw, error = t.get(key + "_status_word", 0), t.get(key + "_error_code", 0)
            seen = t.get(key + "_feedback_seen", False)
            if seen and sw & 4: self.ever_enabled.add(key)
            self.condition(key + "_fault", seen and (bool(sw & 8) or error != 0), now,
                           {"status_word": sw, "error_603f": error, "namespace": "wheel_CANopen"})
            self.condition(key + "_disabled", seen and key in self.ever_enabled and not sw & 4, now)
            for kind, limit in (("wheel", 500), ("steer", 300)):
                k = end + "_" + kind
                seen = t.get(k + "_feedback_seen", False)
                age = t.get(k + "_feedback_age_ms", -1)
                self.condition(k + "_startup_missing", grace and not seen, now)
                self.condition(k + "_stale", seen and (age < 0 or age + elapsed > limit), now,
                               {"reported_age_ms": age, "telemetry_age_ms": elapsed})
        self.condition("joy_stale_active", t.get("mode", 0) != 0 and
                       (self.joy_at is None or now - self.joy_at > .5), now)
        for topic in ("/joy", "/cmd_vel", "/wheel_command", "/to_can_bus"):
            n = len(self.graph.get(topic, []))
            if n > 1: self.multi_since.setdefault(topic, now)
            else: self.multi_since.pop(topic, None)
            self.condition("multiple_publishers:" + topic,
                           topic in self.multi_since and now - self.multi_since[topic] >= 1, now,
                           {"count": n, "endpoints": self.graph.get(topic, [])})
        for ident, q in self.query.items():
            self.condition("steer_query_timeout:" + str(ident),
                           grace and q.get("consecutive_failures", 0) >= self.cfg["query_failures"], now, q)
            reply = q.get("reply", {})
            self.condition("steer_status_error:" + str(ident), bool(reply.get("error_raw", 0)), now, reply)
        can = self.external.get("can", {})
        self.condition("can_unavailable", grace and not can.get("bound", False), now, can)

def decode_status(data):
    if len(data) != 8 or data[0] != 0x9a: raise ValueError("Not a 0x9A reply")
    error = data[7]
    return {"temperature_c": int.from_bytes(data[1:2], "little", signed=True),
            "bus_voltage_v": int.from_bytes(data[2:4], "little", signed=True) * .01,
            "current_a": int.from_bytes(data[4:6], "little", signed=True) * .01,
            "motor_state_raw": data[6], "error_raw": error,
            "low_voltage": bool(error & 1), "over_temperature": bool(error & 8),
            "unknown_error_bits": error & ~9}
