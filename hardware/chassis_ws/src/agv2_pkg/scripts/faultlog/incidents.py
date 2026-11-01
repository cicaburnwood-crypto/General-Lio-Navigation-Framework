"""Asynchronous, bounded incident extraction. Source segments pinned only during reads."""
import json
import os
import threading
import time
from pathlib import Path
from .storage import Store, atomic_json, read_json

class Incidents:
    def __init__(self, cfg, source, source_lock, stats, emit):
        self.cfg, self.source, self.source_lock, self.stats, self.emit = cfg, source, source_lock, stats, emit
        c = dict(cfg, root=cfg["incidents_root"], max_segments=cfg["incident_max_count"],
                 max_total_bytes=cfg["incident_max_total_bytes"])
        self.store = Store(c, metadata={"purpose": "automatic_fault_package"})
        self.jobs, self.lock, self.stop_event = [], threading.Lock(), threading.Event()
        self.active = None; self.offsets = {}; self.requests = []
        self.write_failed = False
        self.thread = threading.Thread(target=self.loop, name="incident-extraction", daemon=True)
        self.thread.start()
    def request(self, reason, detail=None):
        job = {"reason": reason, "detail": detail or {}, "wall_ns": time.time_ns(),
               "mono_ns": time.monotonic_ns(), "done": threading.Event(), "result": None}
        job["watermark"] = self.stats().get("received", 0)
        with self.lock:
            if len(self.jobs) >= 64: raise RuntimeError("Incident request queue full")
            self.jobs.append(job)
        return job
    def persist(self):
        self.store.flush(True)
        self.store.manifest["incident"] = self.active
        atomic_json(self.store.path / "manifest.json", self.store.manifest)
    def scan(self):
        # Flush under the writer lock; no scans/copies while holding it.
        with self.source_lock:
            self.source.flush(True)
            sources = [(p, m) for p, m in self.source.segments()
                       if m["session_id"] == self.source.session_id and
                       m.get("ended_monotonic_ns", 2**63) >= self.active["requested_start_monotonic_ns"] and
                       m["started_monotonic_ns"] <= self.active["requested_end_monotonic_ns"]]
            self.source.pins.update(p.name for p, _ in sources)
            sizes = {(str(p), k): (p / (k + ".jsonl")).stat().st_size
                     for p, _ in sources for k in ("state", "events", "can")}
        try:
            for p, _ in sources:
                for kind in ("events", "state", "can"):
                    f = p / (kind + ".jsonl")
                    key = str(f)
                    fd = os.open(f, os.O_RDONLY | os.O_NOFOLLOW)
                    with os.fdopen(fd, "rb") as stream:
                        stream.seek(self.offsets.get(key, 0))
                        end = sizes[(str(p), kind)]
                        while stream.tell() < end:
                            pos = stream.tell()
                            line = stream.readline(1024 * 1024)
                            if not line.endswith(b"\n"):
                                # A partial trailing record is left untouched and retried next scan.
                                stream.seek(pos); break
                            self.offsets[key] = stream.tell()
                            try: r = json.loads(line)
                            except (ValueError, UnicodeError):
                                self.active["invalid_lines"] += 1; continue
                            stamp = r["monotonic_ns"]
                            if not self.active["requested_start_monotonic_ns"] <= stamp <= self.active["requested_end_monotonic_ns"]:
                                continue
                            if self.store.segment_bytes + len(line) >= self.cfg["incident_max_bytes"] - 65536:
                                self.active["truncated"] = True; continue
                            self.store.append(r)
                            a = self.active
                            a["actual_first_monotonic_ns"] = min(stamp, a["actual_first_monotonic_ns"] or stamp)
                            a["actual_last_monotonic_ns"] = max(stamp, a["actual_last_monotonic_ns"] or stamp)
            self.active["queue_statistics"] = self.stats()
            first = self.active["actual_first_monotonic_ns"]
            self.active["pre_window_available_seconds"] = 0 if first is None else max(0, min(
                self.cfg["incident_pre_seconds"], (self.active["first_trigger_monotonic_ns"]-first)/1e9))
            self.active["pre_window_short"] = self.active["pre_window_available_seconds"] < self.cfg["incident_pre_seconds"] - 1
            self.active["coverage_note"] = "actual record timestamps; gaps/dropped records remain separately reported"
            self.persist()
        finally:
            with self.source_lock:
                self.source.pins.difference_update(p.name for p, _ in sources)
    def close(self, reason):
        self.active["finalization"] = reason
        self.active["post_window_complete"] = time.monotonic_ns() >= self.active["requested_end_monotonic_ns"]
        self.persist()
        self.store.close_segment(reason, self.stats())
        self.active = None; self.offsets = {}
    def loop(self):
        while not self.stop_event.is_set() or self.jobs:
            with self.lock: jobs, self.jobs = self.jobs, []
            try:
                if self.write_failed:
                    previous = self.store.path.name if self.store.path else None
                    if self.store.path is not None:
                        self.store.recover_write(self.stats())
                    else:
                        self.store.enforce_limits()
                    if self.active:
                        self.active.setdefault("previous_incomplete_fragments", []).append(previous)
                        self.active["id"] = self.store.path.name
                        self.active["truncated"] = True
                    elif self.store.path is not None:
                        self.store.close_segment("aborted_incident_recovered", self.stats())
                    self.write_failed = False
                for j in jobs:
                    if self.active is None:
                        self.store.open_segment()
                        n = j["mono_ns"]
                        self.active = {"id": self.store.path.name, "triggers": [],
                            "first_trigger_monotonic_ns": n,
                            "requested_start_monotonic_ns": n - int(self.cfg["incident_pre_seconds"] * 1e9),
                            "requested_end_monotonic_ns": n + int(self.cfg["incident_post_seconds"] * 1e9),
                            "actual_first_monotonic_ns": None, "actual_last_monotonic_ns": None,
                            "pre_window_available_seconds": min(self.cfg["incident_pre_seconds"],
                                (n - self.source.metadata["recorder_started_monotonic_ns"]) / 1e9),
                            "invalid_lines": 0, "truncated": False}
                        self.store.manifest["metadata"]["configuration"] = self.source.metadata
                    a = self.active
                    a["triggers"].append({k: j[k] for k in ("reason", "detail", "wall_ns", "mono_ns")})
                    a["requested_end_monotonic_ns"] = min(
                        a["first_trigger_monotonic_ns"] + int(self.cfg["incident_extend_seconds"] * 1e9),
                        j["mono_ns"] + int(self.cfg["incident_post_seconds"] * 1e9))
                    self.requests.append(j)
                if self.active:
                    watermark = max((j["watermark"] for j in self.requests), default=0)
                    deadline = time.monotonic() + 5
                    while self.source.last_written_sequence < watermark and time.monotonic() < deadline:
                        time.sleep(.01)
                    if self.source.last_written_sequence < watermark:
                        raise OSError("Requested records not yet durable; save not acknowledged")
                    self.scan()
                    for j in self.requests:
                        j["result"] = {"ok": True, "incident": self.active["id"], "state": "post_capturing",
                                       "coverage": {k:self.active.get(k) for k in ("requested_start_monotonic_ns","requested_end_monotonic_ns","actual_first_monotonic_ns","actual_last_monotonic_ns","pre_window_short","truncated")}}
                        j["done"].set()
                    self.requests.clear()
                    if time.monotonic_ns() >= self.active["requested_end_monotonic_ns"]:
                        self.close("post_window_complete")
                for p, m in self.store.segments():
                    if m["status"] != "active" and time.time_ns() - m["started_at_ns"] > self.cfg["incident_max_age_seconds"] * 1e9:
                        self.store._delete_closed(p)
                self.store.enforce_limits()
            except (OSError, ValueError, KeyError) as e:
                self.emit("events", "incident_write_failure", {"error": str(e)})
                self.store.abandon_handles()
                self.write_failed = True
                for j in jobs + self.requests:
                    if not j["done"].is_set():
                        j["result"] = {"ok": False, "error": str(e)}; j["done"].set()
                self.requests.clear()
                self.stop_event.wait(max(0, self.cfg["retry_seconds"]-1))
            self.stop_event.wait(1)
        try:
            if self.active: self.scan(); self.close("recorder_stopped_early")
        except Exception as e: self.emit("events", "incident_close_failure", {"error": str(e)})
        finally: self.store.release()
    def stop(self, timeout=5):
        self.stop_event.set(); self.thread.join(timeout)
        return not self.thread.is_alive()
