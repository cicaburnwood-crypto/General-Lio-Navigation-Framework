from __future__ import annotations

import collections
import json
import math
import select
import socket
import struct
import subprocess
import threading
import time
from pathlib import Path

from . import SCHEMA_VERSION

TOPICS = ("/cmd_vel", "/joy", "/wheel_command", "/agv2/chassis_telemetry", "/Battery_SOC_STATE")


def clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    if isinstance(value, str) and len(value) > 16384:
        return value[:16384] + "[TRUNCATED]"
    return value


class RecordQueue:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.can = collections.deque()
        self.other = collections.deque()
        self.sequence = 0
        self.received = {}
        self.losses = collections.Counter()
        self.first_loss_ns = None
        self.last_loss_ns = None

    def _loss(self, reason, record):
        self.losses[reason] += 1
        timestamp = record.get("monotonic_ns", time.monotonic_ns())
        self.first_loss_ns = timestamp if self.first_loss_ns is None else min(self.first_loss_ns, timestamp)
        self.last_loss_ns = timestamp if self.last_loss_ns is None else max(self.last_loss_ns, timestamp)

    def emit(self, kind, source, payload):
        with self.lock:
            self.sequence += 1
            record = {"schema_version": SCHEMA_VERSION, "sequence": self.sequence,
                      "wall_time_ns": time.time_ns(), "monotonic_ns": time.monotonic_ns(),
                      "kind": kind, "source": source, "payload": clean(payload)}
            self.received[source] = record["monotonic_ns"]
            queue = self.can if kind == "can" else self.other
            capacity = self.cfg["can_queue_records" if kind == "can" else "state_queue_records"]
            if len(queue) >= capacity:
                self._loss("can_queue_overflow" if kind == "can" else "state_queue_overflow", queue.popleft())
            queue.append(record)

    def pop(self, limit=500):
        batch = []
        with self.lock:
            for _ in range(limit):
                if not self.can and not self.other:
                    break
                if not self.other or (self.can and self.can[0]["sequence"] < self.other[0]["sequence"]):
                    batch.append(self.can.popleft())
                else:
                    batch.append(self.other.popleft())
        return batch

    def discard(self, records, reason):
        with self.lock:
            for record in records:
                self._loss(reason, record)

    def stats(self):
        with self.lock:
            return {"queued_can": len(self.can), "queued_state_events": len(self.other),
                    "received": self.sequence, "dropped": sum(self.losses.values()),
                    "drop_reasons": dict(self.losses), "first_loss_monotonic_ns": self.first_loss_ns,
                    "last_loss_monotonic_ns": self.last_loss_ns}

    def freshness(self):
        now = time.monotonic_ns()
        with self.lock:
            received = dict(self.received)
        result = {}
        for topic in TOPICS:
            last = received.get(topic)
            age = (now - last) / 1e9 if last is not None else None
            result[topic] = {"last_received_monotonic_ns": last, "age_seconds": age,
                             "state": "not_seen" if last is None else
                             "stale" if age > 0.5 else "fresh"}
        return result


def interface_info(interface):
    try:
        result = subprocess.run(["ip", "-j", "-details", "-statistics", "link", "show", "dev", interface],
                                capture_output=True, text=True, timeout=0.8, check=False)
        if result.returncode:
            return {"available": False, "error": result.stderr.strip()[:512]}
        return {"available": True, "link": json.loads(result.stdout)[0]}
    except (OSError, ValueError, IndexError, subprocess.TimeoutExpired) as exc:
        return {"available": False, "error": str(exc)[:512]}


def decode_raw_can(data, flags, ancillary=()):
    if len(data) != 16:
        raise ValueError(f"Expected classic CAN frame (16 bytes), got {len(data)}")
    raw_id, dlc, payload = struct.unpack("=IB3x8s", data)
    if dlc > 8:
        raise ValueError(f"Invalid classic CAN DLC: {dlc}")
    kernel_ns, overflow = None, None
    for level, kind, value in ancillary:
        if level != socket.SOL_SOCKET:
            continue
        if kind == getattr(socket, "SO_TIMESTAMPNS", 35) and len(value) >= struct.calcsize("@ll"):
            seconds, nanos = struct.unpack_from("@ll", value)
            kernel_ns = seconds * 1000000000 + nanos
        elif kind == getattr(socket, "SO_RXQ_OVFL", 40) and len(value) >= 4:
            overflow = struct.unpack_from("=I", value)[0]
    extended = bool(raw_id & 0x80000000)
    error = bool(raw_id & 0x20000000)
    return {"id": raw_id & (0x1FFFFFFF if extended or error else 0x7FF),
            "raw_id": raw_id, "dlc": dlc, "data_hex": payload[:dlc].hex(),
            "extended": extended, "rtr": bool(raw_id & 0x40000000), "error": error,
            "origin": "local_tx_echo" if flags & socket.MSG_DONTROUTE else "bus_rx",
            "recvmsg_flags": flags, "kernel_time_ns": kernel_ns,
            "socket_drops_counter": overflow}
