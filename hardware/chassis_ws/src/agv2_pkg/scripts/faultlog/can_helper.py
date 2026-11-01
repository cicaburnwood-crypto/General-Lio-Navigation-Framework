"""Passive CAN capture. Driver-owned 0x9A transactions are observed, never sent."""
import json
import os
import select
import signal
import socket
import struct
import time
from pathlib import Path
from .collectors import decode_raw_can, interface_info
from .faults import decode_status
from .settings import identity
from .storage import read_json

QUERY = b"\x9a\x00\x00\x00\x00\x00\x00\x00"
class Queries:
    def __init__(self, ids, period=1., timeout=.2):
        if len(ids) != 2 or len(set(ids)) != 2 or any(not 0x141 <= i <= 0x17f for i in ids):
            raise ValueError("Steering query ID whitelist")
        if period < 1 or not 0 < timeout <= .2: raise ValueError("Query rate/timeout")
        self.ids, self.period, self.timeout = ids, period, timeout
        self.next = {i: time.monotonic() + n * period / 2 for n, i in enumerate(ids)}
        self.pending, self.failures, self.last_reply = {}, dict.fromkeys(ids, 0), dict.fromkeys(ids)
        self.serial = 0
        self.was_enabled = False
        self.last_send = {}
        self.last_bus_reply_ns = {}
    def step(self, now, enabled, send, emit):
        if enabled and not self.was_enabled:
            self.next = {i: max(now + n * self.period / 2, self.last_send.get(i, -1e30) + self.period)
                         for n, i in enumerate(self.ids)}
        self.was_enabled = bool(enabled)
        for i in self.ids:
            if i in self.pending and now - self.pending[i]["sent_monotonic"] >= self.timeout:
                q = self.pending.pop(i); self.failures[i] += 1
                emit("events", "query", dict(q, status="timeout", consecutive_failures=self.failures[i],
                     last_reply_monotonic=self.last_reply[i]))
            if enabled and i not in self.pending and now >= self.next[i]:
                self.next[i] = now + self.period
                self.last_send[i] = now
                self.serial += 1
                q = {"id": i, "transaction": self.serial, "sent_monotonic": now,
                     "sent_wall_ns": time.time_ns(), "command": "0x9A"}
                try:
                    # Hard-coded payload; never accepts a control payload from IPC or config.
                    send(struct.pack("=IB3x8s", i, 8, QUERY))
                    self.pending[i] = q
                    emit("events", "query", dict(q, status="send_ok", consecutive_failures=self.failures[i]))
                except OSError as e:
                    self.failures[i] += 1
                    emit("events", "query", dict(q, status="send_failed", error=str(e),
                         consecutive_failures=self.failures[i]))
    def receive(self, frame, now, emit):
        i = frame["id"]
        if i not in self.ids or frame["error"] or frame["rtr"] or frame["origin"] != "bus_rx":
            return
        data = bytes.fromhex(frame["data_hex"])
        if len(data) != 8 or data[0] != 0x9a: return
        q = self.pending.get(i)
        timely = q is not None and 0 <= now - q["sent_monotonic"] < self.timeout
        if timely: self.pending.pop(i); self.failures[i] = 0
        self.last_reply[i] = now
        if frame.get("kernel_time_ns"):
            self.last_bus_reply_ns[i] = frame["kernel_time_ns"]
        emit("state", "query", {"id": i, "status": "reply" if timely else "late_or_unsolicited",
             "transaction": q["transaction"] if timely else None,
             "correlation": "time_and_id_only_no_wire_transaction_id",
             "reply": decode_status(data), "consecutive_failures": self.failures[i],
             "last_reply_monotonic": now})

    def observe_query(self, frame, now, emit):
        if (frame.get("origin") != "tx_observed" or frame.get("error") or
                frame.get("rtr") or frame.get("extended") or
                frame.get("id") not in self.ids or frame.get("data_hex") != QUERY.hex()):
            return
        i = frame["id"]
        if frame.get("kernel_time_ns", 0) and frame["kernel_time_ns"] <= self.last_bus_reply_ns.get(i, -1):
            return  # RX can be drained before AF_PACKET; do not invent a timeout
        self.serial += 1
        q = {"id": i, "transaction": self.serial, "sent_monotonic": now,
             "sent_wall_ns": time.time_ns(), "command": "0x9A",
             "owner": "external_driver_observed_passively"}
        self.pending[i] = q
        emit("events", "query", dict(q, status="send_observed", consecutive_failures=self.failures[i]))

def run(root, interface, ids, period=1., timeout=.2):
    root = Path(root)
    peer = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    peer.setblocking(False)
    dropped = 0
    def emit(kind, source, payload):
        nonlocal dropped
        obj = {"kind": kind, "source": source, "payload": payload,
               "capture_wall_ns": time.time_ns(), "capture_monotonic_ns": time.monotonic_ns()}
        try: peer.sendto(json.dumps(obj, separators=(",", ":")).encode(), str(root / ".ingest"))
        except OSError: dropped += 1
    stopping = False
    def stop(*_):
        nonlocal stopping
        stopping = True
    for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP): signal.signal(sig, stop)
    raw = packet = None
    bound_index = None
    queries = Queries(ids, period, timeout)
    last_health = 0
    up = False
    boot = time.monotonic()
    last_valid = boot
    cached_lease = {}
    while not stopping:
        now = time.monotonic()
        try:
            cached_lease = read_json(root / ".status.json")
        except (OSError, ValueError, KeyError, TypeError): pass
        lease = dict(cached_lease)
        try:
            valid = (0 <= time.monotonic() - lease["heartbeat_monotonic"] < 1.5 and
                     identity(lease["identity"][0]) == lease["identity"])
        except (KeyError, TypeError): valid = False
        if valid: last_valid = now
        try:
            if read_json(root / ".query_pause.json").get("session_id") == lease.get("session_id"):
                lease["queries_enabled"] = False
        except (OSError, ValueError): pass
        if not valid and now - last_valid > 3: break
        if now - last_health >= .5:
            last_health = now
            link = interface_info(interface)
            flags = link.get("link", {}).get("flags", [])
            index = link.get("link", {}).get("ifindex")
            up = "UP" in flags
            canstate = link.get("link", {}).get("linkinfo", {}).get("info_data", {}).get("state")
            if canstate == "BUS-OFF": up = False
            if raw and (not up or index != bound_index):
                raw.close(); packet.close(); raw = packet = None
            if up and raw is None:
                try:
                    raw = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                    raw.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_ERR_FILTER, struct.pack("=I", 0x1fffffff))
                    raw.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_RECV_OWN_MSGS, 1)
                    raw.setsockopt(socket.SOL_SOCKET, 35, 1)
                    raw.setsockopt(socket.SOL_SOCKET, 40, 1)
                    raw.bind((interface,))
                    packet = socket.socket(socket.AF_PACKET, socket.SOCK_RAW, socket.htons(0x0003))
                    packet.setsockopt(socket.SOL_SOCKET, 35, 1)
                    packet.setsockopt(socket.SOL_SOCKET, 40, 1)
                    packet.bind((interface, 0))
                    bound_index = index
                except OSError as e:
                    for s in (raw, packet):
                        if s: s.close()
                    raw = packet = None
                    emit("events", "can_bind_error", {"error": str(e)})
            emit("events", "can_health", {"bound": bool(raw and packet), "interface": interface,
                 "link": link, "helper_dropped": dropped, "helper_identity": identity(os.getpid()),
                 "queries_enabled": False, "query_mode": "passive_driver_observer"})
        if raw:
            now = time.monotonic()
            # Never transmit, even when the old recorder lease enables queries.
            queries.step(now, False, None, emit)
            ready, _, _ = select.select([raw, packet], [], [], .01)
            for s in sorted(ready, key=lambda item: item is not packet):
                try:
                    data, anc, flags, addr = s.recvmsg(72, 128)
                    if s is packet and addr[2] != 4: continue  # PACKET_OUTGOING only
                    if len(data) != 16: continue
                    f = decode_raw_can(data, flags, anc)
                    if s is packet:
                        f["origin"] = "tx_observed"
                        queries.observe_query(f, time.monotonic(), emit)
                        if f["id"] in queries.pending and f["data_hex"] == QUERY.hex():
                            q = queries.pending[f["id"]]
                            f["query_transaction"] = q["transaction"]
                            emit("events", "query_path_observed", dict(q, observation="local_tx_path_not_motor_ack"))
                    f["interface"] = interface
                    emit("can", interface, f)
                    if s is raw: queries.receive(f, time.monotonic(), emit)
                except (OSError, ValueError) as e: emit("events", "can_receive_error", {"error": str(e)})
        else:
            queries.step(now, False, None, emit)
            time.sleep(.02)
    emit("events", "can_helper_exit", {"dropped": dropped})
    for s in (raw, packet, peer):
        if s: s.close()
