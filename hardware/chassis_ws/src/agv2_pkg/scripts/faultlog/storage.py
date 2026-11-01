from __future__ import annotations

import fcntl
import json
import os
import re
import shutil
import stat
import time
import uuid
from pathlib import Path

from . import FORMAT, SCHEMA_VERSION

SEGMENT_RE = re.compile(r"^segment_\d{8}T\d{6}Z_[0-9a-f]{12}_\d{10}$")
FILES = {"manifest.json", "state.jsonl", "can.jsonl", "events.jsonl"}


def read_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW)
    with os.fdopen(fd, "r", encoding="utf-8") as stream:
        return json.load(stream)


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + "." + uuid.uuid4().hex + ".tmp")
    try:
        fd = os.open(tmp, os.O_WRONLY | os.O_CREAT | os.O_EXCL | os.O_NOFOLLOW, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


class StoragePressure(OSError):
    pass


class Store:
    """One writer; one process lock. Only recognized closed segments are removed."""

    def __init__(self, cfg, session_id=None, metadata=None):
        self.cfg = cfg
        self.root = Path(cfg["root"])
        if self.root.absolute() != self.root.resolve():
            raise ValueError("Symlinked log root rejected")
        self.root.mkdir(parents=True, exist_ok=True, mode=0o700)
        self.session_id = session_id or uuid.uuid4().hex[:12]
        self.metadata = metadata or {}
        self.lock_fd = os.open(self.root / ".lock", os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
        try:
            fcntl.flock(self.lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception:
            os.close(self.lock_fd)
            raise RuntimeError("Another chassis recorder owns this log root") from None
        self.path = None
        self.handles = {}
        self.manifest = None
        self.segment_bytes = 0
        self.opened_mono = 0.0
        self.closed = False
        self.deleted = []
        self.pins = set()
        self.last_written_sequence = 0
        try:
            marker = self.root / ".owner.json"
            if marker.exists():
                if read_json(marker).get("format") != FORMAT:
                    raise ValueError("Log directory belongs to a different application")
            else:
                if any(p.name != ".lock" for p in self.root.iterdir()):
                    raise ValueError("Nonempty directory lacks chassis logger ownership marker")
                atomic_json(marker, {"format": FORMAT, "schema_version": SCHEMA_VERSION})
            for directory, manifest in self.segments():
                if manifest.get("status") == "active":
                    manifest.update(status="interrupted", recovered_at_ns=time.time_ns(),
                                    exit_reason="previous_recorder_did_not_close",
                                    completeness="tail_may_be_incomplete")
                    atomic_json(directory / "manifest.json", manifest)
        except Exception:
            self.release()
            raise

    def segments(self):
        found = []
        for directory in self.root.iterdir():
            if not SEGMENT_RE.fullmatch(directory.name):
                continue
            if not stat.S_ISDIR(directory.lstat().st_mode):
                continue
            try:
                manifest = read_json(directory / "manifest.json")
                if (isinstance(manifest, dict) and manifest.get("format") == FORMAT and
                        manifest.get("schema_version") == SCHEMA_VERSION and
                        type(manifest.get("sequence")) is int and
                        manifest.get("directory") == directory.name):
                    found.append((directory, manifest))
            except (OSError, ValueError, TypeError):
                continue
        return sorted(found, key=lambda pair: pair[1]["sequence"])

    def usage(self):
        total = 0
        for current, dirs, names in os.walk(self.root, followlinks=False):
            dirs[:] = [d for d in dirs if not (Path(current) / d).is_symlink()]
            for name in names:
                try:
                    st = (Path(current) / name).lstat()
                    if stat.S_ISREG(st.st_mode):
                        total += st.st_size
                except FileNotFoundError:
                    pass
        return total, shutil.disk_usage(self.root).free

    def _delete_closed(self, directory):
        # Flat, recognized segments only. Directory-relative operations never follow links.
        if directory.parent != self.root or not SEGMENT_RE.fullmatch(directory.name):
            return False
        if directory == self.path or directory.name in self.pins:
            return False
        try:
            fd = os.open(directory, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW)
        except OSError:
            return False
        mfd = None
        try:
            mfd = os.open("manifest.json", os.O_RDONLY | os.O_NOFOLLOW, dir_fd=fd)
            fcntl.flock(mfd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            with os.fdopen(os.dup(mfd), "r", encoding="utf-8") as stream:
                manifest = json.load(stream)
            if (not isinstance(manifest, dict) or manifest.get("format") != FORMAT or manifest.get("directory") != directory.name
                    or manifest.get("status") == "active"):
                return False
            names = os.listdir(fd)
            for name in names:
                if name not in FILES and not re.fullmatch(r"manifest\.json\.[0-9a-f]{32}\.tmp", name):
                    return False
                if not stat.S_ISREG(os.stat(name, dir_fd=fd, follow_symlinks=False).st_mode):
                    return False
            for name in names:
                os.unlink(name, dir_fd=fd)
        except (OSError, ValueError):
            return False
        finally:
            if mfd is not None:
                os.close(mfd)
            os.close(fd)
        os.rmdir(directory)
        self.deleted.append(directory.name)
        return True

    def enforce_limits(self, reserve=0):
        segments = self.segments()
        total, free = self.usage()

        def enough():
            return (len(segments) + reserve <= self.cfg["max_segments"] and
                    total + reserve * 16384 < self.cfg["max_total_bytes"] and
                    free - reserve * 16384 >= self.cfg["min_free_bytes"])

        for directory, manifest in list(segments):
            if enough():
                break
            if directory == self.path or manifest.get("status") == "active":
                continue
            if self._delete_closed(directory):
                segments = [pair for pair in segments if pair[0] != directory]
                total, free = self.usage()
        if not enough():
            raise StoragePressure(f"Storage limits: segments={len(segments)}, bytes={total}, free={free}")
        return total, free

    def open_segment(self):
        if self.path is not None:
            raise RuntimeError("Close the current segment before opening another")
        self.enforce_limits(reserve=1)
        index_path = self.root / ".index.json"
        next_sequence = 1
        if index_path.exists():
            next_sequence = int(read_json(index_path)["next_sequence"])
        segments = self.segments()
        if segments:
            next_sequence = max(next_sequence, segments[-1][1]["sequence"] + 1)
        atomic_json(index_path, {"next_sequence": next_sequence + 1})
        utc = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        self.path = self.root / f"segment_{utc}_{self.session_id}_{next_sequence:010d}"
        self.path.mkdir(mode=0o700)
        self.manifest = {
            "format": FORMAT, "schema_version": SCHEMA_VERSION,
            "session_id": self.session_id, "sequence": next_sequence,
            "directory": self.path.name, "status": "active", "started_at_ns": time.time_ns(),
            "started_monotonic_ns": time.monotonic_ns(), "metadata": self.metadata,
            "records": {"can": 0, "state": 0, "events": 0}, "completeness": "recording",
        }
        self.segment_bytes = 0
        self.opened_mono = time.monotonic()
        atomic_json(self.path / "manifest.json", self.manifest)
        for kind in ("can", "state", "events"):
            fd = os.open(self.path / (kind + ".jsonl"),
                         os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
            self.handles[kind] = os.fdopen(fd, "wb", buffering=65536)

    def append(self, record):
        if self.path is None:
            self.open_segment()
        kind = record["kind"]
        data = (json.dumps(record, ensure_ascii=False, allow_nan=False,
                           separators=(",", ":")) + "\n").encode("utf-8")
        self.handles[kind].write(data)
        self.segment_bytes += len(data)
        self.manifest["records"][kind] += 1
        self.last_written_sequence = max(self.last_written_sequence, record.get("sequence", 0))

    def due(self):
        return self.path is not None and (
            time.monotonic() - self.opened_mono >= self.cfg["segment_seconds"] or
            self.segment_bytes >= self.cfg["segment_bytes"])

    def flush(self, sync=False):
        for handle in self.handles.values():
            handle.flush()
            if sync:
                os.fsync(handle.fileno())

    def close_segment(self, reason, stats=None, *, status="closed"):
        if self.path is None:
            return
        self.flush(sync=True)
        for handle in self.handles.values():
            handle.close()
        self.handles.clear()
        self.manifest.update(status=status, exit_reason=reason, ended_at_ns=time.time_ns(),
                             ended_monotonic_ns=time.monotonic_ns(),
                             completeness="closed_with_reported_gaps" if (stats or {}).get("dropped", 0)
                             else "closed", queue_statistics=stats or {})
        atomic_json(self.path / "manifest.json", self.manifest)
        self.path = None

    def abandon_handles(self):
        # Do not append behind a possibly incomplete JSON line after an I/O failure.
        for handle in self.handles.values():
            try:
                handle.close()
            except OSError:
                pass
        self.handles.clear()

    def recover_write(self, stats):
        self.enforce_limits()
        if self.path is not None:
            self.manifest.update(status="write_error", exit_reason="disk_write_interrupted",
                                 ended_at_ns=time.time_ns(), completeness="tail_may_be_incomplete",
                                 queue_statistics=stats)
            atomic_json(self.path / "manifest.json", self.manifest)
            self.path = None
        self.open_segment()

    def release(self):
        if not self.closed:
            self.closed = True
            os.close(self.lock_fd)
