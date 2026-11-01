"""Validated configuration; no robot parameters are changed."""
import hashlib
import json
import os
import re
from pathlib import Path
import yaml

def load(config, params, root, interface):
    c = yaml.safe_load(Path(config).read_text())
    if not isinstance(c, dict): raise ValueError("Invalid logger config")
    for k, v in c.items():
        if not isinstance(v, (int, float)) or isinstance(v, bool) or v <= 0:
            raise ValueError("Invalid positive config value: " + k)
    if c["query_period_seconds"] < 1 or c["query_timeout_seconds"] > .2:
        raise ValueError("Queries limited to <= 1 Hz, timeout <= 200 ms")
    if c["incident_extend_seconds"] < c["incident_post_seconds"]:
        raise ValueError("Incident maximum window shorter than post window")
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,15}", interface): raise ValueError("Invalid CAN interface")
    p = yaml.safe_load(Path(params).read_text())
    p = p.get("agv2_control_node", p.get("/**", {})).get("ros__parameters", {})
    ids = [p.get("can", {}).get("steer_front_id"), p.get("can", {}).get("steer_rear_id")]
    if any(type(i) is not int or not 0x141 <= i <= 0x17f for i in ids) or len(set(ids)) != 2:
        raise ValueError("Need distinct steering IDs 0x141..0x17f from actual launch config")
    r = Path(root)
    if not r.is_absolute() or r.resolve() != r or r.name != "continuous":
        raise ValueError("Root must be absolute, non-symlink continuous directory")
    c.update(root=str(r), interface=interface, steering_ids=ids,
             incidents_root=str(r.parent / "incidents"))
    # Chassis file is the explicitly selected config, never the full environment.
    meta = {"params": p, "log_config": dict(c), "sha256": {
        str(Path(f).resolve()): hashlib.sha256(Path(f).read_bytes()).hexdigest()
        for f in (config, params)}, "domain": os.getenv("ROS_DOMAIN_ID"),
        "rmw": os.getenv("RMW_IMPLEMENTATION", "default"),
        "boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip()}
    return c, meta

def identity(pid):
    try:
        # comm may contain spaces/parentheses; starttime is field 22.
        return [int(pid), Path("/proc/%s/stat" % pid).read_text().rsplit(")", 1)[1].split()[19]]
    except (OSError, IndexError, ValueError): return None
