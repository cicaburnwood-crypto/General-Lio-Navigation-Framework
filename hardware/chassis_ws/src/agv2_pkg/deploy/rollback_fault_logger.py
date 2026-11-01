#!/usr/bin/env python3
"""Explicit offline rollback; never stops hardware and never deletes log directories."""
import argparse, hashlib, json, os, shutil, tarfile, time
from pathlib import Path
WS=Path("/home/hkclr/agv/agv_ws")
BACKUP=Path("/home/hkclr/agv/backups/fault_logger_20261007_162639")
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser(); p.add_argument("--confirm-robot-stopped",action="store_true"); a=p.parse_args()
    if not a.confirm_robot_stopped: raise RuntimeError("First use the normal hardware stop procedure, then confirm")
    for d in Path("/proc").iterdir():
        if not d.name.isdigit(): continue
        try:
            exe=(d/"exe").resolve().name
            cmd=(d/"cmdline").read_bytes()
            if exe in ("agv2_control_node","fault_log_collector") or b"chassis_fault_log.py\x00run" in cmd:
                raise RuntimeError("Robot/recorder process still running: "+d.name)
        except (FileNotFoundError,PermissionError,ProcessLookupError): pass
    entries=json.loads((BACKUP/"installation_receipt.json").read_text())["files"]
    for e in entries:
        dest=Path(e["path"])
        if not (dest.is_relative_to(WS/"src/agv2_pkg") or dest.is_relative_to(WS/"install_release_humble/agv2_pkg")):
            raise RuntimeError("Unexpected target")
        if dest.is_symlink() or not dest.is_file() or sha(dest)!=e["after"]:
            raise RuntimeError("File changed since installation; review before rollback: "+str(dest))
    saved=BACKUP/("rollback_saved_"+time.strftime("%Y%m%d_%H%M%S")); saved.mkdir(mode=0o700)
    with tarfile.open(BACKUP/"source.tar") as source, tarfile.open(BACKUP/"install.tar") as installed:
        for e in entries:
            dest=Path(e["path"]); rel=dest.relative_to(WS); copy=saved/rel
            copy.parent.mkdir(parents=True,exist_ok=True); shutil.copy2(dest,copy)
            if e["before"] is None:
                dest.unlink()  # Exact verified receipt file, recoverable in rollback_saved.
            else:
                archive=source if rel.parts[0]=="src" else installed
                member=archive.getmember(rel.as_posix())
                if not member.isfile(): raise RuntimeError("Backup entry is not a regular file")
                data=archive.extractfile(member).read()
                if hashlib.sha256(data).hexdigest()!=e["before"]: raise RuntimeError("Backup hash mismatch")
                tmp=dest.with_name(dest.name+".rollback-tmp")
                with tmp.open("xb") as f: f.write(data); f.flush(); os.fsync(f.fileno())
                os.chmod(tmp,member.mode); os.replace(tmp,dest)
    print("Restored previous deploy/source. Removed added files are recoverable in "+str(saved))
    print("All continuous, incident and historical bag logs remain untouched.")
if __name__=="__main__": main()

