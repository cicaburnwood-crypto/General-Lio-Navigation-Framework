#!/usr/bin/env python3
"""Chassis fault recorder CLI; status/summarize never send CAN traffic."""
import argparse, json, os, signal, subprocess, sys, syslog, time
from pathlib import Path
from faultlog.settings import identity, load
from faultlog.storage import read_json, atomic_json
from faultlog.supervisor import Runtime, request

def summarize(path):
    p=Path(path).resolve()
    files=[p/"manifest.json"] if (p/"manifest.json").exists() else sorted(p.glob("segment_*/manifest.json"))
    result=[]
    for f in files:
        if f.parent.is_symlink() or f.is_symlink(): continue
        try: m=read_json(f)
        except (OSError,ValueError): continue
        item={"directory":str(f.parent),"manifest":m,"invalid_or_partial_lines":0,"sources":{},"wheel_faults":{},"timeline":[]}
        for k in ("state","can","events"):
            q=f.parent/(k+".jsonl")
            if q.is_symlink(): continue
            try:
                with q.open("rb") as stream:
                    for line in stream:
                        try:
                            if not line.endswith(b"\n"): raise ValueError("partial")
                            r=json.loads(line); src=r["source"]; item["sources"][src]=item["sources"].get(src,0)+1
                            if src in ("fault_trigger","deploy","kernel","write_gap","transport_gap","recorder_exit") and len(item["timeline"]) < 200:
                                item["timeline"].append({"wall_ns":r["wall_time_ns"],"monotonic_ns":r["monotonic_ns"],"source":src,"detail":r["payload"]})
                            if src=="/agv2/chassis_telemetry":
                                for end in ("front","rear"):
                                    v=r["payload"]
                                    if v.get(end+"_wheel_error_code",0) or v.get(end+"_wheel_status_word",0)&8:
                                        item["wheel_faults"][end]={"status_word":v.get(end+"_wheel_status_word"),
                                            "error_603f":v.get(end+"_wheel_error_code"),"namespace":"wheel_CANopen"}
                        except (ValueError,KeyError,TypeError): item["invalid_or_partial_lines"]+=1
            except FileNotFoundError: pass
        result.append(item)
    return result

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("command",choices=["run","status","save","summarize","export","helper","wait","activate","pause","stop","event","watch"])
    p.add_argument("--root",default="/home/hkclr/agv/agv_ws/data/chassis_faults/continuous")
    p.add_argument("--config"); p.add_argument("--params"); p.add_argument("--collector")
    p.add_argument("--interface",default="can0"); p.add_argument("--ids",nargs=2,type=int)
    p.add_argument("--period",type=float,default=1); p.add_argument("--query-timeout",type=float,default=.2)
    p.add_argument("--path"); p.add_argument("--destination"); p.add_argument("--reason",default="manual")
    p.add_argument("--field",default="durable"); p.add_argument("--timeout",type=float,default=10); p.add_argument("--session")
    a=p.parse_args(); root=Path(a.root)
    if a.command=="run":
        if os.getuid()==0: raise RuntimeError("Recorder must run as the robot user, not root")
        c,m=load(a.config,a.params,a.root,a.interface)
        if not a.collector or not os.access(a.collector,os.X_OK): raise RuntimeError("Missing C++ collector")
        repo=Path(__file__).resolve().parents[1]
        for label,args in (("git_head",["rev-parse","HEAD"]),("git_status",["status","--porcelain=v1"])):
            r=subprocess.run(["git","-C",str(repo),*args],capture_output=True,text=True,timeout=2)
            m[label]=r.stdout.strip() if r.returncode==0 else "unavailable"
        Runtime(c,m,a.collector).run()
    elif a.command=="helper":
        from faultlog.can_helper import run
        run(a.root,a.interface,a.ids,a.period,a.query_timeout)
    elif a.command=="status":
        st=read_json(root/".status.json")
        st["process_alive"]=identity(st["identity"][0])==st["identity"]
        st["heartbeat_age_seconds"]=time.monotonic()-st["heartbeat_monotonic"]
        print(json.dumps(st,ensure_ascii=False,indent=2))
        return 0 if st["process_alive"] and st["heartbeat_age_seconds"]<2 and st["healthy"] else 1
    elif a.command=="wait":
        deadline=time.monotonic()+a.timeout
        while time.monotonic()<deadline:
            try:
                s=read_json(root/".status.json")
                if (identity(s["identity"][0])==s["identity"] and time.monotonic()-s["heartbeat_monotonic"]<2
                    and s.get(a.field) and s.get("healthy") and s.get("collector_healthy")):
                    print(json.dumps(s)); return 0
            except (OSError,ValueError,KeyError): pass
            time.sleep(.1)
        print("Recorder readiness timeout: "+a.field,file=sys.stderr); return 1
    elif a.command=="summarize": print(json.dumps(summarize(a.path or a.root),ensure_ascii=False,indent=2))
    elif a.command=="export":
        # Closed packages only; copy, never move/delete. Destination must be outside managed roots.
        import fcntl, tarfile
        src=Path(a.path).resolve(); dest=Path(a.destination).absolute()
        managed=root.parent.resolve()
        if dest.resolve().is_relative_to(managed) or dest.exists() or dest.is_symlink(): raise ValueError("Use a NEW export path outside chassis_faults")
        fd=os.open(src/"manifest.json",os.O_RDONLY|os.O_NOFOLLOW)
        with os.fdopen(fd) as mf:
            fcntl.flock(mf,fcntl.LOCK_SH)
            m=json.load(mf)
            if m.get("format")!="hkclr.chassis-log" or m.get("status")=="active": raise ValueError("Only closed verified packages can be exported")
            if any(x.is_symlink() or not x.is_file() for x in src.iterdir()): raise ValueError("Unsafe package")
            # Rotation takes an exclusive manifest lock, so the source cannot disappear mid-export.
            with tarfile.open(dest,"x:gz") as out:
                for f in src.iterdir(): out.add(f,arcname=src.name+"/"+f.name,recursive=False)
        print(str(dest))
    elif a.command=="watch":
        syslog.openlog("agv234-faultlog")
        while True:
            try:
                st=read_json(root/".status.json")
                if st.get("session_id")!=a.session or st.get("closed"): return 0
                try:
                    if read_json(root/".watch_stop.json").get("session_id")==a.session: return 0
                except (OSError,ValueError): pass
                if (identity(st["identity"][0])!=st["identity"] or time.monotonic()-st["heartbeat_monotonic"]>2
                    or not st.get("healthy") or not st.get("collector_healthy") or not st.get("can_bound")):
                    syslog.syslog(syslog.LOG_WARNING,"Recording unhealthy or incomplete; robot control is NOT changed. Run chassis_fault_log.py status.")
            except (OSError,ValueError,KeyError): syslog.syslog(syslog.LOG_WARNING,"Recorder status unavailable")
            time.sleep(5)
    else:
        if a.command in ("pause","stop"):
            # Independent pause lease, also works when the recorder IPC is stalled.
            try:
                st=read_json(root/".status.json")
                atomic_json(root/".query_pause.json",{"session_id":st["session_id"]})
            except (OSError,ValueError,KeyError): pass
        try: result=request(root,a.command,reason=a.reason,detail={"stage":a.reason,"wall_ns":time.time_ns()})
        except OSError:
            if a.command != "stop": raise
            st=read_json(root/".status.json")
            if identity(st["identity"][0])==st["identity"]: raise
            atomic_json(root/".watch_stop.json",{"session_id":st["session_id"]})
            print("Recorder already exited; interrupted evidence retained; warning watcher stopped.")
            return 0
        print(json.dumps(result,ensure_ascii=False))
        if a.command=="stop":
            deadline=time.monotonic()+10
            while time.monotonic()<deadline:
                try:
                    st=read_json(root/".status.json")
                    if st.get("closed"):
                        atomic_json(root/".watch_stop.json",{"session_id":st["session_id"]}); return 0
                except (OSError,ValueError): pass
                time.sleep(.1)
            return 1
        return 0 if result.get("ok") else 1
    return 0
if __name__=="__main__":
    try: sys.exit(main())
    except Exception as e:
        syslog.openlog("agv234-faultlog"); syslog.syslog(syslog.LOG_ERR,str(e))
        print(str(e),file=sys.stderr); sys.exit(1)
