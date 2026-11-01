#!/usr/bin/env python3
"""Root launcher only. Recorder and ROS collector have no capabilities."""
import argparse, json, os, pwd, shlex, subprocess, sys, time
from pathlib import Path
repo=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(repo/"scripts"))
from faultlog.settings import load, identity
from faultlog.storage import read_json
def main():
    p=argparse.ArgumentParser(); p.add_argument("--user",required=True); p.add_argument("--params",required=True)
    p.add_argument("--interface",required=True); a=p.parse_args()
    if os.geteuid()!=0: raise RuntimeError("Launcher requires existing deploy sudo privileges")
    user=pwd.getpwnam(a.user)
    if user.pw_uid==0: raise RuntimeError("Invoke deploy from the robot user via sudo")
    ws=repo.parents[1]; root=ws/"data/chassis_faults/continuous"
    cli=repo/"scripts/chassis_fault_log.py"; config=repo/"config/fault_logger.yaml"
    collector=ws/"install_release_humble/agv2_pkg/lib/agv2_pkg/fault_log_collector"
    c,_=load(config,a.params,root,a.interface)
    if not collector.is_file(): raise RuntimeError("Build/install fault_log_collector first")
    if root.exists():
        try:
            old=read_json(root/".status.json")
            if identity(old["identity"][0])==old["identity"] and not old.get("closed"):
                raise RuntimeError("Existing recorder owns this deployment; inspect/stop it first")
        except (FileNotFoundError,KeyError): pass
    def user_cmd(args):
        cmd="source "+shlex.quote(str(repo/"deploy/setup_agv_env.bash"))+" && exec "+shlex.join(args)
        return ["/usr/bin/sudo","-u",a.user,"env","HOME="+user.pw_dir,"/bin/bash","-c",cmd]
    run=[sys.executable,"-B",str(cli),"run","--root",str(root),"--config",str(config),
         "--params",a.params,"--interface",a.interface,"--collector",str(collector)]
    proc=subprocess.Popen(user_cmd(run),stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
    deadline=time.monotonic()+10
    try:
        while time.monotonic()<deadline:
            if proc.poll() is not None: raise RuntimeError("Recorder exited during preflight; see journalctl -t agv234-faultlog")
            try:
                st=read_json(root/".status.json")
                if st.get("durable") and st.get("healthy") and st.get("collector_healthy") and identity(st["identity"][0])==st["identity"]: break
            except (OSError,ValueError,KeyError): pass
            time.sleep(.1)
        else: raise RuntimeError("Recorder first durable write/ROS readiness exceeded 10 seconds")
        # Drop all capabilities except NET_RAW in this ONE process. No permanent setcap.
        helper=["/usr/bin/setpriv","--reuid="+str(user.pw_uid),"--regid="+str(user.pw_gid),"--init-groups",
                "--bounding-set=-all,+net_raw","--inh-caps=-all,+net_raw","--ambient-caps=-all,+net_raw","--no-new-privs",
                sys.executable,"-B",str(cli),"helper","--root",str(root),"--interface",a.interface,
                "--ids",*[str(i) for i in c["steering_ids"]],"--period",str(c["query_period_seconds"]),
                "--query-timeout",str(c["query_timeout_seconds"])]
        subprocess.Popen(helper,stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        subprocess.Popen(user_cmd([sys.executable,"-B",str(cli),"watch","--root",str(root),"--session",st["session_id"]]),
                         stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,start_new_session=True)
        print("Recorder PID=%s directory=%s DOMAIN=76"%(st["identity"][0],st["directory"]))
    except Exception:
        # This process only owns the newly launched preflight recorder; no robot was started.
        proc.terminate()
        raise
if __name__=="__main__":
    try: main()
    except Exception as e: print(str(e),file=sys.stderr); sys.exit(1)

