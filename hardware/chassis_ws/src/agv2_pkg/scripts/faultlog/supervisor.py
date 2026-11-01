"""Persistent unprivileged recorder, independent of the deployment shell."""
import json, os, signal, socket, struct, subprocess, sys, threading, time
from pathlib import Path
from .collectors import RecordQueue
from .faults import Detector
from .incidents import Incidents
from .settings import identity
from .storage import Store, atomic_json

def request(root, command, **extra):
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.settimeout(8)
    try:
        s.connect(str(Path(root) / ".control"))
        s.sendall((json.dumps(dict(command=command, **extra)) + "\n").encode())
        with s.makefile("rb") as f: return json.loads(f.readline(65536))
    finally: s.close()

class Runtime:
    def __init__(self, cfg, metadata, collector=None):
        self.cfg, self.metadata, self.collector = cfg, metadata, collector
        self.root = Path(cfg["root"])
        metadata["recorder_started_monotonic_ns"] = time.monotonic_ns()
        metadata["recorder_identity"] = identity(os.getpid())
        self.store = Store(cfg, metadata=metadata)
        self.store_lock = threading.RLock()
        self.queue = RecordQueue(cfg)
        self.stop_event, self.writer_stop = threading.Event(), threading.Event()
        self.transport_losses = {}
        self.incidents = Incidents(cfg, self.store, self.store_lock, self.all_stats, self.queue.emit)
        self.detector = Detector(cfg, self.trigger)
        self.detector_lock = threading.Lock()
        self.queries_enabled = self.armed = self.healthy = self.durable = self.bound = False
        self.child = None; self.helper_at = self.collector_at = 0
        self.previous_can = {}; self.last_processes = None
        self.last_tmux = None
        self.stop_reason = "normal"; self.threads = []
        self.server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.ingest = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        for name in (".control", ".ingest"):
            p = self.root / name
            if p.exists() or p.is_symlink():
                if p.is_symlink() or not p.is_socket(): raise ValueError("Unsafe IPC path")
                p.unlink()
        self.server.bind(str(self.root / ".control")); self.server.listen(8); self.server.settimeout(.2)
        self.ingest.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        self.ingest.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4*1024*1024)
        self.ingest.bind(str(self.root / ".ingest")); self.ingest.settimeout(.2)
        for name in (".control", ".ingest"): os.chmod(self.root / name, 0o600)
    def trigger(self, reason, detail):
        self.queue.emit("events", "fault_trigger", {"reason": reason, "detail": detail})
        try: return self.incidents.request(reason, detail)
        except RuntimeError as e: self.queue.emit("events", "incident_request_dropped", {"error": str(e)})
    def all_stats(self):
        s = self.queue.stats()
        s["transport_losses"] = dict(self.transport_losses)
        s["dropped"] += sum(self.transport_losses.values())
        return s
    def ingest_record(self, obj):
        kind, source, payload = obj["kind"], obj["source"], obj["payload"]
        if kind not in ("can","state","events") or not isinstance(payload,dict): return
        now = time.monotonic()
        for k, count in (("can_helper_ipc", payload.get("helper_dropped") if source == "can_health" else None),
                         ("ros_collector", payload.get("dropped") if source == "collector_health" else None),
                         ("can_socket", payload.get("socket_drops_counter") if kind == "can" else None)):
            if count is not None and count > self.transport_losses.get(k,0):
                self.queue.emit("events","transport_gap",{"stage":k,"counter":count,"previous":self.transport_losses.get(k,0),"detected_monotonic_ns":time.monotonic_ns()})
                self.transport_losses[k] = count
        if source == "can_health":
            self.bound = bool(payload.get("bound")); self.helper_at = now
            def flatten(v, prefix=""):
                out = {}
                if isinstance(v,dict):
                    for k,n in v.items(): out.update(flatten(n,prefix+"/"+k))
                elif type(v) is int and any(k in prefix for k in ("error","drop","bus_off","bus-off")): out[prefix]=v
                return out
            counters=flatten(payload.get("link",{}))
            delta={k:v-self.previous_can[k] for k,v in counters.items() if k in self.previous_can and v>self.previous_can[k]}
            self.previous_can=counters
            if self.armed:
                with self.detector_lock: self.detector.condition("can_counter_increase",bool(delta),now,delta)
            payload["counter_delta"]=delta
        if source=="collector_health": self.collector_at=now
        with self.detector_lock: self.detector.feed(source,payload,now)
        if "capture_monotonic_ns" in obj: payload=dict(payload,_capture_monotonic_ns=obj["capture_monotonic_ns"])
        self.queue.emit(kind,source,payload)
    def socket_reader(self):
        # Continues through the two-second shutdown tail.
        while not self.writer_stop.is_set():
            try:
                data,anc,_,_=self.ingest.recvmsg(65536,64)
                creds=[struct.unpack("3i",v) for l,k,v in anc if l==socket.SOL_SOCKET and k==socket.SCM_CREDENTIALS]
                if not creds or creds[0][1]!=os.getuid(): continue
                self.ingest_record(json.loads(data))
            except socket.timeout: pass
            except (OSError,ValueError,KeyError,TypeError) as e: self.queue.emit("events","ingest_invalid",{"error":str(e)})
    def client(self,sock):
        with sock:
            sock.settimeout(8)
            if struct.unpack("3i",sock.getsockopt(socket.SOL_SOCKET,socket.SO_PEERCRED,12))[1] not in (0,os.getuid()): return
            try:
                with sock.makefile("rb") as f: req=json.loads(f.readline(65536))
                op=req["command"]
                if op=="status": result=self.status()
                elif op=="save":
                    if not self.healthy: raise RuntimeError("Recorder unhealthy; no persistence acknowledgment")
                    j=self.trigger(str(req.get("reason","manual"))[:512],{})
                    result=j["result"] if j and j["done"].wait(7) else {"ok":False,"error":"Persistence pending; do not assume saved"}
                elif op=="activate":
                    st=self.status()
                    if not (st["healthy"] and st["collector_healthy"] and st["can_bound"]):
                        raise RuntimeError("Recorder not ready; activation blocked")
                    self.armed=self.queries_enabled=True
                    with self.detector_lock: self.detector.started=time.monotonic()
                    self.queue.emit("events","stack_start_requested",{}); result={"ok":True}
                elif op=="pause":
                    self.queries_enabled=False
                    atomic_json(self.root/".status.json",self.status())
                    self.queue.emit("events","queries_paused",{"reason":req.get("reason","stopping")}); result={"ok":True}
                elif op=="event": self.queue.emit("events","deploy",req.get("detail",{})); result={"ok":True}
                elif op=="stop":
                    self.queries_enabled=False; self.stop_reason="deploy_stop"; result={"ok":True}; self.stop_event.set()
                else: raise ValueError("Unknown command")
                sock.sendall((json.dumps(result)+"\n").encode())
            except Exception as e:
                try: sock.sendall((json.dumps({"ok":False,"error":str(e)})+"\n").encode())
                except OSError: pass
    def serve(self):
        while not self.stop_event.is_set():
            try: s,_=self.server.accept()
            except socket.timeout: continue
            if threading.active_count()>40: s.close(); continue
            threading.Thread(target=self.client,args=(s,),daemon=True).start()
    def ros_reader(self,child):
        for line in child.stdout:
            try: self.ingest_record(json.loads(line))
            except (ValueError,KeyError,TypeError): self.queue.emit("events","collector_output",{"line":line[:4096].decode(errors="replace")})
    def process_health(self):
        found={}
        try:
            tmux = subprocess.run(["tmux","has-session","-t","agv_control"],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,timeout=.5).returncode == 0
            if self.last_tmux is True and not tmux and self.armed:
                self.trigger("tmux_session_disappeared", {"session":"agv_control","action":"continue_passive_recording"})
            self.last_tmux = tmux
        except (OSError,subprocess.TimeoutExpired): pass
        for d in Path("/proc").iterdir():
            if not d.name.isdigit(): continue
            try:
                exe=(d/"exe").resolve().name
                if exe in ("agv2_control_node","socket_can_sender_node_exe","socket_can_receiver_node_exe","joy_node"):
                    found[d.name]={"executable":exe,"identity":identity(int(d.name))}
            except OSError: pass
        if self.last_processes is not None:
            for pid,v in self.last_processes.items():
                if found.get(pid)!=v and self.armed: self.trigger("control_process_exit",v)
            for pid,v in found.items():
                if pid not in self.last_processes: self.queue.emit("events","process_start",v)
        self.last_processes=found
        return found
    def status(self):
        now=time.monotonic()
        return {"format":"agv234.faultlog.status","identity":identity(os.getpid()),
          "session_id":self.store.session_id,"heartbeat_monotonic":now,"wall_ns":time.time_ns(),
          "durable":self.durable,"healthy":self.healthy,"can_bound":self.bound and now-self.helper_at<2,
          "collector_healthy":self.collector is None or now-self.collector_at<2,
          "disk":getattr(self,"disk",{}),
          "queries_enabled":self.queries_enabled and self.healthy and (self.collector is None or now-self.collector_at<2),
          "directory":str(self.store.path) if self.store.path else None,"queue":self.all_stats(),"closed":False}
    def writer(self):
        flush=sync=limits=retry=0.; gap=None
        while not self.writer_stop.is_set() or self.queue.stats()["queued_can"] or self.queue.stats()["queued_state_events"]:
            now=time.monotonic(); batch=self.queue.pop(1000)
            if now<retry: self.queue.discard(batch,"disk_unavailable"); time.sleep(.02); continue
            try:
                with self.store_lock:
                    if gap is not None:
                        self.store.recover_write(self.all_stats())
                        self.queue.emit("events","write_gap",{"start_monotonic_ns":gap,"end_monotonic_ns":time.monotonic_ns(),"stats":self.queue.stats()})
                        gap=None
                    if self.store.path is None: self.store.open_segment()
                    for r in batch:
                        r["session_id"]=self.store.session_id; self.store.append(r)
                        if self.store.due(): self.store.close_segment("rotation",self.all_stats()); self.store.open_segment()
                    if now-flush>=self.cfg["flush_seconds"]: self.store.flush(); flush=now
                    if now-sync>=self.cfg["fsync_seconds"]: self.store.flush(True); sync=now; self.durable=True
                    if now-limits>=1:
                        total,free=self.store.enforce_limits(); limits=now
                        self.disk={"bytes":total,"free_bytes":free}
                self.healthy=True
            except (OSError,ValueError) as e:
                self.healthy=False; self.queue.discard(batch,"write_error")
                gap=gap or time.monotonic_ns(); retry=now+self.cfg["retry_seconds"]
                with self.store_lock: self.store.abandon_handles()
                import syslog
                syslog.syslog(syslog.LOG_WARNING,"FAULT LOGGER WRITE FAILURE: "+str(e))
            time.sleep(.005)
    def journal(self):
        try:
            proc=subprocess.Popen(["journalctl","-k","-f","-n","0","--output=json","--no-pager"],stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
            self.journal_child=proc
            for line in proc.stdout:
                try: msg=json.loads(line).get("MESSAGE","")
                except ValueError:
                    self.queue.emit("events","kernel_journal_unavailable",{"detail":line[:512].decode(errors="replace")}); continue
                if any(s in msg.lower() for s in ("usb","gs_usb",self.cfg["interface"].lower())):
                    self.queue.emit("events","kernel",{"message":msg,"scope":"CAN_or_USB_system_event"})
                    if "disconnect" in msg.lower() and self.armed: self.trigger("usb_disconnect_observed",{"message":msg})
        except OSError as e: self.queue.emit("events","kernel_journal_unavailable",{"error":str(e)})
    def run(self):
        def stop(sig,_):
            self.stop_reason="signal_"+str(sig); self.queries_enabled=False; self.stop_event.set()
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(sig,stop)
        for f in (self.writer,self.serve,self.socket_reader,self.journal):
            t=threading.Thread(target=f,daemon=True); self.threads.append(t); t.start()
        self.queue.emit("events","recorder_start",self.metadata)
        try:
            while not self.stop_event.wait(.2):
                if self.collector and (self.child is None or self.child.poll() is not None):
                    if self.child is not None: self.queue.emit("events","collector_exit",{"returncode":self.child.returncode})
                    self.child=subprocess.Popen([self.collector,str(self.root)],stdout=subprocess.PIPE,stderr=subprocess.STDOUT,start_new_session=True)
                    threading.Thread(target=self.ros_reader,args=(self.child,),daemon=True).start()
                if self.armed:
                    with self.detector_lock:
                        self.detector.condition("can_helper_missing",time.monotonic()-self.helper_at>2,time.monotonic())
                        self.detector.condition("ros_collector_missing",self.collector is not None and time.monotonic()-self.collector_at>2,time.monotonic())
                        self.detector.tick()
                if not hasattr(self,"last_health") or time.monotonic()-self.last_health>=1:
                    self.last_health=time.monotonic(); now=time.monotonic_ns()
                    with self.queue.lock: received=dict(self.queue.received)
                    freshness={topic:{"state":"not_seen" if topic not in received else "stale" if now-received[topic]>500000000 else "fresh",
                               "age_ms":(now-received[topic])/1e6 if topic in received else None}
                               for topic in ("/joy","/cmd_vel","/wheel_command","/agv2/chassis_telemetry","/Battery_SOC_STATE")}
                    self.queue.emit("events","health",{"processes":self.process_health(),"freshness":freshness,"disk":getattr(self,"disk",{}),**self.status()})
                try: atomic_json(self.root/".status.json",self.status())
                except OSError: self.healthy=False
        finally:
            self.queries_enabled=False
            # Keep helper lease fresh through passive two-second tail.
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                try: atomic_json(self.root/".status.json",self.status())
                except OSError: pass
                time.sleep(.2)
            if self.child and self.child.poll() is None:
                self.child.terminate()
                try: self.child.wait(timeout=1)
                except subprocess.TimeoutExpired: self.child.kill()
            if getattr(self,"journal_child",None): self.journal_child.terminate()
            self.queue.emit("events","recorder_exit",{"reason":self.stop_reason})
            self.writer_stop.set(); self.threads[0].join(2)
            complete=self.incidents.stop(4)
            with self.store_lock:
                try: self.store.close_segment(self.stop_reason,self.all_stats())
                except OSError: pass
                self.store.release()
            st=self.status(); st.update(closed=True,healthy=False,incident_worker_closed=complete)
            try: atomic_json(self.root/".status.json",st)
            except OSError: pass
            self.server.close(); self.ingest.close()
