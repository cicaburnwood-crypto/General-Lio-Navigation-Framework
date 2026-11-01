#!/usr/bin/env python3
"""No-hardware lifecycle and I/O fault tests."""
import json, os, signal, subprocess, sys, tempfile, threading, time, unittest
from pathlib import Path
from unittest.mock import patch
REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/"scripts"))
from faultlog.settings import load, identity
from faultlog.storage import read_json
from faultlog.supervisor import Runtime, request
def cfg(tmp):
    c,m=load(REPO/"config/fault_logger.yaml",REPO/"deploy/chassis_full_manual_test.yaml",str(Path(tmp)/"continuous"),"vcan_test")
    c.update(min_free_bytes=1,retry_seconds=.1,fsync_seconds=.05,segment_seconds=.2,incident_post_seconds=.1,incident_extend_seconds=.2)
    return c,m
CODE="import sys;sys.path.insert(0,sys.argv[1]);from faultlog.settings import load;from faultlog.supervisor import Runtime;c,m=load(sys.argv[2],sys.argv[3],sys.argv[4],'vcan_test');c.update(min_free_bytes=1,segment_seconds=.2,incident_post_seconds=.1,incident_extend_seconds=.2);Runtime(c,m,None).run()"
def start(root):
    return subprocess.Popen([sys.executable,"-B","-c",CODE,str(REPO/"scripts"),str(REPO/"config/fault_logger.yaml"),
                             str(REPO/"deploy/chassis_full_manual_test.yaml"),str(root)],stdout=subprocess.DEVNULL)
def wait_ready(root):
    for _ in range(70):
        try:
            st=request(root,"status")
            if st["healthy"] and st["durable"]: return st
        except OSError: pass
        time.sleep(.1)
    raise AssertionError("Recorder readiness failed")
class LifecycleTests(unittest.TestCase):
    def test_int_term_hup_close_once_and_parent_independence(self):
        for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):
            with self.subTest(signal=sig), tempfile.TemporaryDirectory() as t:
                root=Path(t)/"continuous"; p=start(root)
                try:
                    wait_ready(root); request(root,"event",detail={"stage":"test_running"})
                    p.send_signal(sig); p.wait(timeout=10); self.assertEqual(p.returncode,0)
                    st=read_json(root/".status.json"); self.assertTrue(st["closed"])
                    events=[json.loads(line) for f in root.glob("segment_*/events.jsonl") for line in f.read_text().splitlines()]
                    self.assertEqual(sum(r["source"]=="recorder_exit" for r in events),1)
                finally:
                    if p.poll() is None: p.kill(); p.wait()
    def test_kill_and_recovery_does_not_modify_raw_records(self):
        with tempfile.TemporaryDirectory() as t:
            root=Path(t)/"continuous"; p=start(root); wait_ready(root)
            request(root,"event",detail={"stage":"before_KILL"}); time.sleep(1.1)
            p.kill(); p.wait()
            before={str(f):f.read_bytes() for f in root.glob("segment_*/*.jsonl")}
            p=start(root)
            try:
                wait_ready(root)
                for name,data in before.items(): self.assertEqual(Path(name).read_bytes(),data)
                self.assertTrue(any(read_json(f)["status"]=="interrupted" for f in root.glob("segment_*/manifest.json")))
                request(root,"stop"); p.wait(timeout=10)
            finally:
                if p.poll() is None: p.kill(); p.wait()
    def test_write_error_pause_gap_and_recovery(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=cfg(t); rt=Runtime(c,m,None)
            failed=threading.Event(); allow=threading.Event(); original=rt.store.append
            def flaky(record):
                if not allow.is_set(): failed.set(); raise OSError("injected disk write failure")
                return original(record)
            with patch.object(rt.store,"append",flaky):
                thread=threading.Thread(target=rt.writer); thread.start()
                rt.queue.emit("state","/joy",{"axes":[0]})
                self.assertTrue(failed.wait(2)); self.assertFalse(rt.healthy)
                allow.set(); rt.queue.emit("state","/joy",{"axes":[1]})
                time.sleep(.5); self.assertTrue(rt.healthy)
                rt.writer_stop.set(); thread.join(2)
            rt.incidents.stop()
            rt.store.close_segment("test",rt.all_stats()); rt.store.release(); rt.server.close(); rt.ingest.close()
            events=[json.loads(line) for f in (Path(t)/"continuous").glob("segment_*/events.jsonl") for line in f.read_text().splitlines()]
            self.assertTrue(any(r["source"]=="write_gap" for r in events))
            self.assertGreater(rt.all_stats()["dropped"],0)
if __name__=="__main__": unittest.main(verbosity=2)

