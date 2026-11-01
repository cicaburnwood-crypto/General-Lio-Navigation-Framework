#!/usr/bin/env python3
"""Offline tests: no ROS connection, hardware, CAN socket or robot control."""
import json, os, socket, struct, sys, tempfile, threading, time, unittest
from pathlib import Path
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from faultlog.settings import load
from faultlog.storage import Store, StoragePressure, read_json
from faultlog.collectors import RecordQueue, decode_raw_can
from faultlog.faults import Detector, decode_status
from faultlog.can_helper import Queries, QUERY
from faultlog.incidents import Incidents
REPO=Path(__file__).resolve().parents[1]
def config(tmp, **kw):
    c,m=load(REPO/"config/fault_logger.yaml",REPO/"deploy/chassis_full_manual_test.yaml",
             str(Path(tmp)/"continuous"),"vcan0")
    c.update(min_free_bytes=1,**kw)
    m["recorder_started_monotonic_ns"]=time.monotonic_ns()
    return c,m
def record(n,kind="can"):
    return {"sequence":n,"monotonic_ns":time.monotonic_ns(),"wall_time_ns":time.time_ns(),
            "session_id":"test","source":"test","kind":kind,"payload":{"n":n}}
class StorageTests(unittest.TestCase):
    def test_30_segments_retention_pins_symlinks_and_history(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=config(t); s=Store(c,metadata=m)
            history=Path(t)/"oldbag"; history.mkdir(); (history/"db.db3").touch()
            for i in range(30):
                s.open_segment(); s.append(record(i)); s.close_segment("test")
            self.assertEqual(len(s.segments()),24)
            self.assertEqual([m["sequence"] for _,m in s.segments()],list(range(7,31)))
            target=s.segments()[0][0]; s.pins.add(target.name)
            self.assertFalse(s._delete_closed(target)); s.pins.clear()
            (target/"escape").symlink_to(history,target_is_directory=True)
            self.assertFalse(s._delete_closed(target))
            self.assertTrue((history/"db.db3").exists())
            s.open_segment(); self.assertFalse(s._delete_closed(s.path))
            s.close_segment("test"); s.release()
    def test_sequence_rotation_no_loss_or_duplication(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=config(t,max_segments=100,segment_bytes=2000); s=Store(c,metadata=m)
            for n in range(100):
                s.append(record(n))
                if s.due(): s.close_segment("rotation")
            s.close_segment("done")
            records=[json.loads(line)["sequence"] for p,_ in s.segments() for line in (p/"can.jsonl").read_text().splitlines()]
            self.assertEqual(records,list(range(100))); s.release()
    def test_lock_and_recovery_partial_evidence(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=config(t); s=Store(c,metadata=m); s.open_segment(); p=s.path
            with self.assertRaises(RuntimeError): Store(c)
            s.flush(True); s.abandon_handles(); s.release()
            with (p/"can.jsonl").open("ab") as f: f.write(b'{"partial":')
            s=Store(c); self.assertEqual(read_json(p/"manifest.json")["status"],"interrupted")
            self.assertEqual((p/"can.jsonl").read_bytes(),b'{"partial":'); s.release()
    def test_capacity_and_free_space(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=config(t,max_total_bytes=30000); s=Store(c,metadata=m)
            s.open_segment(); s.append(record(1)); s.close_segment("test")
            with patch("faultlog.storage.shutil.disk_usage") as du:
                du.return_value.free=0
                with self.assertRaises(StoragePressure): s.enforce_limits()
            s.release()
    def test_queue_priority_and_gap(self):
        c={"can_queue_records":2,"state_queue_records":3}; q=RecordQueue(c)
        q.emit("state","/joy",{"zero":0})
        for n in range(4): q.emit("can","can0",{"n":n})
        q.emit("events","error",{})
        out=q.pop(); self.assertEqual([r["source"] for r in out],["/joy","can0","can0","error"])
        self.assertEqual(q.stats()["dropped"],2); self.assertIsNotNone(q.stats()["first_loss_monotonic_ns"])
    def test_isolated_roots(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            ca,_=config(a); cb,_=config(b); sa=Store(ca); sb=Store(cb)
            sa.open_segment(); sb.open_segment()
            self.assertNotEqual(sa.root,sb.root)
            self.assertFalse(sa._delete_closed(sb.path))
            sa.close_segment("done"); sb.close_segment("done"); sa.release(); sb.release()

class QueryTests(unittest.TestCase):
    def test_zero_and_fault_bits(self):
        self.assertEqual(decode_status(QUERY)["bus_voltage_v"],0)
        v=decode_status(bytes.fromhex("9aff701764ff0119"))
        self.assertEqual(v["temperature_c"],-1); self.assertTrue(v["low_voltage"]); self.assertTrue(v["over_temperature"])
        self.assertEqual(v["unknown_error_bits"],16)
    def test_whitelist_rate_timeout_late_and_echo(self):
        q=Queries([0x142,0x141]); q.next={0x142:0,0x141:.5}; sent=[]; events=[]
        def emit(k,s,p): events.append(p)
        for i in range(301): q.step(i/100,True,sent.append,emit)
        self.assertEqual(len(sent),7)
        self.assertTrue(all(struct.unpack("=IB3x8s",f)[1:]==(8,QUERY) for f in sent))
        self.assertTrue(any(x["status"]=="timeout" for x in events))
        f=decode_raw_can(struct.pack("=IB3x8s",0x142,8,QUERY),socket.MSG_DONTROUTE)
        n=len(events); q.receive(f,3.01,emit); self.assertEqual(len(events),n)
        f["origin"]="bus_rx"; q.receive(f,3.01,emit); self.assertEqual(events[-1]["status"],"reply")
        q.receive(f,3.8,emit); self.assertEqual(events[-1]["status"],"late_or_unsolicited")
        n=len(sent); q.step(99,False,sent.append,emit); self.assertEqual(len(sent),n)
    def test_invalid_ids_and_rate(self):
        for ids in ([0x201,0x142],[0x141,0x141]):
            with self.assertRaises(ValueError): Queries(ids)
        with self.assertRaises(ValueError): Queries([0x141,0x142],.1)
    def test_send_fail(self):
        q=Queries([0x142,0x141]); q.next=dict.fromkeys(q.ids,0); out=[]
        def fail(_): raise OSError("down")
        q.step(0,True,fail,lambda k,s,p:out.append(p))
        self.assertTrue(all(x["status"]=="send_failed" for x in out))
    def test_delayed_bind_staggers_first_queries(self):
        q=Queries([0x142,0x141]); out=[]; send=[]
        emit=lambda k,s,p: out.append(p)
        q.step(100,False,None,emit)
        q.step(110,True,send.append,emit)
        self.assertEqual(len(send),1)
        q.step(110.49,True,send.append,emit); self.assertEqual(len(send),1)
        q.step(110.5,True,send.append,emit); self.assertEqual(len(send),2)
    def test_error_frame_timestamp_and_echo(self):
        frame=struct.pack("=IB3x8s",0x20000004,8,b"\0"*8)
        f=decode_raw_can(frame,0,[(socket.SOL_SOCKET,35,struct.pack("@ll",5,7))])
        self.assertTrue(f["error"]); self.assertEqual(f["kernel_time_ns"],5000000007)

class DetectorTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.cfg,_=config(self.tmp.name)
        self.events=[]; self.d=Detector(self.cfg,lambda a,b:self.events.append((a,b)),now=0)
    def tearDown(self): self.tmp.cleanup()
    def test_real_combination_not_ros_interference(self):
        t={"mode":1,"front_wheel_feedback_seen":True,"rear_wheel_feedback_seen":True,
           "front_wheel_feedback_age_ms":0,"rear_wheel_feedback_age_ms":0,
           "front_wheel_status_word":0x233,"rear_wheel_status_word":0x88,"rear_wheel_error_code":6,
           "front_steer_feedback_seen":True,"rear_steer_feedback_seen":True,
           "front_steer_feedback_age_ms":900,"rear_steer_feedback_age_ms":1000}
        self.d.feed("/agv2/chassis_telemetry",t,20); self.d.feed("/joy",{},20)
        self.d.tick(20)
        keys=[e[0] for e in self.events]
        self.assertIn("rear_wheel_fault",keys); self.assertIn("front_steer_stale",keys)
        self.assertFalse(any("multiple_publishers" in k for k in keys))
        self.assertFalse(any(k=="joy_stale_active" for k in keys))
        detail=next(e[1] for e in self.events if e[0]=="rear_wheel_fault")
        self.assertEqual(detail["error_603f"],6); self.assertEqual(detail["namespace"],"wheel_CANopen")
    def test_multi_publisher_persistence_and_rearm(self):
        self.d.feed("ros_graph",{"/cmd_vel":[{"gid":"A"},{"gid":"B"}]},1)
        self.d.tick(1); self.d.tick(2.1); self.d.tick(3)
        self.assertEqual(sum(k=="multiple_publishers:/cmd_vel" for k,_ in self.events),1)
        self.d.feed("ros_graph",{},4); self.d.tick(4); self.d.tick(10)
        self.d.feed("ros_graph",{"/cmd_vel":[{},{}]},11); self.d.tick(11); self.d.tick(12.1)
        self.assertEqual(sum(k=="multiple_publishers:/cmd_vel" for k,_ in self.events),2)
    def test_initial_missing_is_distinct(self):
        self.d.tick(9); self.assertFalse(self.events)
        self.d.tick(11); keys=[k for k,_ in self.events]
        self.assertIn("front_steer_startup_missing",keys); self.assertNotIn("front_steer_stale",keys)

class IncidentTests(unittest.TestCase):
    def test_cross_segment_window_merge_and_durable_ack(self):
        with tempfile.TemporaryDirectory() as t:
            c,m=config(t,incident_pre_seconds=60,incident_post_seconds=2,incident_extend_seconds=3)
            s=Store(c,metadata=m); lock=threading.RLock()
            for n in range(3):
                s.open_segment(); s.append(record(n)); s.close_segment("rotate")
            s.open_segment()
            inc=Incidents(c,s,lock,lambda:{"dropped":0},lambda *a:None)
            a=inc.request("rear_error_6"); self.assertTrue(a["done"].wait(3)); self.assertTrue(a["result"]["ok"])
            with lock: s.append(record(3)); s.flush(True)
            time.sleep(2.1); inc.stop()
            found=inc.store.segments(); self.assertEqual(len(found),1)
            p,mf=found[0]; self.assertEqual(mf["status"],"closed")
            nums=[json.loads(x)["sequence"] for x in (p/"can.jsonl").read_text().splitlines()]
            self.assertEqual(nums,[0,1,2,3])
            self.assertLess(mf["incident"]["pre_window_available_seconds"],60)
            s.close_segment("done"); s.release()
if __name__=="__main__": unittest.main(verbosity=2)
