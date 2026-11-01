#!/usr/bin/env python3
"""Run ONLY via sudo unshare --net. Creates vcan_test in that isolated namespace."""
import json, os, pwd, select, signal, socket, struct, subprocess, sys, tempfile, time, unittest
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(REPO/"scripts"))
from faultlog.storage import atomic_json
from faultlog.settings import identity
class VirtualCanTests(unittest.TestCase):
    def test_minimal_capability_tx_without_loopback_queries_and_disconnect(self):
        # Hard guard: never alter or transmit on a namespace with physical interfaces.
        before=json.loads(subprocess.check_output(["ip","-j","link","show"]))
        self.assertEqual({x["ifname"] for x in before},{"lo"},"Must use a NEW unshare --net namespace")
        subprocess.run(["ip","link","add","vcan_test","type","vcan"],check=True)
        subprocess.run(["ip","link","set","vcan_test","up"],check=True)
        user=pwd.getpwnam("hkclr")
        with tempfile.TemporaryDirectory(prefix="faultlog-vcan-") as temp:
            root=Path(temp); os.chown(root,user.pw_uid,user.pw_gid)
            ingest=socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM); ingest.bind(str(root/".ingest")); ingest.setblocking(False)
            os.chown(root/".ingest",user.pw_uid,user.pw_gid)
            def lease(enabled=True):
                atomic_json(root/".status.json",{"identity":identity(os.getpid()),"healthy":True,
                    "heartbeat_monotonic":time.monotonic(),"queries_enabled":enabled})
                os.chown(root/".status.json",user.pw_uid,user.pw_gid)
            lease()
            args=["setpriv","--reuid="+str(user.pw_uid),"--regid="+str(user.pw_gid),"--init-groups",
                  "--bounding-set=-all,+net_raw","--inh-caps=-all,+net_raw","--ambient-caps=-all,+net_raw","--no-new-privs",
                  sys.executable,"-B",str(REPO/"scripts/chassis_fault_log.py"),"helper","--root",str(root),
                  "--interface","vcan_test","--ids","322","321"]
            helper=subprocess.Popen(args)
            tx=socket.socket(socket.PF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
            tx.setsockopt(socket.SOL_CAN_RAW,socket.CAN_RAW_LOOPBACK,0); tx.bind(("vcan_test",))
            records=[]
            def pump(seconds,enabled=True):
                end=time.monotonic()+seconds
                while time.monotonic()<end:
                    lease(enabled)
                    while select.select([ingest],[],[],0)[0]: records.append(json.loads(ingest.recv(65536)))
                    time.sleep(.02)
            try:
                pump(.8)
                status=(Path("/proc")/str(helper.pid)/"status").read_text()
                caps={line.split(":")[0]:int(line.split()[1],16) for line in status.splitlines() if line.startswith(("CapEff:","CapBnd:","CapAmb:"))}
                self.assertTrue(all(v==0x2000 for v in caps.values()),caps)
                for _ in range(5): tx.send(struct.pack("=IB3x8s",0x555,8,b"TEST_TX!"))
                pump(2.5)
                frames=[r["payload"] for r in records if r["kind"]=="can"]
                self.assertTrue(any(f["id"]==0x555 and f["origin"]=="tx_observed" for f in frames))
                self.assertFalse(any(f["id"]==0x555 and f["origin"]=="bus_rx" for f in frames))
                queries=[r["payload"] for r in records if r["source"]=="query" and r["payload"].get("status")=="send_ok"]
                self.assertGreaterEqual(len(queries),4)
                for i in (321,322):
                    times=[q["sent_monotonic"] for q in queries if q["id"]==i]
                    self.assertTrue(all(b-a>=.99 for a,b in zip(times,times[1:])))
                self.assertFalse(any(r["source"]=="query" and r["payload"].get("status")=="reply" for r in records))
                # No transmissions after pause has propagated.
                pump(.2,False); count=len([r for r in records if r["source"]=="query" and r["payload"].get("status")=="send_ok"])
                pump(1.2,False)
                self.assertEqual(count,len([r for r in records if r["source"]=="query" and r["payload"].get("status")=="send_ok"]))
                subprocess.run(["ip","link","set","vcan_test","down"],check=True)
                pump(1.5,False)
                health=[r["payload"] for r in records if r["source"]=="can_health"]
                self.assertFalse(health[-1]["bound"])
                subprocess.run(["ip","link","set","vcan_test","up"],check=True); pump(1.5,False)
                health=[r["payload"] for r in records if r["source"]=="can_health"]
                self.assertTrue(health[-1]["bound"], [r for r in records[-30:] if r["kind"] != "can"])
                print("VERIFIED: CAP_NET_RAW only, AF_PACKET TX with loopback disabled, query rate, echo exclusion, pause, interface down/up")
            finally:
                helper.terminate(); helper.wait(timeout=3); tx.close(); ingest.close()
if __name__=="__main__": unittest.main(verbosity=2)
