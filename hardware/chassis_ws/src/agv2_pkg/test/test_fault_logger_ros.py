#!/usr/bin/env python3
"""Isolated ROS tests: domain 93, localhost only; never launches control/real CAN."""
import json, os, signal, subprocess, sys, tempfile, threading, time, unittest
from pathlib import Path
os.environ["ROS_DOMAIN_ID"]="93"
os.environ["ROS_LOCALHOST_ONLY"]="1"
os.environ["RMW_IMPLEMENTATION"]="rmw_fastrtps_cpp"
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"scripts"))
from faultlog.settings import load
from faultlog.storage import read_json
from faultlog.supervisor import request
import rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
REPO=Path(__file__).resolve().parents[1]
WS=REPO.parent.parent if REPO.parent.name == "src" else REPO.parent
COLLECTOR=Path(os.environ.get("FAULT_LOG_COLLECTOR", str(WS/"install_release_humble/agv2_pkg/lib/agv2_pkg/fault_log_collector")))
class RosTests(unittest.TestCase):
    def test_gid_graph_service_and_signal_tail(self):
        with tempfile.TemporaryDirectory(prefix="faultlog-ros93-") as temp:
            root=Path(temp)/"continuous"
            code="import sys;sys.path.insert(0,sys.argv[1]);from faultlog.settings import load;from faultlog.supervisor import Runtime;c,m=load(sys.argv[2],sys.argv[3],sys.argv[4],'vcan_test');c.update(min_free_bytes=1,segment_seconds=1,incident_pre_seconds=2,incident_post_seconds=2,incident_extend_seconds=4);Runtime(c,m,sys.argv[5]).run()"
            proc=subprocess.Popen([sys.executable,"-B","-c",code,str(REPO/"scripts"),str(REPO/"config/fault_logger.yaml"),
                                   str(REPO/"deploy/chassis_full_manual_test.yaml"),str(root),str(COLLECTOR)])
            nodes=[]
            try:
                for _ in range(100):
                    try:
                        if request(root,"status")["collector_healthy"]: break
                    except OSError: pass
                    time.sleep(.1)
                else: self.fail("collector did not start")
                rclpy.init()
                n1=rclpy.create_node("fault_test_source_1"); n2=rclpy.create_node("fault_test_source_2"); nodes=[n1,n2]
                p1=n1.create_publisher(Twist,"/cmd_vel",10); p2=n2.create_publisher(Twist,"/cmd_vel",10)
                pj=n1.create_publisher(Joy,"/joy",10)
                time.sleep(2)
                self.assertEqual(n1.count_subscribers("/to_can_bus"),0)
                for _ in range(40):
                    p1.publish(Twist()); p2.publish(Twist()); pj.publish(Joy(axes=[0.,.25],buttons=[0,1]))
                    rclpy.spin_once(n1,timeout_sec=.01); time.sleep(.04)
                service=n1.create_client(Trigger,"/agv2/save_fault_log")
                self.assertTrue(service.wait_for_service(timeout_sec=3))
                future=service.call_async(Trigger.Request())
                rclpy.spin_until_future_complete(n1,future,timeout_sec=9)
                self.assertTrue(future.done())
                response=future.result(); self.assertTrue(response.success,response.message)
                self.assertIn("post_capturing",response.message)
                request(root,"pause"); proc.send_signal(signal.SIGTERM); proc.wait(timeout=10)
                self.assertEqual(proc.returncode,0)
                states=[json.loads(line) for f in root.glob("segment_*/state.jsonl") for line in f.read_text().splitlines()]
                gids={r["payload"]["_publisher_gid"] for r in states if r["source"]=="/cmd_vel"}
                self.assertEqual(len(gids),2)
                self.assertTrue(all(len(x)==48 for x in gids))
                events=[json.loads(line) for f in root.glob("segment_*/events.jsonl") for line in f.read_text().splitlines()]
                graphs=[r["payload"] for r in events if r["source"]=="ros_graph"]
                self.assertTrue(any(len(g.get("/cmd_vel",[]))==2 for g in graphs))
                self.assertTrue(read_json(root/".status.json")["closed"])
                self.assertTrue(any(r["source"]=="recorder_exit" for r in events))
                packages=list((Path(temp)/"incidents").glob("segment_*/manifest.json"))
                self.assertEqual(len(packages),1)
                print("VERIFIED: two per-message GIDs, graph cardinality, no to_can_bus subscription, durable service acknowledgment, TERM tail")
            finally:
                if proc.poll() is None: proc.kill(); proc.wait()
                for n in nodes: n.destroy_node()
                if rclpy.ok(): rclpy.shutdown()
if __name__=="__main__": unittest.main(verbosity=2)
