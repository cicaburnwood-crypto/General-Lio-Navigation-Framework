"""Run ONLY in network-isolated container. Fake CAN messages, no SocketCAN."""
import heapq
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from can_msgs.msg import Frame
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from agv2_pkg.msg import ChassisTelemetry

assert os.environ.get('ROS_DOMAIN_ID') == '227', 'Isolated test domain required'
workspace = Path('/work')
output = workspace / 'test_results'
output.mkdir(exist_ok=True)
node_log = (output / 'fake_driver.log').open('w')
binary = workspace / 'install/agv2_pkg/lib/agv2_pkg/agv2_control_node'
params = workspace / 'src/agv2_pkg/deploy/chassis_full_manual_test.yaml'
rclpy.init()
n = Node('socket_can_sender')
rx = n.create_publisher(Frame, '/from_can_bus', 1000)
joy_pub = n.create_publisher(Joy, '/joy', 10)
cmd_pub = n.create_publisher(Twist, '/cmd_vel', 10)
start = time.monotonic()
data = []
wire = []
pending = []
serial = 0
angles = {321: 0, 322: 0}
states = {513: 0, 514: 0}
wheel_speed = {513: 0, 514: 0}
last_tx = {}
overlaps = []

def on_tx(f):
    global serial
    now = time.monotonic() - start
    wire.append((now, f.id, list(f.data)))
    if f.id in angles:
        if f.id in last_tx and now-last_tx[f.id][0] < .019:
            overlaps.append((now, f.id, f.data[0], last_tx[f.id]))
        last_tx[f.id] = (now, f.data[0])
        # Outage while commands continue, recovery; a second outage with no input.
        if 4 <= now < 5 or 8 <= now < 9:
            return
        reply = Frame(id=f.id, dlc=8, data=[0]*8)
        reply.data[0] = f.data[0]
        if f.data[0] == 0x94:
            reply.data[4:8] = list(angles[f.id].to_bytes(4, 'little', signed=True))
        elif f.data[0] == 0xa4:
            angles[f.id] = int.from_bytes(bytes(f.data[4:8]), 'little', signed=True)
        elif f.data[0] == 0x9a:
            reply.data[1] = 30
        elif f.data[0] == 0x88 and f.id == 321:
            # Legacy rear enable does not always produce an ACK.
            return
        serial += 1
        heapq.heappush(pending, (time.monotonic()+.002, serial, reply))
    elif f.id in states:
        states[f.id] = {6: 1, 7: 3, 15: 7}.get(f.data[0], states[f.id])
        wheel_speed[f.id] = int.from_bytes(bytes(f.data[3:7]), 'little', signed=True)

def on_state(msg):
    data.append((time.monotonic()-start, msg.commanded_front_speed_mps,
                 msg.commanded_rear_speed_mps, msg.front_steer_feedback_age_ms,
                 msg.rear_steer_feedback_age_ms, msg.front_steer_loopback_count,
                 msg.rear_steer_loopback_count, msg.mode))

n.create_subscription(Frame, '/to_can_bus', on_tx, 1000)
n.create_subscription(ChassisTelemetry, '/agv2/chassis_telemetry', on_state, 1000)
proc = subprocess.Popen([str(binary), '--ros-args', '--params-file', str(params)],
                        stdout=node_log, stderr=subprocess.STDOUT)
try:
    next_input = 0
    while time.monotonic()-start < 14:
        t = time.monotonic()-start
        if t >= next_input:
            next_input = t+.04
            j = Joy(axes=[0.0]*8, buttons=[0]*15)
            if 1.0 <= t < 1.2: j.buttons[8] = 1
            if 1.5 <= t < 1.7: j.buttons[9] = 1
            if 12 <= t < 12.2: j.buttons[7] = 1
            joy_pub.publish(j)
            if t >= 2 and not 8 <= t < 10.5:
                cmd_pub.publish(Twist(linear=__import__('geometry_msgs.msg', fromlist=['Vector3']).Vector3(x=.2)))
            for cid, sw in states.items():
                f = Frame(id=cid-128, dlc=8, data=[sw, 0, 0, 0]+list(wheel_speed[cid].to_bytes(4,'little',signed=True)))
                rx.publish(f)
        while pending and pending[0][0] <= time.monotonic():
            _, _, frame = heapq.heappop(pending)
            last_tx.pop(frame.id, None)
            rx.publish(frame)
        rclpy.spin_once(n, timeout_sec=.001)
        if proc.poll() is not None: raise RuntimeError('Driver exited unexpectedly')

    def moving(a,b): return any(max(abs(r[1]),abs(r[2]))>.05 for r in data if a<=r[0]<b)
    def stopped(a,b):
        rows=[r for r in data if a<=r[0]<b]
        return len(rows)>5 and all(abs(r[1])+abs(r[2])<1e-9 for r in rows)
    tests = {
        'startup_no_nonzero_output': stopped(.4,1),
        'normal_execution': moving(2.5,3.8),
        'loss_stops_wheels_without_mode_cancel': stopped(4.5,4.95) and all(r[7]==2 for r in data if 4.5<r[0]<4.95),
        'auto_recovered_without_button': moving(6.5,7.8),
        'no_replay_when_input_stopped': stopped(9.8,10.4),
        'new_input_resumes': moving(11,11.8),
        'emergency_not_overridden': stopped(12.5,13.8),
        'zero_feedback_not_discarded': all(r[5]==0 and r[6]==0 for r in data),
        'one_inflight_per_motor': not overlaps,
    }
    result = dict(tests=tests, sample_count=len(data), overlaps=overlaps,
                  network='isolated --network none', physical_devices='none')
    (output/'driver_recovery_ros.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    assert all(tests.values()), result
finally:
    proc.send_signal(signal.SIGINT)
    try: proc.wait(timeout=3)
    except subprocess.TimeoutExpired: proc.kill(); proc.wait()
    node_log.close()
    n.destroy_node()
    rclpy.shutdown()
