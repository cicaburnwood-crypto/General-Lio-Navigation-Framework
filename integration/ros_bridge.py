"""Read-only visualization + explicit manager/inspection/initialpose calls."""
import io
import json
import math
from pathlib import Path as FilePath
import re
import struct
import threading
import time

from PIL import Image as PillowImage
import rclpy
from nav_msgs.msg import Path, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParametersAtomically
from rcl_interfaces.msg import Log
from chassis_localization_msgs.srv import Relocalize, IsValid
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scan_planner_msgs.srv import DetectDoorOpening
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import MarkerArray
from door_opening_server.backend import image_to_rgb
from fastlio_restart import restart_fastlio


class RosBridge(Node):
    def __init__(self, image_topic, pcd_map=None, fastlio_restart=None):
        rclpy.init()
        super().__init__('door_gui_bridge')
        self.lock = threading.RLock()
        self.detector_lock = threading.Lock()
        self.start_lock = threading.Lock()
        self.fastlio_restart = fastlio_restart or restart_fastlio
        self.lio_odom_stamp = self.lio_cloud_stamp = 0.
        self.data, self.timestamps, self.rgb = {}, {}, None
        self.pcd_map = str(pcd_map or FilePath(__file__).resolve().parents[1]/'src/bringup/maps/hkclr_office/3D/map.pcd')
        self.localization_epoch = 0
        self.localization_check = None
        self.localization_check_epoch = 0
        self.localization_check_started = 0.
        self.localization = dict(state='CHECKING', native_valid=None, reference=None)
        # Node.clients is a read-only ROS property, not application storage.
        self.patrol_clients = {s: self.create_client(Trigger, s) for s in ('/patrol/start','/patrol/cancel')}
        self.mission_parameters = self.create_client(SetParametersAtomically, '/patrol_manager/set_parameters_atomically')
        self.detector = self.create_client(DetectDoorOpening, '/inspection/detection_door_opening')
        self.nav = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.relocalizer = self.create_client(Relocalize, '/localizer/relocalize')
        self.localization_validator = self.create_client(IsValid, '/localizer/relocalize_check')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(String, '/patrol/status', lambda m: self.on_json('patrol_status', m),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, '/inspection/detection_door_opening/details',
                                 lambda m: self.on_json('door_details', m), 10)
        self.create_subscription(Log, '/rosout', self.on_localization_log, 100)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', lambda m: self.touch('imu'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/sensors/cloud', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/Odometry', self.on_lio_odom, qos_profile_sensor_data)
        self.create_subscription(Path, '/plan', lambda m: self.on_path('global', m), 10)
        # This workspace's MPPI publishes the actual optimized trajectory as
        # MarkerArray, not /local_plan. Do not confuse the transformed global
        # reference path with MPPI's selected local trajectory.
        self.create_subscription(MarkerArray, '/trajectories', self.on_trajectory, 10)
        self.create_timer(.1, self.update_pose)
        self.create_timer(.5, self.poll_localization)
        # Node.executor only keeps a weak reference. Own the spinning executor
        # explicitly so it survives initialization and remains available at close.
        self.spin_executor = MultiThreadedExecutor(num_threads=2)
        self.spin_executor.add_node(self)
        self.thread = threading.Thread(target=self.spin_executor.spin, daemon=True)
        self.thread.start()

    def touch(self, name):
        with self.lock:
            self.timestamps[name] = time.monotonic()

    def on_json(self, key, message):
        try:
            value = json.loads(message.data)
            with self.lock:
                self.data[key] = value
        except ValueError:
            pass

    def update_pose(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time())
            stamp = t.header.stamp.sec+t.header.stamp.nanosec/1e9
            age = self.get_clock().now().nanoseconds/1e9-stamp
            if not 0 <= age <= 1.0:
                raise ValueError('map → base_link is stale')
            p, q = t.transform.translation, t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            with self.lock:
                self.data['pose'] = [p.x,p.y,p.z,yaw]
                self.timestamps['pose'] = time.monotonic()-age
        except Exception:
            with self.lock:
                self.data.pop('pose', None)
                self.timestamps.pop('pose', None)

    def poll_localization(self):
        # The existing code=1 API bypasses validity. Always query code=0.
        with self.lock:
            if self.localization['state'] in {'RESTARTING', 'WAITING_LIO', 'SUBMITTING'}:
                return
            future = self.localization_check
            if future is not None:
                if future.done():
                    self.localization_check = None
                    if self.localization_check_epoch == self.localization_epoch:
                        try:
                            valid = bool(future.result().valid)
                            self.localization.update(native_valid=valid)
                            self.localization.pop('check_error', None)
                            if self.localization['state'] not in {'RESTARTING','WAITING_LIO','SUBMITTING','ERROR','UNKNOWN'}:
                                self.localization['state'] = 'LOCALIZED' if valid else 'MATCHING'
                        except Exception as exc:
                            self.localization.update(native_valid=None, check_error=str(exc))
                elif time.monotonic()-self.localization_check_started > 3:
                    self.localization_check = None
                    future.cancel()
                    self.localization.update(native_valid=None, check_error='定位状态查询超时')
                else:
                    return
            if not self.localization_validator.service_is_ready():
                self.localization.update(native_valid=None, check_error='原定位状态服务未连接')
                return
            if self.localization['state'] == 'SUBMITTING':
                return
            self.localization_check_epoch = self.localization_epoch
            self.localization_check_started = time.monotonic()
            self.localization_check = self.localization_validator.call_async(IsValid.Request(code=0))

    def on_localization_log(self, message):
        if message.name != 'localizer.chassis_3d_localizer_node' or '[align]' not in message.msg:
            return
        match = re.search(r'rough_rms=([\d.eE+-]+) refine_rms=([\d.eE+-]+).*decision=(\w+)', message.msg)
        if match is None:
            return
        with self.lock:
            self.localization['alignment'] = dict(rough_rms=float(match[1]),
                refine_rms=float(match[2]), decision=match[3])
            self.timestamps['alignment'] = time.monotonic()

    def on_path(self, prefix, message):
        points = [[p.pose.position.x,p.pose.position.y,p.pose.position.z] for p in message.poses]
        self.store_path(prefix, message.header.frame_id, points)

    def map_transform(self, frame):
        if frame == 'map':
            return ((1,0,0),(0,1,0),(0,0,1)), (0,0,0)
        t = self.tf.lookup_transform('map', frame, Time())
        q, p = t.transform.rotation, t.transform.translation
        x,y,z,w = q.x,q.y,q.z,q.w
        return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y))), (p.x,p.y,p.z)

    def store_path(self, prefix, frame, points):
        try:
            rotation, translation = self.map_transform(frame)
            points = [[sum(r[j]*xyz[j] for j in range(3))+v for r,v in zip(rotation,translation)]
                      for xyz in points if all(math.isfinite(v) for v in xyz)]
        except Exception:
            return
        with self.lock:
            self.data[prefix+'_path'] = points
            self.data[prefix+'_revision'] = self.data.get(prefix+'_revision',0)+1

    def on_trajectory(self, message):
        markers = sorted((m for m in message.markers if m.ns == 'Optimal Trajectory' and m.action == 0),
                         key=lambda m:m.id)
        if not markers:
            self.store_path('local', 'map', [])
            return
        frame = markers[0].header.frame_id
        if any(m.header.frame_id != frame for m in markers):
            return
        self.store_path('local', frame, [[m.pose.position.x,m.pose.position.y,m.pose.position.z] for m in markers])

    def on_lio_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        if all(math.isfinite(v) for v in (p.x,p.y,p.z,q.x,q.y,q.z,q.w)) and sum(v*v for v in (q.x,q.y,q.z,q.w)) > .5:
            with self.lock:
                self.lio_odom_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9

    def on_cloud(self, msg):
        self.touch('lidar')
        if msg.width*msg.height and msg.data:
            with self.lock:
                self.lio_cloud_stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        try:
            rotation, translation = self.map_transform(msg.header.frame_id)
            fields = {f.name:f for f in msg.fields}
            if any(fields[a].datatype != 7 for a in ('x','y','z')):
                return
            points=[]
            count=msg.width*msg.height
            for i in range(0,count,max(1,count//2500)):
                off=(i//msg.width)*msg.row_step+(i%msg.width)*msg.point_step
                xyz=[struct.unpack_from('>f' if msg.is_bigendian else '<f',msg.data,off+fields[a].offset)[0] for a in ('x','y','z')]
                if all(math.isfinite(v) for v in xyz):
                    points.append([sum(r[j]*xyz[j] for j in range(3))+v for r,v in zip(rotation,translation)])
            with self.lock:
                self.data['world_points']=points
                self.data['world_revision']=self.data.get('world_revision',0)+1
                self.timestamps['cloud']=time.monotonic()
        except Exception:
            pass

    def on_image(self, message):
        try:
            rgb = image_to_rgb(message)
            output=io.BytesIO()
            image = PillowImage.fromarray(rgb)
            image.thumbnail((960,540))
            image.save(output,format='JPEG',quality=80)
            with self.lock:
                self.rgb=output.getvalue()
                self.timestamps['rgb']=time.monotonic()
                self.data['rgb_error']=''
        except Exception as exc:
            with self.lock:
                self.data['rgb_error']=str(exc)

    def snapshot(self):
        with self.lock:
            out=dict(self.data)
            out.update({key+'_age_s':round(time.monotonic()-stamp,3) for key,stamp in self.timestamps.items()})
            out['nav_ready']=self.nav.server_is_ready()
            out['door_ready']=self.detector.service_is_ready()
            localization = dict(self.localization)
            fresh = out.get('pose_age_s') is not None and 0 <= out['pose_age_s'] <= 1.0
            localization['pose_fresh'] = fresh
            # Display-only: an accepted request is not a localization result.
            # Never substitute the operator reference or a cached stale TF.
            if not fresh or localization.get('native_valid') is not True or localization['state'] in {'RESTARTING','WAITING_LIO','SUBMITTING','ERROR','UNKNOWN'}:
                out['pose'] = None
            out['localization'] = localization
            return out

    def jpeg(self):
        with self.lock:
            return self.rgb

    @staticmethod
    def wait(future, timeout):
        deadline=time.monotonic()+timeout
        while not future.done():
            if time.monotonic()>deadline:
                raise RuntimeError('Request response timed out; inspect manager status before retrying')
            time.sleep(.02)
        return future.result()

    def trigger(self, service):
        client=self.patrol_clients[service]
        if not client.wait_for_service(timeout_sec=1):
            raise RuntimeError(service+' is not running')
        reply=self.wait(client.call_async(Trigger.Request()),5)
        return dict(success=reply.success,message=reply.message)

    def start_patrol(self, mission=None):
        # Serialize staging + start across GUI requests. Queue execution remains
        # entirely in manager; use its existing native parameter and Trigger APIs.
        with self.start_lock:
            if not self.mission_parameters.wait_for_service(timeout_sec=1):
                raise RuntimeError('Patrol manager parameter service is not running')
            request = SetParametersAtomically.Request(parameters=[Parameter(
                'runtime_mission_json', value=json.dumps(mission, allow_nan=False) if mission is not None else ''
            ).to_parameter_msg()])
            reply = self.wait(self.mission_parameters.call_async(request), 5)
            if not reply.result.successful:
                raise ValueError(reply.result.reason)
            return self.trigger('/patrol/start')

    def detect(self):
        with self.lock:
            state=self.data.get('patrol_status',{}).get('state','IDLE')
        if state not in {'IDLE','SUCCEEDED','FAILED','CANCELED'}:
            raise ValueError('During patrol only manager calls the detector; diagnostics remain visible')
        if not self.detector_lock.acquire(blocking=False):
            raise ValueError('Inspection already in progress')
        try:
            if not self.detector.service_is_ready():
                raise ValueError('Door detector is not running')
            reply=self.wait(self.detector.call_async(DetectDoorOpening.Request()),60)
            return dict(success=True,opening_percent=float(reply.opening_percent))
        finally:
            self.detector_lock.release()

    def initial_pose(self, data):
        values=[float(data.get(k,0)) for k in ('x','y','z','yaw')]
        if not all(math.isfinite(v) for v in values):
            raise ValueError('Initial pose must be finite')
        with self.start_lock:
            with self.lock:
                state=self.data.get('patrol_status',{}).get('state','IDLE')
            if state not in {'IDLE','SUCCEEDED','FAILED','CANCELED'}:
                raise ValueError('Cancel the active manager task before changing localization')
            if not self.relocalizer.wait_for_service(timeout_sec=1):
                raise ValueError('原生 /localizer/relocalize 服务未连接')
            with self.lock:
                self.localization_epoch += 1
                self.localization = dict(state='RESTARTING', native_valid=False, reference=values)
                self.data.pop('pose', None)
                self.timestamps.pop('pose', None)
                self.timestamps.pop('alignment', None)
            request = Relocalize.Request(pcd_path=self.pcd_map, x=values[0], y=values[1],
                z=values[2], yaw=values[3], pitch=0., roll=0.)
            try:
                restart = self.fastlio_restart()
                # Source timestamps, not reception time: queued old-session
                # messages must not satisfy the new-session readiness wait.
                fresh_after = self.get_clock().now().nanoseconds/1e9
                self.tf.clear()
                with self.lock:
                    self.localization.update(state='WAITING_LIO', fastlio_restart=restart)
                deadline = time.monotonic()+30
                while True:
                    with self.lock:
                        ready = self.lio_odom_stamp > fresh_after and self.lio_cloud_stamp > fresh_after
                    if ready:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError('FAST-LIO 已重启，但 30 秒内未收到新会话的里程计与点云；未提交地图匹配')
                    time.sleep(.05)
            except Exception as exc:
                with self.lock:
                    self.localization.update(state='ERROR', native_valid=False, error=str(exc))
                raise
            with self.lock:
                self.localization.update(state='SUBMITTING')
            try:
                reply = self.wait(self.relocalizer.call_async(request), 15)
            except Exception:
                with self.lock:
                    self.localization.update(state='UNKNOWN', native_valid=None,
                        error='请求结果未知；未将参考位置当作定位成功，请检查原定位服务')
                raise
            with self.lock:
                self.localization.update(state='MATCHING' if reply.success else 'ERROR',
                    native_valid=False, message=reply.message)
            if not reply.success:
                raise ValueError(reply.message)
            return dict(success=True, message='FAST-LIO 已重启并收到新里程计与点云；参考位姿已交给原定位模块，正在粗配准 / 精配准，尚不代表定位成功。')

    def close(self):
        self.spin_executor.shutdown()
        self.thread.join(timeout=3)
        self.destroy_node()
        rclpy.shutdown()
