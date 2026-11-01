"""Real DDS service calls and RGB publications, including the actual epoch-11 model."""
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import urllib.error
import urllib.request

import numpy as np
from PIL import Image as PILImage
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from scan_planner_msgs.srv import DetectDoorOpening
from ament_index_python.packages import get_package_share_directory

from door_opening_server.backend import OnnxDoorClassifier
from door_opening_server.ros_node import DoorOpeningServer, SERVICE_NAME

DATA=Path(__file__).parent/'data'


def wait_until(predicate,timeout=5):
    deadline=time.monotonic()+timeout
    while not predicate():
        assert time.monotonic()<deadline,'Timed out waiting for ROS event'
        time.sleep(.005)


@pytest.fixture(scope='module')
def model():
    path=Path(get_package_share_directory('door_opening_server'))/'models/door_epoch11.onnx'
    result=OnnxDoorClassifier(path,'cpu',4)
    opened=np.array(PILImage.open(DATA/'open.png').convert('RGB'))
    closed=np.array(PILImage.open(DATA/'closed.png').convert('RGB'))
    assert result.score(opened)>=.47 and result.score(closed)<.47
    return result


class Harness:
    def __init__(self,predictor):
        rclpy.init()
        self.server=DoorOpeningServer(predictor=predictor,
            parameter_overrides=[Parameter('image_reliability',value='reliable')])
        self.camera=Node('inspection_test_camera_client')
        self.pub=self.camera.create_publisher(Image,'/camera/color/image_raw',10)
        self.details=[]
        self.camera.create_subscription(String,SERVICE_NAME+'/details',
            lambda message:self.details.append(json.loads(message.data)),10)
        self.client=self.camera.create_client(DetectDoorOpening,SERVICE_NAME)
        self.executor=MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(self.server);self.executor.add_node(self.camera)
        self.spin=threading.Thread(target=self.executor.spin,daemon=True);self.spin.start()
        assert self.client.wait_for_service(timeout_sec=5)
        wait_until(lambda:self.pub.get_subscription_count()>0)
        self.stop_stream=threading.Event();self.thread=None
        self.images={name:np.array(PILImage.open(DATA/f'{name}.png').convert('RGB')) for name in ('open','closed')}

    def stream(self,pattern=lambda elapsed:'open',stamp_mode='fresh'):
        self.stop_stream.clear()
        fixed_stamp=self.camera.get_clock().now().to_msg()
        started=time.monotonic()
        def publish():
            while not self.stop_stream.is_set():
                elapsed=time.monotonic()-started
                pixels=self.images[pattern(elapsed)]
                message=Image();message.height,message.width=pixels.shape[:2]
                message.encoding='rgb8';message.step=message.width*3;message.data=pixels.tobytes()
                if stamp_mode=='fresh':message.header.stamp=self.camera.get_clock().now().to_msg()
                elif stamp_mode=='stale':
                    message.header.stamp=self.camera.get_clock().now().to_msg();message.header.stamp.sec-=5
                elif stamp_mode=='repeat':message.header.stamp=fixed_stamp
                self.pub.publish(message)
                self.stop_stream.wait(1/30)
        self.thread=threading.Thread(target=publish,daemon=True);self.thread.start()

    def halt_stream(self):
        self.stop_stream.set()
        if self.thread:self.thread.join(timeout=2)
        self.thread=None

    def call(self):
        started=time.monotonic();future=self.client.call_async(DetectDoorOpening.Request())
        wait_until(future.done,timeout=15)
        value=future.result().opening_percent
        assert value in (0.0,100.0)
        return value,time.monotonic()-started

    def close(self):
        self.halt_stream();self.server.stop();self.executor.shutdown()
        self.server.destroy_node();self.camera.destroy_node();rclpy.shutdown();self.spin.join(timeout=3)


@pytest.fixture
def harness(model):
    result=Harness(model)
    try:yield result
    finally:result.close()


def test_all_five_real_open_frames_return_100_and_next_request_uses_new_frames(harness):
    harness.stream()
    value,elapsed=harness.call()
    assert value==100.0 and elapsed>=1.0
    harness.halt_stream()
    value,elapsed=harness.call()
    assert value==0.0 and elapsed>=1.0


def test_mixed_real_frames_use_mean_instead_of_individual_veto(harness):
    harness.stream(lambda elapsed:'closed' if .37<=elapsed<=.73 else 'open')
    value,elapsed=harness.call()
    wait_until(lambda:bool(harness.details))
    scores=harness.details[-1]['probabilities']
    assert len(scores)==5 and any(p<.47 for p in scores) and any(p>=.47 for p in scores)
    assert value==(100.0 if sum(scores)/len(scores)>.47 else 0.0)
    assert elapsed>=1.0


@pytest.mark.parametrize('count',[1,2,3,4])
def test_sparse_early_frames_are_inferred_after_window_ends(harness,count):
    class Scores:
        threshold=.47
        def __init__(self):self.calls=0
        def score(self,rgb):
            self.calls+=1
            return .8
    predictor=Scores()
    harness.server.predictor=predictor
    future=harness.client.call_async(DetectDoorOpening.Request())
    wait_until(lambda:harness.server._window is not None)
    for index in range(count):
        pixels=harness.images['open']
        message=Image();message.height,message.width=pixels.shape[:2]
        message.encoding='rgb8';message.step=message.width*3;message.data=pixels.tobytes()
        message.header.stamp=harness.camera.get_clock().now().to_msg()
        harness.pub.publish(message)
        wait_until(lambda:len(harness.server._window.samples)==index+1)
        time.sleep(.01)
    wait_until(future.done,15)
    assert future.result().opening_percent==100.0 and predictor.calls==count
    wait_until(lambda:bool(harness.details))
    assert harness.details[-1]['sample_count']==count
    assert harness.details[-1]['mean_probability']==pytest.approx(.8)


@pytest.mark.parametrize('mode',['stale','repeat','zero'])
def test_old_repeated_or_unstamped_images_cannot_open_door(harness,mode):
    harness.stream(stamp_mode=mode)
    assert harness.call()[0]==0.0


def test_concurrent_request_returns_zero_without_consuming_first_capture(harness):
    harness.stream()
    first=harness.client.call_async(DetectDoorOpening.Request())
    wait_until(lambda:harness.server._window is not None)
    value,elapsed=harness.call()
    assert value==0.0 and elapsed<.8
    wait_until(first.done,15)
    assert first.result().opening_percent==100.0


def test_inference_exception_returns_zero(harness):
    class Failed:
        threshold=.47
        def score(self,rgb):raise RuntimeError('deliberate inference failure')
    harness.server.predictor=Failed()
    harness.stream()
    assert harness.call()[0]==0.0


def test_installed_launch_file_and_real_service_process():
    rclpy.init()
    client_node=Node('launch_smoke_client')
    client=client_node.create_client(DetectDoorOpening,SERVICE_NAME)
    log_path=Path(os.environ.get('TEST_RESULTS_DIR','/tmp'))/'launch_smoke.log'
    with log_path.open('w') as log:
        process=subprocess.Popen(['ros2','launch','door_opening_server','door_opening.launch.py','provider:=cpu'],
                                 stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            assert client.wait_for_service(timeout_sec=15),'Installed launch did not advertise service'
            future=client.call_async(DetectDoorOpening.Request())
            rclpy.spin_until_future_complete(client_node,future,timeout_sec=10)
            assert future.done() and future.result().opening_percent==0.0
            assert (SERVICE_NAME,['scan_planner_msgs/srv/DetectDoorOpening']) in client_node.get_service_names_and_types()
        finally:
            try:os.killpg(process.pid,signal.SIGINT)
            except ProcessLookupError:pass
            try:process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
            client_node.destroy_node();rclpy.shutdown()


def test_gui_camera_preview_and_real_ros_service_call(harness):
    from door_opening_server.gui import DoorOpeningGui,create_http_server
    gui=DoorOpeningGui();harness.executor.add_node(gui)
    http=create_http_server(gui,'127.0.0.1',0)
    thread=threading.Thread(target=http.serve_forever,daemon=True);thread.start()
    url=f'http://127.0.0.1:{http.server_port}'
    def get(path):
        with urllib.request.urlopen(url+path,timeout=20) as response:return response.read()
    def post(path,payload):
        request=urllib.request.Request(url+path,data=json.dumps(payload).encode(),headers={'Content-Type':'application/json'})
        with urllib.request.urlopen(request,timeout=20) as response:return json.load(response)
    try:
        assert gui.client.wait_for_service(timeout_sec=5)
        assert '打开 Camera' in get('/').decode()
        assert post('/api/camera',{'open':True})['camera_open']
        harness.stream()
        wait_until(lambda:json.loads(get('/api/status'))['camera_ready'])
        import io
        image=PILImage.open(io.BytesIO(get('/api/frame.jpg')))
        assert image.width==640 and image.height==360
        result=post('/api/detect',{})
        assert result['opening_percent']==100.0 and result['duration_s']>=1.0
        assert json.loads(get('/api/status'))['last_result']==result
        assert not post('/api/camera',{'open':False})['camera_open']
        with pytest.raises(urllib.error.HTTPError) as error:post('/api/detect',{})
        assert error.value.code==409
        request=urllib.request.Request(url+'/api/camera',data=b'{"open":true}',headers={'Content-Type':'application/json','Origin':'https://example.com'})
        with pytest.raises(urllib.error.HTTPError) as error:urllib.request.urlopen(request)
        assert error.value.code==403
    finally:
        http.shutdown();http.server_close();thread.join(timeout=3)
        harness.executor.remove_node(gui);gui.destroy_node()
