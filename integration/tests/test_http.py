import asyncio
import importlib.util
import json
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace

from aiohttp.test_utils import TestClient, TestServer
from PIL import Image
import yaml

# This test runs both from the source tree and the isolated test image.
server_path = Path(__file__).resolve().parents[1]/'web_server.py'
if not server_path.exists():
    server_path=Path('/integration/web_server.py')
spec=importlib.util.spec_from_file_location('door_web',server_path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
from test_gate import config


def test_gui_settings_and_preview_cannot_dispatch(tmp_path, monkeypatch):
    web_dir=tmp_path/'web';web_dir.mkdir();(web_dir/'index_2d.html').write_text('<html>2D fixture</html>')
    monkeypatch.setattr(module,'ROOT',tmp_path)
    nav_dir=tmp_path/'src/navigation/config';nav_dir.mkdir(parents=True)
    footprint='[[0.28, 0.18], [0.28, -0.18], [-0.28, -0.18], [-0.28, 0.18]]'
    (nav_dir/'nav2_mppi.yaml').write_text(yaml.safe_dump({'controller_server':{'ros__parameters':{'FollowPath':{'vx_max':.45,'vy_max':.25,'wz_max':.7}}},'global_costmap':{'global_costmap':{'ros__parameters':{'footprint':footprint}}}}))
    image=tmp_path/'map.pgm';Image.new('L',(8,8),0).save(image)
    map_file=tmp_path/'map.yaml';map_file.write_text(yaml.safe_dump(dict(image=image.name,resolution=.1,origin=[0,0,0],occupied_thresh=.65)))
    mission=tmp_path/'mission.yaml';mission.write_text(yaml.safe_dump(config()))
    args=SimpleNamespace(mission=str(mission),map=str(map_file))
    headers={'X-Scan-Nav':'operator'}
    async def run():
        async with TestClient(TestServer(module.create_app(args))) as client:
            response=await client.get('/')
            assert response.status==200 and '2D fixture' in await response.text()
            response=await client.get('/api/status');status=await response.json()
            assert response.status==200 and status['preview'] and not status['armed']
            assert status['deployment']=='AL'
            assert status['navigation_caps']==dict(max_vx_mps=.45,max_vy_mps=.25,max_wz_radps=.7)
            assert status['robot_footprint']==yaml.safe_load(footprint)
            response=await client.get('/api/maps/map/grid');grid=await response.json()
            assert grid['width']==grid['height']==8
            assert grid['resolution']==.1 and grid['origin']==[0,0,0]
            assert grid['frame_id']=='map'
            response=await client.get(grid['image_url'])
            assert response.status==200 and response.content_type=='image/png'
            assert Image.open(io.BytesIO(await response.read())).getpixel((0,0))==0
            response=await client.get('/api/maps/not-the-current-map/grid')
            assert response.status==404
            response=await client.get('/api/settings');settings=await response.json()
            rev=settings['patrol']['revision']
            value=config();value['points'][1]['position'][1]=5.
            response=await client.post('/api/settings',headers=headers,json=dict(category='patrol',value=value,revision=rev))
            assert response.status==200
            saved=await response.json()
            assert saved['patrol']['value']['points'][1]['position'][1]==5.
            response=await client.post('/api/settings',headers=headers,json=dict(category='patrol',value=config(),revision=rev))
            assert response.status==400 # Do not overwrite a concurrent editor.
            response=await client.post('/api/action',headers=headers,json=dict(action='start_patrol',revision=saved['patrol']['revision']))
            assert response.status==400 and 'Offline' in (await response.json())['error']
            response=await client.get('/api/maps/map/points')
            assert len((await response.json())['points'])==64
    asyncio.run(run())


def test_raster_preserves_map_geometry_and_occupancy(tmp_path):
    image=Image.new('L',(3,2));image.putdata([0,255,205,127,80,250]);image.save(tmp_path/'test.pgm')
    path=tmp_path/'test.yaml'
    values=dict(image='test.pgm',resolution=.05,origin=[-13.6,-41.5,math.pi/2],occupied_thresh=.65,free_thresh=.25,negate=0)
    path.write_text(yaml.safe_dump(values))
    metadata,png=module.map_grid(path)
    assert metadata==dict(width=3,height=2,resolution=.05,origin=values['origin'],frame_id='map',ground_z=0.)
    with Image.open(io.BytesIO(png)) as raster:
        assert list(raster.getdata())==[0,255,255,127,0,255]
    values['negate']=1;path.write_text(yaml.safe_dump(values))
    with Image.open(io.BytesIO(module.map_grid(path)[1])) as raster:
        assert list(raster.getdata())==[255,0,0,127,127,0]


def test_realtime_http_uses_memory_only_and_saved_start_still_uses_yaml(tmp_path, monkeypatch):
    monkeypatch.setattr(module,'ROOT',tmp_path)
    (tmp_path/'web').mkdir()
    nav=tmp_path/'src/navigation/config';nav.mkdir(parents=True)
    (nav/'nav2_mppi.yaml').write_text(yaml.safe_dump({'controller_server':{'ros__parameters':{'FollowPath':{'vx_max':.45,'vy_max':.25,'wz_max':.7}}}}))
    Image.new('L',(2,2),255).save(tmp_path/'map.pgm')
    map_file=tmp_path/'map.yaml';map_file.write_text(yaml.safe_dump(dict(image='map.pgm',resolution=.1,origin=[0,0,0],occupied_thresh=.65)))
    mission=tmp_path/'mission.yaml';mission.write_text(yaml.safe_dump(config()))
    before=mission.read_bytes();mtime=mission.stat().st_mtime_ns
    files=set(tmp_path.rglob('*'))
    calls=[]
    class FakeBridge:
        def start_patrol(self, value=None):
            calls.append(value)
            return dict(success=True,message='accepted by manager')
        def initial_pose(self, value):
            calls.append(('reference',value))
            return dict(success=True,message='reference accepted; awaiting native match')
    args=SimpleNamespace(mission=str(mission),map=str(map_file))
    headers={'X-Scan-Nav':'operator'}
    async def run():
        async with TestClient(TestServer(module.create_app(args,FakeBridge()))) as client:
            value=config();value['points'][0]['type']='blue';value['points'][1]['type']='red'
            response=await client.post('/api/action',headers=headers,json=dict(action='start_realtime',mission=value))
            assert response.status==200 and (await response.json())['ok']
            assert calls==[value]
            invalid=dict(value,points=[])
            response=await client.post('/api/action',headers=headers,json=dict(action='start_realtime',mission=invalid))
            assert response.status==400 and len(calls)==1
            revision=module.mission_view(mission)['revision']
            response=await client.post('/api/action',headers=headers,json=dict(action='start_patrol',revision=revision))
            assert response.status==200 and calls[-1] is None
            response=await client.post('/api/action',headers=headers,json=dict(action='relocalize',x=1,y=2,yaw=.3))
            assert response.status==200
            assert (await response.json())['result']['message']=='reference accepted; awaiting native match'
            assert calls[-1][0]=='reference'
    asyncio.run(run())
    assert mission.read_bytes()==before and mission.stat().st_mtime_ns==mtime
    assert set(tmp_path.rglob('*'))==files # No YAML, backup, or temporary output.
