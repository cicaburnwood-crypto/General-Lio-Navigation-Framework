#!/usr/bin/env python3
"""Copied GUI HTTP adapter. No waypoint sequencing or velocity publishing here."""
import argparse
import asyncio
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import time

from aiohttp import web
from PIL import Image
import yaml
from rgb_recording import RecordingControl, atomic_json

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/patrol_manager'))
from patrol_manager.door_mission import parse_mission


def mission_view(path):
    content = path.read_text()
    return dict(value=yaml.safe_load(content), revision=hashlib.sha256(content.encode()).hexdigest(),
                file=str(path), content=content)


def map_points(path):
    data = yaml.safe_load(path.read_text())
    with Image.open(path.parent/data['image']) as image:
        pixels = image.convert('L')
        width, height = pixels.size
        resolution = float(data['resolution'])
        ox, oy, yaw = data['origin']
        points = []
        step = max(1, math.ceil(math.sqrt(width*height/40000)))
        for y in range(0, height, step):
            for x in range(0, width, step):
                p = pixels.getpixel((x, y))/255
                occupied = p if data.get('negate', 0) else 1-p
                if occupied <= data['occupied_thresh']:
                    continue
                xx, yy = (x+.5)*resolution, (height-y-.5)*resolution
                points.append([ox+math.cos(yaw)*xx-math.sin(yaw)*yy,
                               oy+math.sin(yaw)*xx+math.cos(yaw)*yy, 0.0])
    return dict(points=points, ground_z=0.0)


def map_grid(path):
    """Read the AL map in map_server's trinary convention; display only."""
    data = yaml.safe_load(path.read_text())
    with Image.open(path.parent/data['image']) as source:
        gray = source.convert('L')
        # Match Nav2 trinary occupied/free thresholds (strict comparisons).
        palette = []
        for value in range(256):
            occupancy = value/255 if data.get('negate', 0) else 1-value/255
            palette.append(0 if occupancy > data['occupied_thresh'] else
                           255 if occupancy < data.get('free_thresh', .25) else 127)
        raster = gray.point(palette)
        output = io.BytesIO()
        raster.save(output, format='PNG')
    return dict(width=raster.width, height=raster.height,
                resolution=float(data['resolution']), origin=list(data['origin']),
                frame_id='map', ground_z=0.0), output.getvalue()


def create_app(args, bridge=None):
    mission = Path(args.mission).resolve()
    map_file = Path(args.map).resolve()
    map_name = map_file.stem
    points = map_points(map_file)
    grid, grid_png = map_grid(map_file)
    nav_config = yaml.safe_load((ROOT/'src/navigation/config/nav2_mppi.yaml').read_text())
    controller = nav_config['controller_server']['ros__parameters']['FollowPath']
    navigation_caps = dict(max_vx_mps=controller['vx_max'], max_vy_mps=controller['vy_max'],
                           max_wz_radps=controller['wz_max'])
    footprint = nav_config.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters', {}).get('footprint', [])
    if isinstance(footprint, str):
        footprint = yaml.safe_load(footprint)
    save_lock = asyncio.Lock()
    recorder = RecordingControl(ROOT/'recordings/rgb')

    @web.middleware
    async def errors(request, handler):
        try:
            if request.method != 'GET':
                if request.headers.get('X-Scan-Nav') != 'operator':
                    raise web.HTTPForbidden(text='Operator header required')
                origin = request.headers.get('Origin')
                if origin and origin not in (f'http://{request.host}', f'https://{request.host}'):
                    raise web.HTTPForbidden(text='Cross-origin command rejected')
            return await handler(request)
        except (ValueError, KeyError, TypeError, RuntimeError) as exc:
            return web.json_response(dict(ok=False, error=str(exc)), status=400)

    async def status(request):
        d = bridge.snapshot() if bridge else {}
        native = d.get('patrol_status')
        active = native is not None and native['state'] not in {'IDLE','SUCCEEDED','FAILED','CANCELED'}
        # Compatibility fields preserve the original viewer, not SCAN/PCT behavior.
        out = dict(nav2_door=True, preview=bridge is None, mode='localization' if d.get('pose') else 'idle',
            map=map_name, maps=[map_name], map_mtime_ns=map_file.stat().st_mtime_ns,
            armed=active, control='manager' if active else 'none', error='',
            processes={'localization':bool(d.get('pose')), 'init':False, 'mapping':False,
                       'global':False, 'planner':d.get('nav_ready',False), 'base':False, 'patrol':native is not None},
            patrol_startup=dict(state='READY' if native else 'OFFLINE', stage='Nav2 + manager + RGB door'),
            patrol_status=native, door_details=d.get('door_details'), door_ready=d.get('door_ready',False),
            nav_ready=d.get('nav_ready',False),
            lidar_age_s=None, imu_age_s=None, cloud_age_s=None, pose_age_s=None, pose=None, pose_frame='map',
            rgb_age_s=None, rgb_error='No RGB topic received',
            raw_points=[], world_points=[], global_path=[], local_path=[],
            raw_revision=0, world_revision=0, global_revision=0, local_revision=0,
            active_waypoint_index=None, active_waypoint_total=None, active_waypoint=None,
            waypoint_mtime_ns=mission.stat().st_mtime_ns, map_saved=True, tiles_saved=False,
            tomogram_current=False, tomogram_available=False, terrain_worker_age_s=None, terrain_job=None,
            relocalization_ready=False, relocalization_started=False, pose_file_path=None,
            recording=dict(active=False), sensor_recovery='offline',
            navigation_caps=navigation_caps, robot_footprint=footprint, deployment='AL')
        out.update(d)
        out['recording'] = recorder.status()
        return web.json_response(out)

    async def settings(request):
        if request.method == 'GET':
            return web.json_response(dict(patrol=mission_view(mission)))
        data = await request.json()
        if data.get('category') != 'patrol':
            raise ValueError('This copy edits manager missions only; Nav2 settings are YAML files')
        async with save_lock:
            old = mission_view(mission)
            if data.get('revision') != old['revision']:
                raise ValueError('Mission changed in another window; reload first')
            value = data['value']
            parse_mission(value)
            content = yaml.safe_dump(value, allow_unicode=True, sort_keys=False)
            backup = mission.with_suffix(f'.{time.time_ns()}.bak')
            backup.write_text(old['content'])
            temporary = mission.with_suffix('.tmp')
            temporary.write_text(content)
            os.replace(temporary, mission)
        return web.json_response(dict(ok=True, patrol=dict(mission_view(mission), backup=str(backup))))

    async def action(request):
        body = await request.json()
        if bridge is None:
            raise ValueError('Offline GUI preview: no ROS calls or motion. Start integration with --ros first.')
        name = body['action']
        if name == 'start_patrol':
            if body.get('revision') != mission_view(mission)['revision']:
                raise ValueError('Reload the current mission before starting')
            parse_mission(mission_view(mission)['value'])
            result = await asyncio.to_thread(bridge.start_patrol)
        elif name == 'start_realtime':
            # Validate an in-memory snapshot, not the saved YAML. No settings,
            # backup, temporary file, or route persistence on this branch.
            value = body['mission']
            parse_mission(value)
            result = await asyncio.to_thread(bridge.start_patrol, value)
        elif name in ('cancel_patrol', 'stop', 'estop'):
            result = await asyncio.to_thread(bridge.trigger, '/patrol/cancel')
        elif name == 'relocalize':
            result = await asyncio.to_thread(bridge.initial_pose, body)
        elif name == 'detect_door':
            result = await asyncio.to_thread(bridge.detect)
        else:
            raise ValueError('Not a Nav2 door workflow operation; no SCAN/PCT command was sent')
        if not result['success']:
            raise ValueError(result.get('message', 'Operation failed'))
        return web.json_response(dict(ok=True, result=result))

    async def waypoints(request):
        value = mission_view(mission)['value']
        pts = [p['position'] for p in value['points'] if all(isinstance(v, (float,int)) for v in p['position'])]
        return web.json_response(dict(waypoints=pts, mtime_ns=mission.stat().st_mtime_ns))

    async def rgb(request):
        if bridge is None:
            raise web.HTTPServiceUnavailable(text='Offline preview: no camera')
        data = bridge.jpeg()
        if data is None:
            raise web.HTTPServiceUnavailable(text='Waiting for RGB')
        return web.Response(body=data, content_type='image/jpeg', headers={'Cache-Control':'no-store'})

    async def rgb_stream(request):
        if bridge is None:
            raise web.HTTPServiceUnavailable(text='Offline preview: no camera')
        response = web.StreamResponse(headers={'Content-Type':'multipart/x-mixed-replace; boundary=frame', 'Cache-Control':'no-store'})
        await response.prepare(request)
        try:
            while True:
                data = bridge.jpeg()
                if data:
                    await response.write(b'--frame\r\nContent-Type: image/jpeg\r\n\r\n'+data+b'\r\n')
                await asyncio.sleep(.15)
        except (ConnectionError, asyncio.CancelledError):
            return response

    async def index(request):
        return web.FileResponse(ROOT/'web/index_2d.html', headers={'Cache-Control':'no-store'})

    async def recording_command(request):
        if bridge is None:
            raise ValueError('Offline preview: no camera recording')
        result = await asyncio.to_thread(recorder.command, request.match_info['action'])
        return web.json_response(result)

    async def recordings(request):
        return web.json_response(dict(recordings=recorder.recordings()),headers={'Cache-Control':'no-store'})

    async def recording_file(request):
        path=recorder.file(request.match_info['name'])
        return web.FileResponse(path,headers={'Content-Disposition':f'attachment; filename="{path.name}"'})

    async def recording_received(request):
        path=recorder.file(request.match_info['name'])
        body=await request.json()
        metadata=json.loads(path.with_suffix('.json').read_text())
        if body.get('sha256')!=metadata['sha256'] or not body.get('local_path'):
            raise ValueError('录像传输校验不匹配')
        atomic_json(path.with_name(path.name+'.received.json'),dict(local_path=str(body['local_path']),
                    sha256=body['sha256'],received_at=time.time()))
        return web.json_response(dict(ok=True))

    async def cloud(request):
        return web.json_response(points)

    async def raster_metadata(request):
        if request.match_info['name'] != map_name:
            raise web.HTTPNotFound()
        return web.json_response(dict(grid, image_url=f'/api/maps/{map_name}/image.png'),
                                 headers={'Cache-Control': 'no-cache'})

    async def raster_image(request):
        if request.match_info['name'] != map_name:
            raise web.HTTPNotFound()
        return web.Response(body=grid_png, content_type='image/png',
                            headers={'Cache-Control': 'no-cache'})

    app = web.Application(middlewares=[errors])
    app.router.add_get('/', index)
    app.router.add_get('/api/status', status)
    app.router.add_get('/api/maps/{name}/points', cloud)
    app.router.add_get('/api/maps/{name}/grid', raster_metadata)
    app.router.add_get('/api/maps/{name}/image.png', raster_image)
    app.router.add_get('/api/maps/{name}/waypoints', waypoints)
    app.router.add_get('/api/settings', settings)
    app.router.add_post('/api/settings', settings)
    app.router.add_post('/api/action', action)
    app.router.add_get('/api/rgb.jpg', rgb)
    app.router.add_get('/api/rgb.mjpeg', rgb_stream)
    app.router.add_post('/api/recording/{action}', recording_command)
    app.router.add_get('/api/recordings', recordings)
    app.router.add_get('/api/recordings/{name}', recording_file)
    app.router.add_post('/api/recordings/{name}/received', recording_received)
    app.router.add_static('/static', ROOT/'web')
    return app


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ros', action='store_true', help='Connect only to the selected ROS_DOMAIN_ID')
    ap.add_argument('--port', type=int, default=8092)
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--image-topic', default='/camera/d435i/color/image_raw')
    ap.add_argument('--mission', default=str(ROOT/'src/patrol_manager/config/door_patrol.yaml'))
    ap.add_argument('--map', default=str(ROOT/'src/bringup/maps/hkclr_office/2D/hkclr_office_2d.yaml'))
    ap.add_argument('--pcd-map', default=str(ROOT/'src/bringup/maps/hkclr_office/3D/map.pcd'))
    args = ap.parse_args()
    bridge = None
    if args.ros:
        from ros_bridge import RosBridge
        bridge = RosBridge(args.image_topic, args.pcd_map)
    try:
        web.run_app(create_app(args, bridge), host=args.host, port=args.port)
    finally:
        if bridge:
            bridge.close()


if __name__ == '__main__':
    main()
