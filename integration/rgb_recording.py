"""Native camera recorder; filesystem IPC only, no navigation interfaces."""
from fractions import Fraction
from pathlib import Path
import hashlib
import json
import os
import queue
import threading
import time
import uuid


def atomic_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
    temporary.write_text(json.dumps(value, ensure_ascii=False))
    os.replace(temporary, path)


def read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return default


class NativeRecorder:
    def __init__(self, root):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        (self.root/'commands').mkdir(exist_ok=True)
        self.lock = threading.RLock()
        self.session = None
        self.camera_at = 0.
        self.shape = None
        self.status = dict(state='idle', active=False)
        self.shutdown = threading.Event()
        # Never replay a start command left by a previous camera process.
        for path in (self.root/'commands').glob('*.json'):
            path.unlink()
        self.control = threading.Thread(target=self._control, daemon=True)
        self.control.start()

    def offer(self, frame, stamp):
        with self.lock:
            self.camera_at = time.monotonic()
            self.shape = frame.shape[:2]
            session = self.session
            if session is None or session['stop'].is_set():
                return
            try:
                session['queue'].put_nowait((frame, stamp))
            except queue.Full:
                session['dropped'] += 1

    def _start(self):
        import av
        with self.lock:
            if self.session is not None:
                raise ValueError('录像正在进行或封装中')
            if self.shape is None or time.monotonic()-self.camera_at > 2:
                raise ValueError('尚无原生 RGB 帧，无法开始录像')
            height, width = self.shape
            name = time.strftime('AL_RGB_%Y%m%d_%H%M%S')+'_'+uuid.uuid4().hex[:8]+'.mp4'
            partial = self.root/(name[:-4]+'.partial.mp4')
            container = av.open(str(partial), 'w', format='mp4', options={
                'movflags': '+frag_keyframe+empty_moov+default_base_moof', 'frag_duration': '1000000'})
            try:
                stream = container.add_stream('libx264', rate=30)
                stream.width, stream.height = width, height
                stream.pix_fmt = 'yuv420p'
                stream.time_base = Fraction(1,90000)
                stream.codec_context.time_base = Fraction(1,90000)
                stream.codec_context.thread_count = 2
                stream.options = dict(preset='ultrafast', crf='20', tune='zerolatency')
                stream.codec_context.open()
            except Exception:
                container.close()
                raise
            self.session = session = dict(name=name, partial=partial, container=container,
                stream=stream, queue=queue.Queue(maxsize=8), stop=threading.Event(),
                dropped=0, frames=0, duration_s=0., width=width, height=height,
                started_at=time.time())
            self.status = dict(state='recording', active=True, name=name, width=width,
                height=height, frames=0, dropped=0, duration_s=0., started_at=session['started_at'])
            session['worker'] = threading.Thread(target=self._encode, args=(session,), daemon=True)
            session['worker'].start()

    def _stop(self):
        with self.lock:
            if self.session is not None:
                self.session['stop'].set()
                self.status.update(state='finalizing', active=False)

    def _encode(self, s):
        import av
        first = None
        previous_pts = -1
        try:
            while not s['stop'].is_set() or not s['queue'].empty():
                try:
                    frame, stamp = s['queue'].get(timeout=.2)
                except queue.Empty:
                    continue
                if frame.shape[:2] != (s['height'],s['width']):
                    raise ValueError('相机分辨率发生变化；录像片段保留在小车端')
                if first is None:
                    first = stamp
                image = av.VideoFrame.from_ndarray(frame, format='bgr24')
                image.pts = max(previous_pts+1, round((stamp-first)*90000))
                image.time_base = Fraction(1,90000)
                previous_pts = image.pts
                for packet in s['stream'].encode(image):
                    s['container'].mux(packet)
                s['frames'] += 1
                s['duration_s'] = (stamp-first)
                with self.lock:
                    self.status.update(frames=s['frames'], duration_s=round(s['duration_s'],2), dropped=s['dropped'])
            for packet in s['stream'].encode():
                s['container'].mux(packet)
            s['container'].close()
            if not s['frames']:
                raise ValueError('没有录入 RGB 帧；未生成空录像')
            final = self.root/s['name']
            os.replace(s['partial'], final)
            digest = hashlib.sha256()
            with final.open('rb') as f:
                for block in iter(lambda:f.read(1024*1024), b''):
                    digest.update(block)
            metadata = {k:s[k] for k in ('name','width','height','frames','duration_s','dropped','started_at')}
            metadata.update(state='complete', active=False, bytes=final.stat().st_size,
                            sha256=digest.hexdigest(), completed_at=time.time(), transfer='pending')
            atomic_json(final.with_suffix('.json'), metadata)
            with self.lock:
                self.status = metadata
        except Exception as exc:
            try:
                s['container'].close()
            except Exception:
                pass
            with self.lock:
                self.status.update(state='error', active=False, error=str(exc))
        finally:
            with self.lock:
                self.session = None

    def _control(self):
        while not self.shutdown.is_set():
            for path in sorted((self.root/'commands').glob('*.json')):
                request = read_json(path,{})
                try:
                    if time.time()-request.get('created',0) > 15:
                        raise ValueError('录像请求已过期，请重新点击')
                    if request.get('action') == 'start':
                        self._start()
                    elif request.get('action') == 'stop':
                        self._stop()
                    else:
                        raise ValueError('未知录像操作')
                    reply = dict(ok=True)
                except Exception as exc:
                    reply = dict(ok=False,error=str(exc))
                atomic_json(self.root/('reply_'+path.stem+'.json'),reply)
                path.unlink(missing_ok=True)
            with self.lock:
                value = dict(self.status, heartbeat=time.time(),
                             camera_age_s=round(time.monotonic()-self.camera_at,3))
            atomic_json(self.root/'status.json', value)
            self.shutdown.wait(.2)

    def close(self):
        self._stop()
        with self.lock:
            worker = self.session['worker'] if self.session else None
        if worker:
            worker.join(timeout=15)
        self.shutdown.set()
        self.control.join(timeout=2)


class RecordingControl:
    def __init__(self, root):
        self.root = Path(root)

    def status(self):
        value = read_json(self.root/'status.json',dict(state='offline',active=False))
        value['online'] = time.time()-value.get('heartbeat',0) < 3
        if not value['online']:
            value.update(state='offline',active=False)
        name = value.get('name')
        if name:
            receipt = read_json(self.root/(name+'.received.json'))
            if receipt:
                value.update(transfer='received',local_path=receipt['local_path'])
        return value

    def command(self, action):
        if action not in ('start','stop'):
            raise ValueError('Unknown recording operation')
        if not self.status()['online']:
            raise ValueError('相机录像模块尚未连接')
        identity = uuid.uuid4().hex
        request = self.root/'commands'/(identity+'.json')
        reply = self.root/('reply_'+identity+'.json')
        atomic_json(request,dict(action=action,created=time.time()))
        deadline = time.monotonic()+12
        while time.monotonic()<deadline:
            result=read_json(reply)
            if result is not None:
                reply.unlink(missing_ok=True)
                if not result['ok']:
                    raise ValueError(result['error'])
                return result
            time.sleep(.05)
        request.unlink(missing_ok=True)
        raise ValueError('录像操作响应超时，请查看当前录像状态')

    def recordings(self):
        values=[]
        for path in sorted(self.root.glob('AL_RGB_*.json'),reverse=True):
            item=read_json(path,{})
            if item.get('state')!='complete':
                continue
            receipt=read_json(self.root/(item['name']+'.received.json'))
            if receipt:
                item.update(transfer='received',local_path=receipt['local_path'])
            values.append(item)
        return values

    def file(self, name):
        if Path(name).name!=name or not name.startswith('AL_RGB_') or not name.endswith('.mp4') or '.partial.' in name:
            raise ValueError('Invalid recording filename')
        path=self.root/name
        if not path.is_file() or read_json(path.with_suffix('.json'),{}).get('state')!='complete':
            raise ValueError('录像尚未完成')
        return path
