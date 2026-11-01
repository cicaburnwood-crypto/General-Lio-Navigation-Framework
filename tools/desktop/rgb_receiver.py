#!/usr/bin/env python3
"""Developer-PC receiver: copy finalized AL RGB videos, verify, acknowledge."""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
import urllib.parse
import urllib.request


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--url',default='http://10.30.3.234:8092')
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'recordings/AL_RGB')
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=True)
    lock=(args.output/'.receiver.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    client=urllib.request.build_opener(urllib.request.ProxyHandler({}))
    last_error=None
    while True:
        try:
            with client.open(args.url+'/api/recordings',timeout=10) as r:items=json.load(r)['recordings']
            for item in reversed(items):
                name=item['name']
                if Path(name).name!=name or not name.startswith('AL_RGB_') or not name.endswith('.mp4'):
                    continue
                final=args.output/name
                receipt=final.with_suffix('.json')
                downloaded=None
                if receipt.is_file() and final.is_file() and final.stat().st_size==item['bytes']:
                    downloaded=json.loads(receipt.read_text())
                if not downloaded or downloaded.get('sha256')!=item['sha256']:
                    partial=final.with_suffix('.mp4.part')
                    digest=hashlib.sha256()
                    print('Downloading '+name,flush=True)
                    with client.open(args.url+'/api/recordings/'+urllib.parse.quote(name),timeout=60) as r, partial.open('wb') as f:
                        for block in iter(lambda:r.read(1024*1024),b''):
                            f.write(block);digest.update(block)
                        f.flush();os.fsync(f.fileno())
                    if partial.stat().st_size!=item['bytes'] or digest.hexdigest()!=item['sha256']:
                        raise ValueError('Checksum mismatch: '+name)
                    os.replace(partial,final)
                    receipt.write_text(json.dumps(item,ensure_ascii=False,indent=2)+'\n')
                    print('Saved '+str(final),flush=True)
                if item.get('transfer')!='received' or item.get('local_path')!=str(final):
                    request=urllib.request.Request(args.url+'/api/recordings/'+urllib.parse.quote(name)+'/received',
                        data=json.dumps(dict(sha256=item['sha256'],local_path=str(final))).encode(),
                        headers={'Content-Type':'application/json','X-Scan-Nav':'operator'})
                    with client.open(request,timeout=10) as r:json.load(r)
            last_error=None
        except Exception as exc:
            if str(exc)!=last_error:
                print('Will retry: '+str(exc),flush=True)
                last_error=str(exc)
        time.sleep(2)


if __name__=='__main__':
    main()
