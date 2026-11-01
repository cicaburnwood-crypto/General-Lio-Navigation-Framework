#!/usr/bin/env python3
"""Fetch private release assets through gh; verify before atomic installation."""
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def main():
    manifest = json.loads((ROOT / 'assets.json').read_text())
    for asset in manifest['files']:
        target = ROOT / asset['path']
        if target.is_file() and digest(target) == asset['sha256']:
            print('Verified:', asset['path'])
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(dir=target.parent) as folder:
            subprocess.run(['gh', 'release', 'download', manifest['release'],
                            '--repo', manifest['repository'], '--pattern', asset['asset'],
                            '--dir', folder], check=True)
            download = Path(folder) / asset['asset']
            if digest(download) != asset['sha256']:
                raise RuntimeError('Asset SHA-256 mismatch: ' + asset['asset'])
            download.replace(target)
        print('Installed:', asset['path'])


if __name__ == '__main__':
    main()
