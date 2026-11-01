"""Restart only the AL FAST-LIO process supervised by its existing ROS launch."""
import os
from pathlib import Path
import shutil
import signal
import time


def owned_processes(root, proc=Path('/proc')):
    executable = (root/'install/fast_lio/lib/fast_lio/fastlio_mapping').resolve()
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if (entry/'exe').resolve() != executable:
                continue
            env = (entry/'environ').read_bytes().split(b'\0')
            if b'ROS_DOMAIN_ID=76' not in env or b'AL_FASTLIO_RESPAWN=1' not in env:
                continue
            found.append(int(entry.name))
        except (OSError, RuntimeError):
            continue
    return found


def restart_fastlio(root=None, timeout=20):
    root = Path(root or Path(__file__).resolve().parents[1]).resolve()
    processes = owned_processes(root)
    if len(processes) != 1:
        raise RuntimeError(f'AL FAST-LIO 自动重启进程应为 1 个，实际 {len(processes)}；请检查 AL 启动配置')
    previous = processes[0]
    # Preserve the failure evidence before native FAST-LIO truncates its logs.
    archive = root/'logs/fastlio_sessions'/str(time.time_ns())
    archive.mkdir(parents=True)
    for name in ('mat_pre.txt', 'mat_out.txt'):
        source = root/'src/FAST_LIO/Log'/name
        if source.is_file():
            shutil.copy2(source, archive/name)
    os.kill(previous, signal.SIGINT)
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        current = owned_processes(root)
        if len(current) == 1 and current[0] != previous:
            return dict(previous_pid=previous, pid=current[0], log_archive=str(archive))
        time.sleep(.1)
    raise RuntimeError('AL FAST-LIO 已收到重启请求，但 ROS launch 未在 20 秒内拉起新进程')
