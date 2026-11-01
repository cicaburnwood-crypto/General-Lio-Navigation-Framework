import importlib.util
from pathlib import Path

spec=importlib.util.spec_from_file_location('fastlio_restart',Path(__file__).parents[1]/'fastlio_restart.py')
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_process_selection_never_includes_bl_or_other_domains(tmp_path):
    root=tmp_path/'AL';proc=tmp_path/'proc';proc.mkdir()
    al_exe=root/'install/fast_lio/lib/fast_lio/fastlio_mapping'
    for pid,exe,env in [
        (1,al_exe,b'ROS_DOMAIN_ID=76\0AL_FASTLIO_RESPAWN=1\0'),
        (2,tmp_path/'BL/fastlio_mapping',b'ROS_DOMAIN_ID=76\0AL_FASTLIO_RESPAWN=1\0'),
        (3,al_exe,b'ROS_DOMAIN_ID=192\0AL_FASTLIO_RESPAWN=1\0'),
        (4,al_exe,b'ROS_DOMAIN_ID=76\0'),
    ]:
        entry=proc/str(pid);entry.mkdir();(entry/'exe').symlink_to(exe)
        (entry/'environ').write_bytes(env)
    assert module.owned_processes(root,proc)==[1]


def test_restart_archives_logs_and_waits_for_replacement(tmp_path,monkeypatch):
    log=tmp_path/'src/FAST_LIO/Log';log.mkdir(parents=True)
    (log/'mat_pre.txt').write_text('old session evidence')
    processes=iter([[101],[101],[202]])
    signals=[]
    monkeypatch.setattr(module,'owned_processes',lambda root:next(processes))
    monkeypatch.setattr(module.os,'kill',lambda pid,sig:signals.append((pid,sig)))
    monkeypatch.setattr(module.time,'sleep',lambda _:None)
    result=module.restart_fastlio(tmp_path)
    assert result['previous_pid']==101 and result['pid']==202
    assert signals==[(101,module.signal.SIGINT)]
    assert (Path(result['log_archive'])/'mat_pre.txt').read_text()=='old session evidence'
