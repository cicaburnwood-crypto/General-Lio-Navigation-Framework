#!/usr/bin/env python3
"""No-hardware domain regression tests; all launcher side effects are mocked.

Run with the system Python. Optional --ros tests use localhost-only dedicated
String topics, never robot topics/services. Optional --tmux uses its own socket.
"""
import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import tempfile
import time
import unittest
import uuid

CLI = argparse.ArgumentParser()
CLI.add_argument('--home', default='/home/hkclr', help='Deployment/staged home root')
CLI.add_argument('--ros', action='store_true')
CLI.add_argument('--tmux', action='store_true')
ARGS, EXTRA = CLI.parse_known_args()
SOURCE = Path(ARGS.home)
AGV = Path(os.environ.get('FAULT_LOG_TEST_REPO', str(SOURCE / 'agv/agv_ws/src/agv2_pkg')))
NAV = SOURCE / 'chassis_nav_ws'

MOCK = r'''#!/usr/bin/python3
import json, os, pathlib, subprocess, sys
name = pathlib.Path(sys.argv[0]).name
args = sys.argv[1:]
def record(kind, **data):
    with open(os.environ['AUDIT_LOG'], 'a') as f:
        f.write(json.dumps(dict(kind=kind, args=args, domain=os.environ.get('ROS_DOMAIN_ID'), **data))+'\n')
if name == 'sudo':
    assert args[:2] == ['-u', 'hkclr'], args
    args = args[2:]
    env = dict(os.environ)
    env.pop('ROS_DOMAIN_ID', None)  # sudo does not need to preserve the domain
    if '-lc' in args:
        args[args.index('-lc')] = '-c'  # fixture must not source host /etc/profile
    sys.exit(subprocess.call(args, env=env))
elif name == 'getent':
    print('hkclr:x:1000:1000:hkclr:'+os.environ['HOME']+':/bin/bash')
elif name == 'tmux':
    record('tmux')
    if args[0] == 'has-session': sys.exit(0 if os.environ.get('MOCK_EXISTS') == '1' else 1)
    if args[0] == 'show-option': print(os.environ.get('MOCK_DOMAIN', ''))
    if args[0] in ['list-panes', 'split-window']: print('%0')
elif name == 'ip':
    record('hardware_ip_mock')
elif name in ['ros2', 'rviz2']:
    record('ros_mock')
    if args[:2] == ['service', 'call'] and os.environ.get('MOCK_SERVICE_FAIL') == '1': sys.exit(1)
    if args[:2] == ['service', 'list']: print('/localizer/relocalize')
    if 'agv2_control.launch.py' in args:
        for child in ['rosbag2_recorder', 'chassis_fault_snapshot', 'respawned_agv2_control']:
            subprocess.run(['/usr/bin/python3', '-c',
                "import os,json;open(os.environ['AUDIT_LOG'],'a').write(json.dumps({'kind':'child','name':"+
                repr(child)+",'domain':os.environ.get('ROS_DOMAIN_ID')})+'\\n')"], check=True)
elif name == 'sleep': pass
elif name in ['fault_logger_start.py', 'chassis_fault_log.py']:
    record('logger_mock')
    if name == 'fault_logger_start.py' and os.environ.get('MOCK_LOG_PREFLIGHT_FAIL') == '1': sys.exit(1)
    if name == 'chassis_fault_log.py' and args[0] == os.environ.get('MOCK_LOG_FAIL_OP'): sys.exit(1)
else:
    raise RuntimeError('Unrecognized mock: '+name)
'''


class DomainTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix='robot234-domain-test-')
        self.root = Path(self.tmp.name)
        self.home = self.root / 'home'
        self.repo = self.home / 'agv/agv_ws/src/agv2_pkg'
        self.nav = self.home / 'chassis_nav_ws'
        self.bin = self.root / 'bin'
        self.bin.mkdir()
        self.log = self.root / 'actions.jsonl'
        self.domain = self.home / '.config/hkclr/ros_domain.sh'
        self.domain.parent.mkdir(parents=True)
        shutil.copy2(SOURCE / '.config/hkclr/ros_domain.sh', self.domain)
        for name in ['sudo', 'getent', 'tmux', 'ip', 'ros2', 'rviz2', 'sleep']:
            target = self.bin / name
            target.write_text(MOCK)
            target.chmod(0o755)
        self.setup = self.root / 'ros_setup.bash'
        self.setup.write_text('export ROS_DOMAIN_ID=13\nexport PATH="$MOCK_BIN:$PATH"\n')
        for name in ['setup_agv_env.bash', 'start_agv.sh', 'stop_agv.sh']:
            p = self.repo / 'deploy' / name
            p.parent.mkdir(parents=True, exist_ok=True)
            content = (AGV / 'deploy' / name).read_text()
            # Fixtures only: disable privilege check and replace absolute OS endpoints.
            content = content.replace('${EUID:-$(id -u)}', '0')
            content = content.replace('/usr/bin/tmux', str(self.bin / 'tmux'))
            content = content.replace('/sys/class/net/', str(self.root / 'net') + '/')
            content = content.replace('/opt/ros/humble/setup.bash', str(self.setup))
            p.write_text(content)
        (self.root / 'net/can0').mkdir(parents=True)
        self.params = self.repo / 'deploy/params.yaml'
        self.params.write_text('test: true\n')
        for relative in ['deploy/fault_logger_start.py', 'scripts/chassis_fault_log.py']:
            fake = self.repo / relative
            fake.parent.mkdir(parents=True, exist_ok=True)
            fake.write_text(MOCK)
        overlay = self.home / 'agv/agv_ws/install_release_humble/setup.bash'
        overlay.parent.mkdir(parents=True)
        overlay.write_text('export ROS_DOMAIN_ID=14\n')
        self.nav.mkdir(parents=True)
        for name in ['nav_in_map.sh', 'nav_3d_in_map.sh', 'scripts/nav_tmux_common.sh']:
            dst = self.nav / name
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(NAV / name, dst)
        (self.nav / 'install').mkdir()
        (self.nav / 'install/setup.bash').write_text('export ROS_DOMAIN_ID=15\n')
        for relative in ['src/bringup/maps/hkclr_office/3D/map.pcd',
                         'src/bringup/maps/hkclr_office/2D/hkclr_office_2d.yaml',
                         'src/nav_interface_adapter/config/adapter_fastlio_body.yaml',
                         'src/chassis_3d_localization/config/vgicp_localizer.yaml']:
            path = self.nav / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.touch()
        self.env = {**os.environ, 'HOME': str(self.home), 'PATH': str(self.bin)+':'+os.environ['PATH'],
                    'MOCK_BIN': str(self.bin), 'AUDIT_LOG': str(self.log), 'SUDO_USER': 'hkclr',
                    'ROS_DOMAIN_ID': '0', 'RUN_RVIZ': '1', 'NAV_LOG_DIR': str(self.root / 'nav_logs')}
        for k in ['MOCK_EXISTS', 'MOCK_DOMAIN', 'MOCK_SERVICE_FAIL', 'BASH_ENV', 'ENV',
                  'SESSION_NAME', 'PCD_MAP', 'MAP_YAML', 'ADAPTER_CONFIG', 'LOCALIZER_CONFIG']:
            self.env.pop(k, None)

    def tearDown(self):
        self.tmp.cleanup()

    def shell(self, text, env=None, ok=True):
        result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c', text],
                                env=env or self.env, capture_output=True, text=True, timeout=15)
        if ok:
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0, result.stdout + result.stderr)
        return result

    def records(self):
        return [json.loads(line) for line in self.log.read_text().splitlines()] if self.log.exists() else []

    def clear(self):
        if self.log.exists(): self.log.unlink()

    def start(self, ok=True):
        return self.shell('bash '+shlex.quote(str(self.repo/'deploy/start_agv.sh'))+' '+shlex.quote(str(self.params)), ok=ok)

    def stop(self, ok=True):
        return self.shell('bash '+shlex.quote(str(self.repo/'deploy/stop_agv.sh')), ok=ok)

    def assert_no_hardware_or_ros(self):
        self.assertFalse([r for r in self.records() if r['kind'] in ['hardware_ip_mock', 'ros_mock']])
        self.assertFalse([r for r in self.records() if r['kind'] == 'tmux' and r['args'][0] in ['new-session', 'kill-session', 'send-keys', 'attach-session']])

    def run_panes(self):
        commands = [r['args'][-2] for r in self.records() if r['kind']=='tmux' and r['args'][0]=='send-keys']
        for command in commands:
            # Don't leave the interactive keep-open shell running in the fixture.
            command = command.replace('; exec bash', '')
            self.shell(command)
        return commands

    def test_syntax(self):
        for p in [SOURCE/'.bashrc', SOURCE/'.config/hkclr/ros_domain.sh', *list((AGV/'deploy').glob('*.sh')),
                  AGV/'deploy/setup_agv_env.bash', NAV/'scripts/nav_tmux_common.sh', NAV/'nav_in_map.sh', NAV/'nav_3d_in_map.sh']:
            subprocess.run(['bash', '-n', str(p)], check=True)

    def test_domain_overrides_and_nounset(self):
        for incoming in [None, '0', '75', '76', 'invalid']:
            for nounset in ['', 'set -u; ']:
                with self.subTest(incoming=incoming, nounset=nounset):
                    env = dict(self.env)
                    if incoming is None: env.pop('ROS_DOMAIN_ID', None)
                    else: env['ROS_DOMAIN_ID'] = incoming
                    self.shell(nounset+'source '+shlex.quote(str(self.repo/'deploy/setup_agv_env.bash'))+
                               '; [[ $ROS_DOMAIN_ID == 76 ]]; '+ ('[[ $- == *u* ]]' if nounset else '[[ $- != *u* ]]'), env)

    def test_environment_fails_closed(self):
        for contents in [None, 'return 1\n', 'export ROS_DOMAIN_ID=0\n']:
            with self.subTest(contents=contents):
                if contents is None: self.domain.unlink()
                else: self.domain.write_text(contents)
                self.shell('set -u; source '+shlex.quote(str(self.repo/'deploy/setup_agv_env.bash'))+
                           ' && ros2 forbidden', ok=False)
                self.assert_no_hardware_or_ros()

    def test_missing_workspace_setup_fails_closed(self):
        (self.home/'agv/agv_ws/install_release_humble/setup.bash').unlink()
        self.start(ok=False)
        self.assert_no_hardware_or_ros()

    def test_start_all_four_panes_and_log_children(self):
        self.start()
        self.assertEqual(len(self.run_panes()), 4)
        nodes = [r for r in self.records() if r['kind'] in ['ros_mock', 'child']]
        self.assertEqual(len(nodes), 7)
        self.assertTrue(all(r['domain']=='76' for r in nodes))
        ip = [r for r in self.records() if r['kind']=='hardware_ip_mock']
        self.assertEqual(len(ip), 4)
        self.assertFalse([r for r in self.records() if r['kind']=='tmux' and r['args'][0]=='kill-session'])

    def test_start_missing_or_broken_domain(self):
        for contents in [None, 'return 1\n', 'export ROS_DOMAIN_ID=0\n']:
            with self.subTest(contents=contents):
                self.clear()
                if contents is None: self.domain.unlink()
                else: self.domain.write_text(contents)
                self.start(ok=False)
                self.assert_no_hardware_or_ros()

    def test_start_refuses_existing_session(self):
        self.env['MOCK_EXISTS']='1'
        self.start(ok=False)
        self.assert_no_hardware_or_ros()

    def test_logging_preflight_blocks_all_hardware(self):
        self.env['MOCK_LOG_PREFLIGHT_FAIL'] = '1'
        self.start(ok=False)
        self.assert_no_hardware_or_ros()

    def test_logging_bind_failure_blocks_ros_but_preserves_can_stage(self):
        self.env['MOCK_LOG_FAIL_OP'] = 'wait'
        self.start(ok=False)
        self.assertFalse([r for r in self.records() if r['kind'] == 'tmux' and r['args'][0] == 'new-session'])
        ops = [r['args'][0] for r in self.records() if r['kind'] == 'logger_mock']
        self.assertIn('pause', ops)
        self.assertIn('stop', ops)

    def test_logging_failure_does_not_block_hardware_stop(self):
        self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN='76', MOCK_LOG_FAIL_OP='pause')
        self.stop()
        self.assertTrue([r for r in self.records() if r['kind']=='hardware_ip_mock'])

    def test_stop_order_and_domain(self):
        self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN='76')
        self.stop()
        records = self.records()
        service = next(i for i,r in enumerate(records) if r['kind']=='ros_mock')
        kill = next(i for i,r in enumerate(records) if r['kind']=='tmux' and r['args'][0]=='kill-session')
        down = next(i for i,r in enumerate(records) if r['kind']=='hardware_ip_mock')
        self.assertLess(service, kill)
        self.assertLess(kill, down)
        self.assertEqual(records[service]['domain'], '76')
        self.assertIn('/agv2/prepare_shutdown', records[service]['args'])

    def test_stop_rejects_unknown_or_old_session(self):
        for domain in ['', '0', '75']:
            with self.subTest(domain=domain):
                self.clear()
                self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN=domain)
                self.stop(ok=False)
                self.assert_no_hardware_or_ros()

    def test_stop_preflight_failure(self):
        self.domain.unlink()
        self.stop(ok=False)
        self.assert_no_hardware_or_ros()

    def test_stop_failed_handshake_preserves_running_stack(self):
        self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN='76', MOCK_SERVICE_FAIL='1')
        self.stop(ok=False)
        self.assertFalse([r for r in self.records() if r['kind']=='hardware_ip_mock' or
                          (r['kind']=='tmux' and r['args'][0]=='kill-session')])

    def test_both_navigation_entries_all_panes(self):
        for script in ['nav_in_map.sh', 'nav_3d_in_map.sh']:
            with self.subTest(script=script):
                self.clear()
                extra = ' initial_pose:=0,0,0,0,0,0,1' if '3d' in script else ''
                self.shell('bash '+shlex.quote(str(self.nav/script))+extra)
                commands = self.run_panes()
                self.assertEqual(len(commands), 7 if '3d' in script else 5)
                records = [r for r in self.records() if r['kind']=='ros_mock']
                self.assertTrue(records)
                self.assertTrue(all(r['domain']=='76' for r in records))
                if '3d' in script:
                    self.assertIn('ros_domain_id: 76', (self.root/'nav_logs/session_info.txt').read_text())

    def test_navigation_refuses_missing_config_and_legacy_sessions(self):
        for script in ['nav_in_map.sh', 'nav_3d_in_map.sh']:
            for domain in ['', '0']:
                with self.subTest(script=script, domain=domain):
                    self.clear()
                    self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN=domain)
                    self.shell('bash '+shlex.quote(str(self.nav/script)), ok=False)
                    self.assert_no_hardware_or_ros()
        self.domain.unlink()
        for script in ['nav_in_map.sh', 'nav_3d_in_map.sh']:
            self.clear()
            self.shell('bash '+shlex.quote(str(self.nav/script)), ok=False)
            self.assert_no_hardware_or_ros()

    def test_navigation_can_attach_verified_session(self):
        self.env.update(MOCK_EXISTS='1', MOCK_DOMAIN='76')
        self.shell('bash '+shlex.quote(str(self.nav/'nav_in_map.sh')))
        self.assertTrue(any(r['args'][0]=='attach-session' for r in self.records() if r['kind']=='tmux'))
        self.assertFalse(any(r['args'][0]=='new-session' for r in self.records() if r['kind']=='tmux'))

    def test_navigation_blocks_changed_domain_before_ros(self):
        self.shell('bash '+shlex.quote(str(self.nav/'nav_in_map.sh')))
        self.domain.write_text('export ROS_DOMAIN_ID=0\n')
        self.run_panes()
        self.assertFalse([r for r in self.records() if r['kind']=='ros_mock'])

    def test_interactive_shell_loads_shared_config(self):
        content = (SOURCE/'.bashrc').read_text().split('# BEGIN robot234 ROS domain (deployment-managed)',1)[1]
        rc = self.home/'.bashrc'
        rc.write_text(content)
        result = subprocess.run(['bash', '--noprofile', '--rcfile', str(rc), '-ic', '[[ $ROS_DOMAIN_ID == 76 ]]'],
                                env=self.env, capture_output=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)

    @unittest.skipUnless(ARGS.tmux, 'enable with --tmux')
    def test_real_isolated_tmux_with_stale_domain(self):
        socket = 'domain234-test-'+uuid.uuid4().hex
        output = self.root/'tmux_domain.txt'
        base = ['/usr/bin/tmux', '-L', socket, '-f', '/dev/null']
        # Only an environment probe runs in this dedicated server; never a driver.
        command = 'source '+shlex.quote(str(self.repo/'deploy/setup_agv_env.bash'))+' && printf "%s" "$ROS_DOMAIN_ID" > '+shlex.quote(str(output))+'; exec /bin/sleep 20'
        try:
            subprocess.run(base+['new-session','-d','-s','environment_probe','/bin/bash --noprofile --norc -c '+shlex.quote(command)],
                           env=self.env, check=True, timeout=10)
            deadline=time.monotonic()+5
            while not output.exists() and time.monotonic()<deadline: time.sleep(.05)
            self.assertEqual(output.read_text(), '76')
        finally:
            subprocess.run(base+['kill-server'], env=self.env, capture_output=True, timeout=10)

    @unittest.skipUnless(ARGS.ros, 'enable with --ros under ROS Python environment')
    def test_real_ros_localhost_same_domain_and_cross_domain(self):
        # Separate contexts, unique topic, explicit loopback discovery only.
        saved=os.environ.get('ROS_LOCALHOST_ONLY')
        os.environ['ROS_LOCALHOST_ONLY']='1'
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String
        topic='/domain234_isolation_test_'+uuid.uuid4().hex
        contexts=[]; nodes=[]; executors=[]; same=[]; cross=[]; reverse=[]
        try:
            for domain in [76,76,0]:
                ctx=Context(); rclpy.init(context=ctx, domain_id=domain); contexts.append(ctx)
                nodes.append(Node('domain_probe_'+uuid.uuid4().hex, context=ctx, enable_rosout=False, start_parameter_services=False))
                executor=SingleThreadedExecutor(context=ctx)
                executor.add_node(nodes[-1]); executors.append(executor)
            pub76=nodes[0].create_publisher(String,topic,10)
            nodes[1].create_subscription(String,topic,lambda msg:same.append(msg.data),10)
            nodes[2].create_subscription(String,topic,lambda msg:cross.append(msg.data),10)
            pub0=nodes[2].create_publisher(String,topic+'_reverse',10)
            nodes[0].create_subscription(String,topic+'_reverse',lambda msg:reverse.append(msg.data),10)
            deadline=time.monotonic()+6
            while time.monotonic()<deadline:
                pub76.publish(String(data='domain76')); pub0.publish(String(data='domain0'))
                for executor in executors: executor.spin_once(timeout_sec=.02)
            self.assertTrue(same, 'Same-domain discovery did not establish: test inconclusive')
            self.assertFalse(cross); self.assertFalse(reverse)
        finally:
            for executor in executors: executor.shutdown()
            for node in nodes: node.destroy_node()
            for ctx in contexts: ctx.shutdown()
            if saved is None: os.environ.pop('ROS_LOCALHOST_ONLY',None)
            else: os.environ['ROS_LOCALHOST_ONLY']=saved


if __name__ == '__main__':
    unittest.main(argv=[__file__]+EXTRA, verbosity=2)
