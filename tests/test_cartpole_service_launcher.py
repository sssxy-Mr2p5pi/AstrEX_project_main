"""CartPole service supervision contracts. No ROS or simulator is started."""

import io
import json
from pathlib import Path
import signal
import subprocess
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts/lib'))
import cartpole_service_launcher as launcher  # noqa: E402


def config(output):
    return {'ASTREX_ISAAC_OUTPUT_ROOT': str(output), 'ASTREX_ROS_DISTRO': 'jazzy',
            'ASTREX_ROS_DOMAIN_ID': '63', 'ASTREX_RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp'}


def test_default_sim_launch_is_gui_and_never_uses_trial_markers(tmp_path):
    command = launcher.sim_command(root=tmp_path / 'project with spaces')
    assert command == [str(tmp_path / 'project with spaces/scripts/start_isaac_ros.sh')]
    assert launcher.sim_command(True, root=tmp_path)[-1] == '--headless'
    assert not any('--trial-' in flag for flag in command)


def test_system_python_and_overlay_are_quoted_arguments(tmp_path):
    project = tmp_path / 'project with spaces'
    overlay = project / 'ros2_ws/install/setup.bash'
    overlay.parent.mkdir(parents=True)
    overlay.touch()
    command = launcher.service_command(tmp_path / 'isaac with space', tmp_path / 'output with space', root=project)
    assert str(overlay) in command
    script = command[command.index('-c') + 1]
    assert 'source "$1"' in script and '"$@"' in script
    assert script.index('source /opt/ros/jazzy') < script.index('set -u')
    assert 'exec /usr/bin/python3 -B -m astrex_ros_bridge.cartpole_service_node' in script
    assert command[-4:] == ['--isaac-run-dir', str(tmp_path / 'isaac with space'),
                            '--output-dir', str(tmp_path / 'output with space')]


def test_service_environment_does_not_inherit_python_or_ros_contamination(tmp_path, monkeypatch):
    for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH', 'AMENT_PREFIX_PATH',
                'CONDA_PREFIX', 'ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION'):
        monkeypatch.setenv(key, '/wrong')
    env = launcher.service_environment(config(tmp_path), tmp_path)
    assert env['PATH'] == '/usr/bin:/bin'
    assert env['ROS_DOMAIN_ID'] == '63'
    assert env['RMW_IMPLEMENTATION'] == 'rmw_fastrtps_cpp'
    assert not any(key in env for key in ('PYTHONPATH', 'PYTHONHOME', 'LD_LIBRARY_PATH',
                                         'AMENT_PREFIX_PATH', 'CONDA_PREFIX'))


def test_config_loader_uses_only_baseline_and_rejects_wrong_domain(tmp_path):
    path = tmp_path / 'project with spaces/config'
    path.mkdir(parents=True)
    env_file = path / 'isaac_baseline.env'
    env_file.write_text("ASTREX_ISAAC_OUTPUT_ROOT='/tmp/output with spaces'\n"
                        'ASTREX_ROS_DISTRO=jazzy\nASTREX_ROS_DOMAIN_ID=63\n'
                        'ASTREX_RMW_IMPLEMENTATION=rmw_fastrtps_cpp\n')
    assert launcher.load_config(path.parent)['ASTREX_ISAAC_OUTPUT_ROOT'] == '/tmp/output with spaces'
    env_file.write_text(env_file.read_text().replace('=63', '=64'))
    with pytest.raises(RuntimeError, match='domain 63'):
        launcher.load_config(path.parent)


def test_pid_reuse_is_not_alive(monkeypatch):
    monkeypatch.setattr(launcher, 'proc_data', lambda pid: {'state': 'S', 'start_ticks': 200})
    assert not launcher.identity_alive({'pid': 99, 'start_ticks': 100})
    assert launcher.identity_alive({'pid': 99, 'start_ticks': 200})


def test_tree_cleanup_never_signals_reused_pid(monkeypatch):
    rows = {1: {'pid': 1, 'start_ticks': 100, 'state': 'S', 'ppid': 0},
            2: {'pid': 2, 'start_ticks': 200, 'state': 'S', 'ppid': 1}}
    monkeypatch.setattr(launcher, 'proc_data', lambda pid: rows.get(pid))
    monkeypatch.setattr(launcher, 'child_pids', lambda pid: [2] if pid == 1 else [])
    process = type('Process', (), {'pid': 1})()
    owned = launcher.OwnedProcess(process)
    owned.refresh()
    rows[2] = {'pid': 2, 'start_ticks': 300, 'state': 'S', 'ppid': 999}
    signalled = []
    monkeypatch.setattr(launcher.os, 'kill', lambda pid, sig: signalled.append((pid, sig)))
    owned.signal_all(signal.SIGTERM)
    assert signalled == [(1, signal.SIGTERM)]


def test_tree_discovery_cannot_follow_reused_parent(monkeypatch):
    calls = {'root': 0}

    def data(pid):
        if pid == 1:
            calls['root'] += 1
            return {'pid': 1, 'start_ticks': 100 if calls['root'] < 3 else 200, 'state': 'S', 'ppid': 0}
        return {'pid': pid, 'start_ticks': 300, 'state': 'S', 'ppid': 1}

    monkeypatch.setattr(launcher, 'proc_data', data)
    monkeypatch.setattr(launcher, 'child_pids', lambda pid: [2])
    owned = launcher.OwnedProcess(type('Process', (), {'pid': 1})())
    owned.refresh()
    assert 2 not in owned.identities


def test_sim_ready_rejects_trials_and_foreign_pids(tmp_path, monkeypatch):
    run = tmp_path / 'ros/unique'
    run.mkdir(parents=True)
    manifest = {'profile': 'ros', 'trial': None, 'argv': ['/a/run_ros_cartpole.py']}
    (run / 'manifest.json').write_text(json.dumps(manifest))
    ready = {'graph': '/World/AstrEXROSGraph_a', 'pid': 10, 'device': 'cpu', 'gui': True, 'trial': None}
    (run / 'ready.json').write_text(json.dumps(ready))
    identity = {'pid': 10, 'start_ticks': 100, 'cwd': str(run), 'cmdline': b'run_ros_cartpole.py'}
    owned = type('Owned', (), {'identities': {10: identity}, 'refresh': lambda self: None})()
    monkeypatch.setattr(launcher, 'identity_alive', lambda row: row == identity)
    assert launcher.validate_isaac_ready(run, owned, headless=False, output_root=tmp_path) == ready
    ready['pid'] = 999
    (run / 'ready.json').write_text(json.dumps(ready))
    with pytest.raises(RuntimeError, match='own simulator'):
        launcher.validate_isaac_ready(run, owned, headless=False, output_root=tmp_path)
    manifest['trial'] = {'x0': 0}
    (run / 'manifest.json').write_text(json.dumps(manifest))
    with pytest.raises(RuntimeError, match='persistent official'):
        launcher.validate_isaac_ready(run, owned, headless=False, output_root=tmp_path)


def test_shutdown_evidence_does_not_treat_exit_as_zero(tmp_path):
    assert not launcher.shutdown_evidence(tmp_path)
    (tmp_path / 'shutdown.json').write_text('{"zero_confirmed": false}')
    assert not launcher.shutdown_evidence(tmp_path)
    (tmp_path / 'shutdown.json').write_text('{"zero_confirmed": true}')
    assert launcher.shutdown_evidence(tmp_path)


def test_launcher_startup_order_and_owned_zero_before_sim_stop(tmp_path, monkeypatch):
    root = tmp_path / 'project'
    root.mkdir()
    overlay = root / 'ros2_ws/install/setup.bash'
    overlay.parent.mkdir(parents=True)
    overlay.touch()
    (root / 'config').mkdir()
    (root / 'config/isaac_baseline.env').touch()
    monkeypatch.setattr(launcher, 'ROOT', root)
    monkeypatch.setattr(launcher, 'load_config', lambda: config(tmp_path / 'output'))
    monkeypatch.setattr(launcher.tempfile, 'gettempdir', lambda: str(tmp_path))
    monkeypatch.setattr(launcher, 'sim_command', lambda headless: ['sim-launcher'])
    monkeypatch.setattr(launcher, 'service_command', lambda sim, out: ['system-service', str(out)])
    events = []
    handlers = {}

    def register(sig, handler):
        previous = handlers.get(sig)
        handlers[sig] = handler
        return previous

    monkeypatch.setattr(launcher.signal, 'signal', register)

    class Process:
        def __init__(self, command, **kwargs):
            self.kind = command[0]
            self.stdout = io.StringIO('')
            self.output = Path(command[1]) if self.kind == 'system-service' else None
            self.returncode = None
            events.append(self.kind)

        def poll(self):
            return self.returncode

    class Owned:
        def __init__(self, process):
            self.process = process
            self.root = {'pid': 123}

        def refresh(self):
            pass

        def stop(self, *, service=False):
            events.append('zero-service' if service else 'close-sim')
            if service:
                (self.process.output / 'shutdown.json').write_text('{"zero_confirmed": true}')
            return True

    monkeypatch.setattr(launcher.subprocess, 'Popen', Process)
    monkeypatch.setattr(launcher, 'OwnedProcess', Owned)
    isaac = tmp_path / 'output/ros/unique'
    monkeypatch.setattr(launcher, 'wait_for_sim', lambda *args, **kwargs: (isaac, {}))

    def ready(*args):
        events.append('service-ready')
        handlers[signal.SIGINT](signal.SIGINT, None)
        return {'status': 'READY'}

    monkeypatch.setattr(launcher, 'wait_for_service', ready)
    assert launcher.main([]) == 0
    assert events == ['sim-launcher', 'system-service', 'service-ready', 'zero-service', 'close-sim']
    manifest = json.loads(next((tmp_path / 'output/cartpole_service').glob('*/manifest.json')).read_text())
    assert manifest['status'] == 'STOPPED'
    assert manifest['shutdown_zero_confirmed'] and manifest['cleanup_verified']


def test_help_does_not_source_config_or_start_process(monkeypatch):
    monkeypatch.setattr(launcher, 'load_config', lambda: pytest.fail('help must not read config'))
    with pytest.raises(SystemExit) as exit_value:
        launcher.main(['--help'])
    assert exit_value.value.code == 0


def test_shell_entry_explicitly_selects_system_python():
    entry = launcher.ROOT / 'scripts/start_cartpole_service.sh'
    result = subprocess.run(['bash', '-n', str(entry)], capture_output=True, text=True)
    assert result.returncode == 0
    assert 'exec /usr/bin/python3 -B' in entry.read_text()
    assert 'isaac_common.sh' not in entry.read_text()
