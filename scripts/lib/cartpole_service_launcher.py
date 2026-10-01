"""Supervise one persistent CartPole service and its own official Isaac process.

This module has no ROS, Conda or Isaac imports. The existing Isaac launcher owns
the environment checks, domain preflight and global simulator lock. The service
uses a separate, clean system-Jazzy subprocess. Signals never target a name or an
unverified PID; cleanup records each process's Linux start ticks.
"""

import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid


ROOT = Path(__file__).resolve().parents[2]
SERVICE_NAME = '/astrex/cartpole/move_to'
STARTUP_TIMEOUT_SEC = 600.0
NODE_SHUTDOWN_GRACE_SEC = 10.0


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def load_config(root=ROOT):
    """Read the single existing trusted config, not the caller's environment."""
    script = ('set -e; set -a; source "$1"; '
              'exec /usr/bin/python3 -B -c "$2"')
    python = ('import json,os; print(json.dumps({k:v for k,v in os.environ.items() '
              'if k.startswith("ASTREX_")}))')
    result = subprocess.run(
        ['bash', '--noprofile', '--norc', '-c', script, 'cartpole-config',
         str(root / 'config/isaac_baseline.env'), python],
        env={'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8'},
        capture_output=True, text=True, timeout=10, check=True)
    config = json.loads(result.stdout)
    required = ('ASTREX_ISAAC_OUTPUT_ROOT', 'ASTREX_ROS_DISTRO',
                'ASTREX_ROS_DOMAIN_ID', 'ASTREX_RMW_IMPLEMENTATION')
    if any(not config.get(key) for key in required):
        raise RuntimeError('Isaac baseline config is incomplete')
    if (config['ASTREX_ROS_DOMAIN_ID'] != '63'
            or config['ASTREX_RMW_IMPLEMENTATION'] != 'rmw_fastrtps_cpp'
            or config['ASTREX_ROS_DISTRO'] != 'jazzy'):
        raise RuntimeError('CartPole service requires the validated domain 63 Jazzy/Fast DDS profile')
    return config


def proc_data(pid):
    try:
        proc = Path('/proc') / str(pid)
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        return {'pid': int(pid), 'state': fields[0], 'ppid': int(fields[1]),
                'start_ticks': int(fields[19]),
                'cwd': str((proc / 'cwd').resolve(strict=True)),
                'cmdline': (proc / 'cmdline').read_bytes()}
    except (OSError, IndexError, ValueError):
        return None


def identity_alive(identity):
    data = proc_data(identity['pid']) if identity else None
    return bool(data and data['state'] != 'Z'
                and data['start_ticks'] == identity['start_ticks'])


def child_pids(pid):
    try:
        raw = (Path('/proc') / str(pid) / 'task' / str(pid) / 'children').read_text()
        return [int(item) for item in raw.split()]
    except (OSError, ValueError):
        return []


class OwnedProcess:
    """Track only verified descendants of the Popen process created here."""

    def __init__(self, process):
        self.process = process
        self.identities = {}
        identity = proc_data(process.pid)
        if identity is None:
            raise RuntimeError('Cannot identify the process just created by this launcher')
        self.identities[process.pid] = identity
        self.root = identity

    def refresh(self):
        pending = list(self.identities.values())
        seen = set()
        while pending:
            parent = pending.pop()
            if parent['pid'] in seen or not identity_alive(parent):
                continue
            seen.add(parent['pid'])
            for pid in child_pids(parent['pid']):
                data = proc_data(pid)
                # Check the parent again so PID reuse cannot attach an unrelated tree.
                if data and data['ppid'] == parent['pid'] and identity_alive(parent):
                    known = self.identities.get(pid)
                    if known and known['start_ticks'] != data['start_ticks']:
                        continue
                    self.identities[pid] = data
                    pending.append(data)

    def signal_root(self, signum):
        if identity_alive(self.root):
            try:
                os.kill(self.root['pid'], signum)
            except ProcessLookupError:
                pass

    def signal_all(self, signum):
        self.refresh()
        for identity in reversed(list(self.identities.values())):
            if identity_alive(identity):
                try:
                    os.kill(identity['pid'], signum)
                except ProcessLookupError:
                    pass

    def wait_gone(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.refresh()
            if not any(identity_alive(item) for item in self.identities.values()):
                self.process.wait(timeout=1)
                return True
            time.sleep(0.1)
        return False

    def stop(self, *, service=False):
        self.refresh()
        # The service gets time to publish and confirm zeros while Sim still runs.
        self.signal_root(signal.SIGINT if service else signal.SIGTERM)
        if self.wait_gone(NODE_SHUTDOWN_GRACE_SEC if service else 25.0):
            return True
        self.signal_all(signal.SIGTERM)
        if self.wait_gone(5.0):
            return True
        self.signal_all(signal.SIGKILL)
        return self.wait_gone(3.0)


def relay_output(stream, path, label, outputs=None):
    with path.open('x', encoding='utf-8') as log:
        for line in stream:
            log.write(line)
            log.flush()
            print(f'[{label}] {line}', end='', flush=True)
            if outputs is not None and line.startswith('OUTPUT='):
                outputs.put(line[len('OUTPUT='):].strip())


def sim_command(headless=False, root=ROOT):
    # No trial marker or trial initial-state options: requests start from reality.
    return [str(root / 'scripts/start_isaac_ros.sh'), *(['--headless'] if headless else [])]


def service_command(isaac_dir, output_dir, root=ROOT):
    overlay = root / 'ros2_ws/install/setup.bash'
    if not overlay.is_file():
        raise RuntimeError('ROS overlay missing; build astrex_interfaces and astrex_ros_bridge first')
    script = ('set -eo pipefail; source /opt/ros/jazzy/setup.bash; '
              'source "$1"; shift; set -u; '
              'exec /usr/bin/python3 -B -m astrex_ros_bridge.cartpole_service_node "$@"')
    return ['bash', '--noprofile', '--norc', '-c', script, 'cartpole-service', str(overlay),
            '--isaac-run-dir', str(isaac_dir), '--output-dir', str(output_dir)]


def service_environment(config, logs):
    # Do not copy caller site-packages, ROS overlay or dynamic-library paths.
    return {'HOME': os.environ.get('HOME', '/home/sssxy'), 'PATH': '/usr/bin:/bin',
            'LANG': 'C.UTF-8', 'ROS_DOMAIN_ID': config['ASTREX_ROS_DOMAIN_ID'],
            'RMW_IMPLEMENTATION': config['ASTREX_RMW_IMPLEMENTATION'],
            'ROS_LOG_DIR': str(logs), 'PYTHONDONTWRITEBYTECODE': '1',
            'PYTHONUNBUFFERED': '1', 'OMP_NUM_THREADS': '2', 'OPENBLAS_NUM_THREADS': '2'}


def validate_isaac_ready(run, owned, *, headless, output_root):
    run = run.resolve(strict=True)
    if run.parent != (output_root / 'ros').resolve(strict=True):
        raise RuntimeError(f'Isaac supplied an unexpected output directory: {run}')
    manifest = json.loads((run / 'manifest.json').read_text())
    argv = manifest.get('argv', [])
    if (manifest.get('profile') != 'ros' or manifest.get('trial') is not None
            or not any(str(item).endswith('/run_ros_cartpole.py') for item in argv)
            or any(str(item).startswith('--trial-') for item in argv)
            or ('--headless' in argv) != headless):
        raise RuntimeError('Isaac is not the requested persistent official ROS profile')
    path = run / 'ready.json'
    if not path.is_file():
        return None
    ready = json.loads(path.read_text())
    owned.refresh()
    identity = owned.identities.get(int(ready['pid']))
    if (not identity_alive(identity) or identity['cwd'] != str(run)
            or b'run_ros_cartpole.py' not in identity['cmdline']):
        raise RuntimeError('Isaac ready PID is not this launcher\'s own simulator')
    if (ready.get('device') != 'cpu' or bool(ready.get('gui')) != (not headless)
            or ready.get('trial') is not None
            or not str(ready.get('graph', '')).startswith('/World/AstrEXROSGraph_')):
        raise RuntimeError('Isaac readiness does not confirm the expected CPU/GUI/control graph')
    return ready


def wait_for_sim(owned, outputs, *, headless, output_root, deadline, stopping):
    run = None
    while time.monotonic() < deadline:
        if stopping.is_set():
            raise InterruptedError('Stopped during Isaac startup')
        owned.refresh()
        if owned.process.poll() is not None:
            raise RuntimeError(f'Isaac launcher exited {owned.process.returncode} before readiness')
        if run is None:
            try:
                run = Path(outputs.get(timeout=0.1))
            except queue.Empty:
                continue
        ready = validate_isaac_ready(run, owned, headless=headless, output_root=output_root)
        if ready is not None:
            return run.resolve(), ready
        time.sleep(0.1)
    raise TimeoutError('Isaac did not become ready within 600 local seconds')


def wait_for_service(owned, sim, output, isaac_dir, deadline, stopping):
    while time.monotonic() < deadline:
        if stopping.is_set():
            raise InterruptedError('Stopped during service startup')
        owned.refresh()
        sim.refresh()
        if sim.process.poll() is not None or owned.process.poll() is not None:
            raise RuntimeError('Isaac or service exited before service readiness')
        path = output / 'ready.json'
        if path.is_file():
            ready = json.loads(path.read_text())
            if (ready.get('service') != SERVICE_NAME or ready.get('status') != 'READY'
                    or ready.get('pid') != owned.root['pid']
                    or Path(ready.get('isaac_run_dir', '')).resolve() != isaac_dir
                    or not identity_alive(owned.root)):
                raise RuntimeError('Service readiness does not match this launch')
            return ready
        time.sleep(0.1)
    raise TimeoutError('CartPole service did not become ready within its startup limit')


def shutdown_evidence(service_dir):
    try:
        value = json.loads((service_dir / 'shutdown.json').read_text())
        return value.get('zero_confirmed') is True
    except (OSError, ValueError, TypeError):
        return False


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--headless', action='store_true', help='Keep the same CPU/full-experience profile without a GUI')
    parser.add_argument('--print-config', action='store_true', help='Print the startup contract without creating output or processes')
    args = parser.parse_args(argv)
    config = load_config()
    if args.print_config:
        print(json.dumps({'service': SERVICE_NAME, 'headless': args.headless,
                          'startup_order': ['official_isaac_ros', 'system_jazzy_service'],
                          'sim_argv': sim_command(args.headless), 'config': config}, indent=2))
        return 0
    if not (ROOT / 'ros2_ws/install/setup.bash').is_file():
        raise RuntimeError('ROS overlay missing; build both service packages before launch')
    output_root = Path(config['ASTREX_ISAAC_OUTPUT_ROOT'])
    key = hashlib.sha256(str(ROOT).encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / f'astrex-cartpole-service-{os.getuid()}-{key}.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise RuntimeError('Another CartPole service launcher is running') from exc
        run = output_root / 'cartpole_service' / (
            datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10])
        run.mkdir(parents=True, exist_ok=False)
        service_dir = run / 'service'
        service_dir.mkdir()
        ros_logs = run / 'ros_logs'
        ros_logs.mkdir()
        manifest = {'status': 'STARTING', 'pid': os.getpid(), 'service': SERVICE_NAME,
                    'headless': args.headless, 'created_utc': datetime.now(timezone.utc).isoformat(),
                    'config_sha256': hashlib.sha256((ROOT / 'config/isaac_baseline.env').read_bytes()).hexdigest(),
                    'sim_argv': sim_command(args.headless)}
        atomic_json(run / 'manifest.json', manifest)
        print(f'CARTPOLE_SERVICE_OUTPUT={run}\nSERVICE_EVIDENCE={service_dir}', flush=True)
        stopping = threading.Event()
        previous_handlers = {}
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, lambda number, frame: stopping.set())
        sim = node = None
        readers = []
        error = None
        started_node = False
        cleanup_ok = False
        zero_confirmed = False
        try:
            outputs = queue.Queue()
            child = subprocess.Popen(manifest['sim_argv'], cwd=ROOT, stdin=subprocess.DEVNULL,
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                     encoding='utf-8', errors='replace', bufsize=1, start_new_session=True)
            sim = OwnedProcess(child)
            thread = threading.Thread(target=relay_output,
                                      args=(child.stdout, run / 'isaac_launcher.log', 'Isaac', outputs), daemon=True)
            thread.start()
            readers.append(thread)
            deadline = time.monotonic() + STARTUP_TIMEOUT_SEC
            isaac_dir, sim_ready = wait_for_sim(sim, outputs, headless=args.headless,
                                               output_root=output_root, deadline=deadline, stopping=stopping)
            command = service_command(isaac_dir, service_dir)
            manifest.update(isaac_run_dir=str(isaac_dir), isaac_ready=sim_ready, service_argv=command)
            atomic_json(run / 'manifest.json', manifest)
            child = subprocess.Popen(command, cwd=ROOT, env=service_environment(config, ros_logs),
                                     stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding='utf-8', errors='replace', bufsize=1, start_new_session=True)
            node = OwnedProcess(child)
            started_node = True
            thread = threading.Thread(target=relay_output,
                                      args=(child.stdout, run / 'service_console.log', 'Service'), daemon=True)
            thread.start()
            readers.append(thread)
            ready = wait_for_service(node, sim, service_dir, isaac_dir,
                                     min(deadline, time.monotonic() + 30.0), stopping)
            manifest.update(status='READY', service_ready=ready)
            atomic_json(run / 'manifest.json', manifest)
            print(f'CARTPOLE_SERVICE_READY={SERVICE_NAME}\nISAAC_OUTPUT={isaac_dir}\n'
                  'GUI and service stay open; Ctrl+C zeros first, then closes only owned processes.', flush=True)
            while not stopping.wait(0.2):
                sim.refresh()
                node.refresh()
                if sim.process.poll() is not None:
                    raise RuntimeError(f'Isaac closed/exited: {sim.process.returncode}')
                if node.process.poll() is not None:
                    raise RuntimeError(f'CartPole service exited unexpectedly: {node.process.returncode}')
        except (OSError, ValueError, KeyError, RuntimeError, TimeoutError, InterruptedError) as exc:
            error = f'{type(exc).__name__}: {exc}'
            if isinstance(exc, InterruptedError) and stopping.is_set():
                error = None
        finally:
            # Preserve physics while the service's signal handler completes final zeros.
            node_gone = node.stop(service=True) if node is not None else not started_node
            zero_confirmed = shutdown_evidence(service_dir) if started_node else False
            sim_gone = sim.stop() if sim is not None else True
            cleanup_ok = node_gone and sim_gone
            for thread in readers:
                thread.join(timeout=5)
            for signum, handler in previous_handlers.items():
                signal.signal(signum, handler)
            if started_node and not zero_confirmed:
                error = (error + '; ' if error else '') + 'Service shutdown did not confirm Controller zero'
            if not cleanup_ok:
                error = (error + '; ' if error else '') + 'Owned process cleanup could not be verified'
            manifest.update(status='STOPPED' if error is None else 'FAIL', error=error,
                            shutdown_zero_confirmed=zero_confirmed, cleanup_verified=cleanup_ok,
                            finished_utc=datetime.now(timezone.utc).isoformat())
            atomic_json(run / 'manifest.json', manifest)
        print(f'CARTPOLE_SERVICE_STATUS={manifest["status"]}', flush=True)
        if error:
            print(error, file=sys.stderr, flush=True)
        return 0 if error is None else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as exc:
        print(f'CARTPOLE SERVICE LAUNCH ERROR: {exc}', file=sys.stderr, flush=True)
        raise SystemExit(2) from exc
