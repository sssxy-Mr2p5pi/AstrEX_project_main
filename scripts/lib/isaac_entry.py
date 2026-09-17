"""Read-only health checks and isolated process supervision for the fixed baseline."""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import importlib.metadata as metadata
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import tempfile
import uuid

ROOT = Path(os.environ['ASTREX_ROOT'])

def setting(name):
    return os.environ['ASTREX_' + name]

def command(argv, **kwargs):
    return subprocess.run(argv, text=True, capture_output=True, timeout=60, **kwargs)

def health():
    errors = []
    lab = Path(setting('ISAAC_LAB_ROOT'))
    expected_prefix = Path(setting('CONDA_ROOT')) / 'envs' / setting('ISAAC_CONDA_ENV')
    result = {'python': platform.python_version(), 'executable': sys.executable,
              'environment': setting('ISAAC_CONDA_ENV'), 'versions': {}, 'warnings': []}
    if Path(sys.prefix) != expected_prefix:
        errors.append('Python does not belong to the configured environment')
    if '.'.join(platform.python_version_tuple()[:2]) != setting('PYTHON_VERSION'):
        errors.append('Python major/minor mismatch')
    for package, key in [('isaacsim', 'ISAAC_SIM_VERSION'), ('torch', 'TORCH_VERSION'),
                         ('torchvision', 'TORCHVISION_VERSION'), ('torchaudio', 'TORCHAUDIO_VERSION'),
                         ('rsl-rl-lib', 'RSL_RL_VERSION')]:
        try:
            actual = metadata.version(package)
        except metadata.PackageNotFoundError:
            actual = 'MISSING'
        result['versions'][package] = actual
        if actual != setting(key):
            errors.append(f'{package}: expected {setting(key)}, got {actual}')
    for field, args in [('lab_commit', ['rev-parse', 'HEAD']),
                        ('lab_tag_commit', ['rev-parse', setting('ISAAC_LAB_VERSION') + '^{commit}']),
                        ('lab_status', ['status', '--porcelain'])]:
        p = command(['git', '--no-optional-locks', '-C', str(lab), *args])
        result[field] = p.stdout.strip()
        if p.returncode:
            errors.append(f'Cannot inspect {field}: {p.stderr}')
    if result['lab_commit'] != setting('ISAAC_LAB_COMMIT') or result['lab_tag_commit'] != setting('ISAAC_LAB_COMMIT'):
        errors.append('Lab tag/commit mismatch')
    if any(not row.startswith('??') for row in result['lab_status'].splitlines()):
        errors.append('Lab has tracked changes')
    if result['lab_status']:
        result['warnings'].append('Lab untracked files: ' + result['lab_status'])
    # Editable installation must point at the configured source, not another Lab copy.
    try:
        direct = json.loads(metadata.distribution('isaaclab').read_text('direct_url.json') or '{}')
        from urllib.parse import unquote, urlparse
        actual_source = Path(unquote(urlparse(direct.get('url', '')).path))
        if actual_source.resolve() != (lab / 'source/isaaclab').resolve():
            errors.append(f'Unexpected editable Lab source: {direct}')
        result['lab_editable'] = direct
    except Exception as exc:
        errors.append(f'Cannot resolve Lab install: {exc}')
    try:
        import torch
        import torchvision  # noqa: F401
        import torchaudio  # noqa: F401
        result['cuda_available'] = torch.cuda.is_available()
        result['torch_cuda'] = torch.version.cuda
        result['gpu'] = torch.cuda.get_device_name(0) if result['cuda_available'] else None
        if not result['cuda_available'] or setting('GPU_NAME') not in (result['gpu'] or ''):
            errors.append('Configured CUDA GPU unavailable or unexpected GPU')
    except Exception as exc:
        errors.append(f'Core import/CUDA failure: {exc}')
    sim = Path(metadata.distribution('isaacsim').locate_file('isaacsim')) if result['versions']['isaacsim'] != 'MISSING' else Path('/nonexistent')
    bridge = sim / 'exts/isaacsim.ros2.bridge'
    result['bridge'] = str(bridge)
    if not (bridge / 'jazzy/lib').is_dir():
        errors.append('Bundled Jazzy bridge libraries missing')
    if not (lab / setting('ISAAC_EXPERIENCE')).is_file():
        errors.append('Full Lab experience missing')
    if not Path('/opt/ros/' + setting('ROS_DISTRO') + '/setup.bash').is_file():
        errors.append('System ROS missing')
    result['system_python'] = command(['/usr/bin/python3', '--version']).stdout.strip()
    pip = command([sys.executable, '-B', '-m', 'pip', 'check'])
    known = {s for s in (ROOT / 'config/isaac_known_pip_warnings.txt').read_text().splitlines() if s and not s.startswith('#')}
    found = set(pip.stdout.strip().splitlines()) - {'No broken requirements found.'}
    result['known_pip_warnings'] = sorted(found & known)
    result['new_pip_warnings'] = sorted(found - known)
    if found - known or pip.returncode not in (0, 1) or pip.stderr.strip():
        errors.append('New pip diagnostic; no automatic repair: ' + pip.stderr)
    result['errors'] = errors
    result['status'] = 'FAIL' if errors else 'PASS_WITH_KNOWN_WARNINGS'
    return result

def arguments(argv):
    profile = argv[0]
    parser = argparse.ArgumentParser(description=f'AstrEX fixed Isaac baseline: {profile}')
    if profile == 'check':
        parser.add_argument('--allow-legacy', action='store_true', help='Inspect target baseline, never approve legacy use')
        return profile, parser.parse_args(argv[1:]), []
    parser.add_argument('--print-config', action='store_true', help='Print resolved arguments without starting Sim')
    if profile == 'rl':
        parser.add_argument('--task', default='Isaac-Cartpole-Direct-v0')
        parser.add_argument('--num_envs', type=int, default=16)
        parser.add_argument('--seed', type=int, default=42)
        parser.add_argument('--max_iterations', type=int, default=20)
        parser.add_argument('--device', default=setting('RL_DEVICE'))
        mode = parser.add_mutually_exclusive_group()
        mode.add_argument('--gui', action='store_true')
        mode.add_argument('--headless', action='store_true')
        args, extra = parser.parse_known_args(argv[1:])
        if args.device != setting('RL_DEVICE') or args.num_envs < 1 or args.max_iterations < 1:
            parser.error('RL requires cuda:0, positive num_envs and max_iterations')
        for item in extra:
            if (item.startswith(('--dis', '--kit', '--exp', '--ray', '-rid')) or
                any(token in item.lower() for token in ('device', 'distributed', 'multi_gpu', 'ray-proc', 'kit_args', 'experience', 'log_dir', 'log_root', 'hydra.run', 'hydra.sweep'))):
                parser.error('Profile/device/output overrides are not supported: ' + item)
        return profile, args, extra
    parser.add_argument('--headless', action='store_true')
    return profile, parser.parse_args(argv[1:]), []

def main():
    profile, args, extra = arguments(sys.argv[1:])
    caller = os.environ.get('ASTREX_CALLER_CONDA', '')
    legacy = Path(caller).name in ('isaaclab60', 'isaaclab60_6010test')
    if profile == 'check' and legacy and not args.allow_legacy:
        raise RuntimeError('Legacy caller detected. Deactivate it or use --allow-legacy for diagnosis only.')
    if getattr(args, 'print_config', False):
        print(json.dumps({'profile': profile, 'arguments': vars(args), 'extra': extra,
                          'environment': setting('ISAAC_CONDA_ENV'), 'caller': caller}, indent=2))
        return 0
    check = health()
    print(json.dumps(check, indent=2, ensure_ascii=False), flush=True)
    if check['errors']:
        return 1
    if profile == 'check':
        if legacy:
            print('LEGACY_CALLER_NOT_APPROVED: diagnostics refer only to the configured target environment')
        return 0
    if legacy:
        print(f'Caller {caller} is legacy. This child explicitly uses {setting("ISAAC_CONDA_ENV")}.')
    key = hashlib.sha256(str(ROOT).encode()).hexdigest()[:16]
    lock_path = Path(tempfile.gettempdir()) / f'astrex-isaac-{os.getuid()}-{key}.lock'
    fd = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    with os.fdopen(fd, 'w') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another AstrEX Isaac launcher is running')
        run = Path(setting('ISAAC_OUTPUT_ROOT')) / profile / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10])
        run.mkdir(parents=True, exist_ok=False)
        env = dict(os.environ, ASTREX_RUN_DIR=str(run))
        (run / 'health.json').write_text(json.dumps(check, indent=2))
        if profile == 'ros':
            # A clean *system* subprocess does discovery. No system ROS paths enter Sim.
            probe = ['env', '-i', 'HOME=' + os.environ['HOME'], 'PATH=/usr/bin:/bin',
                     'ROS_DOMAIN_ID=' + setting('ROS_DOMAIN_ID'), 'RMW_IMPLEMENTATION=' + setting('RMW_IMPLEMENTATION'),
                     'bash', '--noprofile', '--norc', '-c',
                     'source /opt/ros/' + setting('ROS_DISTRO') + '/setup.bash; exec /usr/bin/python3 -B "$1"',
                     'probe', str(ROOT / 'sim/scripts/ros_domain_check.py')]
            discovered = command(probe)
            (run / 'domain_check.txt').write_text(discovered.stdout + discovered.stderr)
            if discovered.returncode:
                raise RuntimeError('ROS domain is occupied or cannot be inspected: ' + discovered.stdout + discovered.stderr)
            env.update(ROS_DISTRO=setting('ROS_DISTRO'), ROS_DOMAIN_ID=setting('ROS_DOMAIN_ID'),
                       RMW_IMPLEMENTATION=setting('RMW_IMPLEMENTATION'), LD_LIBRARY_PATH=check['bridge'] + '/jazzy/lib')
            script = ROOT / 'sim/scripts/run_ros_cartpole.py'
            tail = ['--headless'] if args.headless else []
        else:
            script = ROOT / 'sim/scripts/rl_official.py'
            tail = ['--task', args.task, '--num_envs', str(args.num_envs), '--seed', str(args.seed),
                    '--max_iterations', str(args.max_iterations), '--device', setting('RL_DEVICE')]
            if not args.gui:
                tail.append('--headless')
            tail += extra
            # Keep Kit logs and persistent settings outside the installed environment.
            tail += ['--kit_args', f'--/app/userConfigPath={run / "user.config.json"} '
                     f'--/log/file={run / "kit.log"} --/app/settings/persistent=false --/app/settings/loadUserConfig=false']
        argv = [sys.executable, '-B', str(script), *tail]
        (run / 'manifest.json').write_text(json.dumps({'profile': profile, 'argv': argv, 'cwd': str(run),
             'config_sha256': hashlib.sha256((ROOT / 'config/isaac_baseline.env').read_bytes()).hexdigest()}, indent=2))
        print(f'PROFILE={profile}\nOUTPUT={run}\nLOG={run / "console.log"}\nCHECKPOINT_ROOT={run / "logs/rsl_rl"}', flush=True)
        child = subprocess.Popen(argv, cwd=run, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                 text=True, bufsize=1, start_new_session=True)
        def forward(signum, frame):
            if child.poll() is None:
                os.killpg(child.pid, signum)
        signal.signal(signal.SIGINT, forward)
        signal.signal(signal.SIGTERM, forward)
        with (run / 'console.log').open('w') as log:
            for line in child.stdout:
                log.write(line)
                log.flush()
                print(line, end='', flush=True)
        code = child.wait()
        if profile == 'rl' and code == 0:
            result_path = run / 'rl_result.json'
            if not result_path.exists() or json.loads(result_path.read_text()).get('status') != 'PASS':
                print('RL evidence missing/failed: simulator exit code is not sufficient', flush=True)
                code = 1
        (run / 'exit.json').write_text(json.dumps({'returncode': code}))
        return code if code >= 0 else 128 - code

if __name__ == '__main__':
    try:
        sys.exit(main())
    except (RuntimeError, OSError, subprocess.SubprocessError) as exc:
        print('BASELINE ERROR: ' + str(exc), file=sys.stderr)
        sys.exit(1)
