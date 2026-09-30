#!/usr/bin/env python3
"""Run the fixed Cartpole Step 3D/B matrix, one isolated Isaac process at a time.

The 0 m/0 degree connection check is separate. This plan contains the remaining
16 cases. Pass ``--accept-completed TRIAL_DIR`` once per already completed prefix case;
each saved assessment and raw evidence are checked again.
Each new case gets a fresh Isaac run and a unique trial directory. The runner
stops at the first failure and never changes the 5 N or assessment limits.
"""

import argparse
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
import time
import uuid

from analyze_balance_hold_trial import assess_trial


ROOT = Path(__file__).resolve().parents[1]
SHARED_ROOT = Path('/data/shared/AstrEX_project_data/logs/isaac')
MATRIX_ROOT = SHARED_ROOT / 'balance_hold_step3d' / 'matrix_runs'
ISAAC_ROOT = SHARED_ROOT / 'ros'
MAX_ISAAC_WALL_SEC = 600.0
MAX_CONTROLLER_WALL_SEC = 120.0
MAX_ZERO_EVIDENCE_WALL_SEC = 30.0
FORCE_LIMIT_N = 5.0
FIXED_DOMAIN = '63'
FIXED_RMW = 'rmw_fastrtps_cpp'
VERSION = 1


@dataclass(frozen=True)
class Case:
    name: str
    batch: int
    hold_position: float
    x0: float
    theta0: float


def planned_cases():
    cases = []
    for batch, degrees in ((1, 1), (2, 2)):
        for position, position_name in ((-0.3, 'm0p3'), (0.0, '0'), (0.3, 'p0p3')):
            for direction, direction_name in ((-1, 'm'), (1, 'p')):
                cases.append(Case(f'batch{batch}_hold_{position_name}_theta_{direction_name}{degrees}',
                                  batch, position, position, math.radians(direction * degrees)))
    for position, position_name in ((-0.05, 'm0p05'), (0.05, 'p0p05')):
        for direction, direction_name in ((-1, 'm'), (1, 'p')):
            cases.append(Case(f'batch3_x0_{position_name}_theta_{direction_name}1',
                              3, 0.0, position, math.radians(direction)))
    assert len(cases) == 16 and len({case.name for case in cases}) == 16
    return tuple(cases)


CASES = planned_cases()
FINGERPRINT_PATHS = (
    'scripts/run_balance_hold_matrix.py',
    'scripts/analyze_balance_hold_trial.py',
    'sim/scripts/run_ros_cartpole.py',
    'ros2_ws/src/astrex_ros_bridge/astrex_ros_bridge/balance_hold_closed_loop_node.py',
    'ros2_ws/src/astrex_ros_bridge/astrex_ros_bridge/balance_hold_controller.py',
    'config/isaac_baseline.env',
)


def fingerprints():
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in FINGERPRINT_PATHS}


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def atomic_json(path, obj):
    """Only the aggregate manifest is updated; individual attempts stay immutable."""
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with tmp.open('x', encoding='utf-8') as stream:
        json.dump(obj, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def _matches_case(config, case):
    try:
        return (all(math.isclose(float(config[key]), getattr(case, key), abs_tol=1e-9)
                    for key in ('x0', 'theta0', 'hold_position'))
                and math.isclose(float(config['force_limit_n']), FORCE_LIMIT_N, abs_tol=1e-9)
                and 0 < float(config['max_sim_sec']) <= 10.0)
    except (KeyError, TypeError, ValueError):
        return False


def verified_pass(trial_dir, case):
    """Check planned inputs, saved verdict, and raw evidence without changing files."""
    try:
        config = json.loads((trial_dir / 'config.json').read_text(encoding='utf-8'))
        saved = json.loads((trial_dir / 'assessment.json').read_text(encoding='utf-8'))
        if not _matches_case(config, case):
            return None
        report = assess_trial(trial_dir)
        if not saved.get('passed') or not report['passed'] or saved.get('issues'):
            return None
        if saved.get('trial_id') != report['trial_id']:
            return None
        if Path(config['isaac_run_dir']).name != report['trial_id']:
            return None
        return report
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _proc_data(pid):
    """Read identity fields without trusting a PID alone (PIDs can be recycled)."""
    proc = Path('/proc') / str(pid)
    try:
        stat = (proc / 'stat').read_text(encoding='utf-8')
        fields = stat.rsplit(')', 1)[1].split()
        return {'state': fields[0], 'ppid': int(fields[1]), 'pgrp': int(fields[2]),
                'start_ticks': int(fields[19]),
                'cwd': (proc / 'cwd').resolve(strict=True),
                'cmdline': (proc / 'cmdline').read_bytes()}
    except (OSError, IndexError, ValueError):
        return None


def _remember_child(launcher, isaac_dir=None):
    """Find only a direct child of our launcher running this trial's Sim script."""
    try:
        raw = (Path('/proc') / str(launcher.pid) / 'task' / str(launcher.pid) /
               'children').read_text(encoding='ascii')
        candidates = [int(item) for item in raw.split()]
    except (OSError, ValueError):
        return None
    for pid in candidates:
        data = _proc_data(pid)
        expected_cwd = isaac_dir if isaac_dir is not None else data['cwd'] if data else None
        if (data and data['ppid'] == launcher.pid and data['pgrp'] == pid
                and data['cwd'] == expected_cwd and data['cwd'].parent == ISAAC_ROOT
                and b'run_ros_cartpole.py' in data['cmdline']):
            return {'pid': pid, 'start_ticks': data['start_ticks'], 'cwd': expected_cwd}
    return None


def _exact_child_alive(identity):
    if identity is None:
        return False
    data = _proc_data(identity['pid'])
    return bool(data and data['state'] != 'Z'
                and data['start_ticks'] == identity['start_ticks']
                and data['cwd'] == identity['cwd']
                and data['pgrp'] == identity['pid']
                and b'run_ros_cartpole.py' in data['cmdline'])


def _stop_controller(process):
    if process is None or process.poll() is not None:
        return
    # SIGINT lets the ROS node publish its final zero commands.
    for sig, delay in ((signal.SIGINT, 8), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        if process.poll() is not None:
            break
        process.send_signal(sig)
        try:
            process.wait(timeout=delay)
        except subprocess.TimeoutExpired:
            continue


def _launcher_children(launcher):
    """Capture direct children before the launcher can reparent them."""
    try:
        raw = (Path('/proc') / str(launcher.pid) / 'task' / str(launcher.pid) /
               'children').read_text(encoding='ascii')
        return {int(item): _proc_data(int(item)) for item in raw.split()}
    except (OSError, ValueError):
        return None


def _stop_isaac(launcher, identity, isaac_dir=None):
    if launcher is None:
        return True
    children_before = _launcher_children(launcher) if launcher.poll() is None else None
    if identity is None and launcher.poll() is None:
        identity = _remember_child(launcher, isaac_dir)
    uncertain_child = (children_before is None and identity is None)
    if children_before:
        for pid, data in children_before.items():
            if identity is None or pid != identity['pid'] or data is None:
                uncertain_child = True
    if launcher.poll() is None:
        launcher.terminate()  # isaac_entry forwards TERM to its own Sim child group.
        try:
            launcher.wait(timeout=25)
        except subprocess.TimeoutExpired:
            pass
    if _exact_child_alive(identity):
        os.killpg(identity['pid'], signal.SIGTERM)
        until = time.monotonic() + 10
        while _exact_child_alive(identity) and time.monotonic() < until:
            time.sleep(0.2)
    if _exact_child_alive(identity):
        os.killpg(identity['pid'], signal.SIGKILL)
        until = time.monotonic() + 5
        while _exact_child_alive(identity) and time.monotonic() < until:
            time.sleep(0.2)
    if launcher.poll() is None:
        launcher.kill()  # Only the exact Popen child created by this runner.
    try:
        launcher.wait(timeout=5)
    except subprocess.TimeoutExpired:
        return False
    # A PID alone never proves a former child has exited; compare start ticks.
    if children_before:
        for pid, before in children_before.items():
            after = _proc_data(pid)
            if before and after and before['start_ticks'] == after['start_ticks'] and after['state'] != 'Z':
                return False
    return not uncertain_child and not _exact_child_alive(identity)

def _read_output(stream, log_path, output_queue):
    with log_path.open('x', encoding='utf-8') as log:
        for line in stream:
            log.write(line)
            log.flush()
            if line.startswith('OUTPUT='):
                output_queue.put(line.removeprefix('OUTPUT=').strip())


def _validate_isaac_dir(path, case):
    run = path.resolve(strict=True)
    if run.parent != ISAAC_ROOT.resolve(strict=True):
        raise RuntimeError(f'Launcher reported an unexpected run directory: {run}')
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    argv = manifest.get('argv', [])
    if manifest.get('profile') != 'ros' or not any(str(item).endswith('/run_ros_cartpole.py') for item in argv):
        raise RuntimeError('Launcher manifest is not the AstrEX ROS Cartpole profile')
    for flag, value in (('--trial-x0', case.x0), ('--trial-theta0', case.theta0),
                        ('--trial-hold-position', case.hold_position)):
        try:
            if not math.isclose(float(argv[argv.index(flag) + 1]), value, abs_tol=1e-9):
                raise ValueError('value mismatch')
        except (ValueError, IndexError, TypeError) as exc:
            raise RuntimeError(f'Launcher manifest has wrong {flag}') from exc
    if '--headless' not in argv:
        raise RuntimeError('Trial launcher must be headless')
    return run


def _wait_for_ready(launcher, case, output_queue, deadline):
    run = None
    identity = None
    while time.monotonic() < deadline:
        if run is None:
            try:
                run = _validate_isaac_dir(Path(output_queue.get(timeout=0.25)), case)
                print(f'ISAAC_OUTPUT={run}', flush=True)
            except queue.Empty:
                if launcher.poll() is not None:
                    raise RuntimeError(f'Isaac launcher exited {launcher.returncode} before OUTPUT')
                continue
        if identity is None:
            identity = _remember_child(launcher, run)
        ready_path = run / 'ready.json'
        if ready_path.exists():
            try:
                ready = json.loads(ready_path.read_text(encoding='utf-8'))
                pid = int(ready['pid'])
            except (OSError, ValueError, KeyError, TypeError) as exc:
                raise RuntimeError('Isaac ready.json is invalid') from exc
            if not ready.get('graph', '').startswith('/World/AstrEXROSGraph_'):
                raise RuntimeError('Isaac ready graph is not the expected ROS graph')
            if identity is None:
                identity = _remember_child(launcher, run)
            if identity is None or identity['pid'] != pid or not _exact_child_alive(identity):
                raise RuntimeError('Isaac ready PID is not the child of this launcher')
            return run, identity
        if launcher.poll() is not None:
            raise RuntimeError(f'Isaac launcher exited {launcher.returncode} before ready.json')
        time.sleep(0.25)
    raise TimeoutError('Isaac did not report ready within 600 seconds')


def _system_ros_command(isaac_dir, trial_dir):
    overlay = ROOT / 'ros2_ws/install/setup.bash'
    if not overlay.is_file():
        raise RuntimeError('ROS overlay is missing; build the workspace before the matrix')
    script = ('set -eo pipefail; source /opt/ros/jazzy/setup.bash; '
              'source "$1"; shift; set -u; '
              'exec /usr/bin/python3 -B -m astrex_ros_bridge.balance_hold_closed_loop_node "$@"')
    return ['bash', '--noprofile', '--norc', '-c', script, 'matrix-ros', str(overlay),
            '--isaac-run-dir', str(isaac_dir), '--trial-dir', str(trial_dir)]


def _zero_evidence(isaac_dir, trial_dir, deadline):
    """Wait for flushed Isaac samples beyond the controller exit, including zero."""
    try:
        result = json.loads((trial_dir / 'result.json').read_text(encoding='utf-8'))
        exit_sim = int(result['final_state']['stamp_ns']) / 1e9
    except (OSError, ValueError, KeyError, TypeError):
        return False
    samples_path = isaac_dir / 'ros_samples.jsonl'
    while time.monotonic() < deadline:
        late = False
        zero = False
        try:
            with samples_path.open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                        t = float(row['simulation_time'])
                        if t <= exit_sim or t > exit_sim + 0.20 + 1e-9:
                            if t >= exit_sim + 0.20:
                                late = True
                            continue
                        names = row['joint_names']
                        effort = row['controller_effort']
                        cart = float(effort[names.index('slider_to_cart')])
                        pole = float(effort[names.index('cart_to_pole')])
                        if math.isfinite(cart) and math.isfinite(pole) and abs(cart) <= 1e-9 and abs(pole) <= 1e-9:
                            zero = True
                    except (json.JSONDecodeError, KeyError, ValueError, TypeError, IndexError):
                        continue  # A concurrent final partial line can appear before the flush.
        except OSError:
            pass
        if zero and late:
            return True
        time.sleep(0.25)
    return False


def _new_attempt(run_dir, case):
    case_dir = run_dir / case.name
    case_dir.mkdir(exist_ok=True)
    for number in range(1, 1000):
        attempt = case_dir / f'attempt_{number:03d}'
        try:
            attempt.mkdir(exist_ok=False)
            return attempt
        except FileExistsError:
            continue
    raise RuntimeError(f'Too many attempts for {case.name}')


def run_case(run_dir, case):
    attempt = _new_attempt(run_dir, case)
    started = utc_now()
    launcher = None
    controller = None
    reader = None
    isaac_dir = None
    identity = None
    controller_code = None
    assessment_code = None
    assessment = None
    zero_observed = False
    error = None
    cleanup_ok = False
    try:
        command = [str(ROOT / 'scripts/start_isaac_ros.sh'), '--headless',
                   '--trial-x0', repr(case.x0), '--trial-theta0', repr(case.theta0),
                   '--trial-hold-position', repr(case.hold_position)]
        (attempt / 'launcher_argv.json').write_text(json.dumps(command, indent=2) + '\n')
        output_queue = queue.Queue()
        launcher = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding='utf-8', errors='replace', bufsize=1,
                                    start_new_session=True)
        deadline = time.monotonic() + MAX_ISAAC_WALL_SEC
        reader = threading.Thread(target=_read_output,
                                  args=(launcher.stdout, attempt / 'launcher_console.log', output_queue),
                                  daemon=True)
        reader.start()
        isaac_dir, identity = _wait_for_ready(launcher, case, output_queue, deadline)
        trial_dir = attempt / 'trial'
        ros_logs = attempt / 'ros_logs'
        ros_logs.mkdir(exist_ok=False)
        env = {'HOME': os.environ['HOME'], 'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8',
               'ROS_DOMAIN_ID': FIXED_DOMAIN, 'RMW_IMPLEMENTATION': FIXED_RMW,
               'ROS_LOG_DIR': str(ros_logs), 'PYTHONDONTWRITEBYTECODE': '1'}
        ros_command = _system_ros_command(isaac_dir, trial_dir) + [
            '--x0', repr(case.x0), '--theta0', repr(case.theta0),
            '--hold-position', repr(case.hold_position)]
        (attempt / 'controller_argv.json').write_text(json.dumps(ros_command, indent=2) + '\n')
        with (attempt / 'controller_console.log').open('x', encoding='utf-8') as log:
            controller = subprocess.Popen(ros_command, cwd=ROOT, env=env,
                                          stdin=subprocess.DEVNULL, stdout=log,
                                          stderr=subprocess.STDOUT, start_new_session=True)
            controller.wait(timeout=min(MAX_CONTROLLER_WALL_SEC,
                                        max(0.1, deadline - time.monotonic())))
            controller_code = controller.returncode
        zero_observed = _zero_evidence(isaac_dir, trial_dir,
                                       min(deadline, time.monotonic() + MAX_ZERO_EVIDENCE_WALL_SEC))
        if trial_dir.is_dir():
            assessor = subprocess.run(['/usr/bin/python3', '-B',
                str(ROOT / 'scripts/analyze_balance_hold_trial.py'), str(trial_dir)],
                cwd=ROOT, capture_output=True, text=True, timeout=30)
            (attempt / 'assessor_console.log').write_text(assessor.stdout + assessor.stderr)
            assessment_code = assessor.returncode
            if (trial_dir / 'assessment.json').is_file():
                assessment = json.loads((trial_dir / 'assessment.json').read_text(encoding='utf-8'))
        if not zero_observed:
            error = 'No flushed zero controller-effort sample plus 0.2 sim-second tail'
        elif controller_code != 0:
            error = f'ROS controller exited {controller_code}'
        elif assessment_code != 0 or not assessment or not assessment.get('passed'):
            error = f'B assessment failed (exit {assessment_code})'
    except (OSError, ValueError, RuntimeError, TimeoutError, subprocess.TimeoutExpired) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        _stop_controller(controller)
        cleanup_ok = _stop_isaac(launcher, identity, isaac_dir)
        if reader is not None:
            reader.join(timeout=5)
        if not cleanup_ok:
            error = (error + '; ' if error else '') + 'Exact spawned Isaac process did not stop'
    passed = error is None and cleanup_ok and bool(assessment and assessment.get('passed'))
    record = {'case': asdict(case), 'status': 'PASS' if passed else 'FAIL',
              'started_utc': started, 'finished_utc': utc_now(),
              'attempt_dir': str(attempt), 'trial_dir': str(attempt / 'trial'),
              'isaac_run_dir': str(isaac_dir) if isaac_dir else None,
              'controller_returncode': controller_code,
              'assessor_returncode': assessment_code,
              'post_exit_zero_observed': zero_observed,
              'assessment': assessment, 'error': error}
    (attempt / 'runner_result.json').write_text(
        json.dumps(record, indent=2, ensure_ascii=False, allow_nan=False) + '\n', encoding='utf-8')
    return record


def _new_manifest(accepted_trials):
    accepted = []
    if len(accepted_trials) > len(CASES):
        raise ValueError('More completed trials than the fixed matrix contains')
    for case, supplied in zip(CASES, accepted_trials):
        trial_dir = supplied.resolve(strict=True)
        assessment = verified_pass(trial_dir, case)
        if assessment is None:
            raise ValueError(f'The supplied trial is not a verified PASS for {case.name}: {trial_dir}')
        accepted.append((case, trial_dir, assessment))
    run_dir = MATRIX_ROOT / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') +
                             uuid.uuid4().hex[:10])
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {'matrix_version': VERSION, 'created_utc': utc_now(),
                'fingerprints': fingerprints(), 'cases': [asdict(case) for case in CASES],
                'results': {}, 'status': 'IN_PROGRESS'}
    for case, trial_dir, assessment in accepted:
        manifest['results'][case.name] = [{
            'case': asdict(case), 'status': 'PASS', 'external_prior_pass': True,
            'trial_dir': str(trial_dir),
            'isaac_run_dir': str(Path(json.loads((trial_dir / 'config.json').read_text())['isaac_run_dir'])),
            'assessment': assessment, 'accepted_utc': utc_now(),
            'fingerprints': manifest['fingerprints']}]
    atomic_json(run_dir / 'manifest.json', manifest)
    return run_dir, manifest


def _resume_manifest(run_dir):
    run_dir = run_dir.resolve(strict=True)
    if run_dir.parent != MATRIX_ROOT.resolve(strict=True):
        raise ValueError('Resume directory is not a direct matrix-run directory')
    manifest = json.loads((run_dir / 'manifest.json').read_text(encoding='utf-8'))
    if manifest.get('matrix_version') != VERSION or manifest.get('cases') != [asdict(case) for case in CASES]:
        raise ValueError('Saved matrix plan does not match the fixed B plan')
    return run_dir, manifest


def _can_skip(manifest, case):
    for record in reversed(manifest['results'].get(case.name, [])):
        if record.get('status') != 'PASS' or record.get('case') != asdict(case):
            continue
        trial_dir = Path(record['trial_dir'])
        if verified_pass(trial_dir, case) is not None:
            return True
    return False


def _write_table(run_dir, manifest):
    lines = ['# BalanceHold B matrix', '',
             '| Case | Batch | Hold (m) | x0 (m) | theta0 (deg) | Status | Trial evidence |',
             '| --- | ---: | ---: | ---: | ---: | --- | --- |']
    for case in CASES:
        records = manifest['results'].get(case.name, [])
        record = records[-1] if records else None
        status = record['status'] if record else 'NOT RUN'
        trial = record.get('trial_dir', '') if record else ''
        lines.append(f'| {case.name} | {case.batch} | {case.hold_position:+.2f} | '
                     f'{case.x0:+.2f} | {math.degrees(case.theta0):+.0f} | {status} | {trial} |')
    (run_dir / 'results.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--accept-completed', type=Path, action='append', default=[],
                       metavar='TRIAL_DIR', help='Reuse verified PASS prefix cases; repeat in matrix order')
    group.add_argument('--resume-run-dir', type=Path, metavar='MATRIX_RUN_DIR')
    parser.add_argument('--list-plan', action='store_true', help='Show fixed cases without starting Sim')
    args = parser.parse_args(argv)
    if args.list_plan:
        for case in CASES:
            print(json.dumps(asdict(case), allow_nan=False))
        return 0
    if args.resume_run_dir:
        run_dir, manifest = _resume_manifest(args.resume_run_dir)
    else:
        run_dir, manifest = _new_manifest(args.accept_completed)
    print(f'MATRIX_OUTPUT={run_dir}', flush=True)
    if manifest.get('fingerprints') != fingerprints():
        print('WARNING: code fingerprint changed; saved PASS cases will be rechecked from raw evidence', flush=True)
    _write_table(run_dir, manifest)
    for case in CASES:
        if _can_skip(manifest, case):
            print(f'SKIP verified PASS {case.name}', flush=True)
            continue
        print(f'START {case.name}: hold={case.hold_position:+.3f} m, '
              f'x0={case.x0:+.3f} m, theta0={math.degrees(case.theta0):+.1f} deg', flush=True)
        record = run_case(run_dir, case)
        record['fingerprints'] = fingerprints()
        manifest['results'].setdefault(case.name, []).append(record)
        manifest['status'] = 'IN_PROGRESS' if record['status'] == 'PASS' else 'STOPPED_ON_FAILURE'
        atomic_json(run_dir / 'manifest.json', manifest)
        _write_table(run_dir, manifest)
        print(f"{record['status']} {case.name}: {record.get('error') or record['trial_dir']}", flush=True)
        if record['status'] != 'PASS':
            return 1
    manifest['status'] = 'PASS' if all(_can_skip(manifest, case) for case in CASES) else 'INCOMPLETE'
    atomic_json(run_dir / 'manifest.json', manifest)
    _write_table(run_dir, manifest)
    print(f"MATRIX_STATUS={manifest['status']}", flush=True)
    return 0 if manifest['status'] == 'PASS' else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'MATRIX ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc
