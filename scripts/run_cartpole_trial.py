#!/usr/bin/env python3
"""Run current CartPole trials with exact process ownership and isolated evidence.

Default: three GUI cases in order, stopping on failure. --mode dry-run
uses one headless, pinned-zero Isaac session and three read-only ROS nodes.
This runner does not import any retired Step 3 experiment tool.
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

from analyze_cartpole_trial import assess_trial

ROOT = Path(__file__).resolve().parents[1]
SHARED_ROOT = Path('/data/shared/AstrEX_project_data/logs/isaac')
RUN_ROOT = SHARED_ROOT / 'cartpole_step4' / 'runs'
ISAAC_ROOT = SHARED_ROOT / 'ros'
MAX_ISAAC_WALL_SEC = 600.0
MAX_ZERO_EVIDENCE_WALL_SEC = 30.0
FORCE_LIMIT_N = 5.0
FIXED_DOMAIN = '63'
FIXED_RMW = 'rmw_fastrtps_cpp'


@dataclass(frozen=True)
class Case:
    name: str
    task: str
    target: float
    x0: float = 0.0
    theta0: float = 0.0
    hold_position: float = 0.0


CASES = (
    Case('M1', 'move', 0.3),
    Case('M2', 'move', -0.3),
    Case('M3', 'move_then_hold', 0.5, theta0=math.radians(2.0)),
)
DRY_CASES = tuple(Case(f'D{i+1}', 'move', target)
                  for i, target in enumerate((0.3, -0.3, 0.5)))


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def fingerprints():
    paths = ('scripts/run_cartpole_trial.py', 'scripts/analyze_cartpole_trial.py',
             'sim/scripts/run_ros_cartpole.py', 'scripts/lib/isaac_entry.py',
             'ros2_ws/src/astrex_ros_bridge/astrex_ros_bridge/balance_hold_closed_loop_node.py',
             'ros2_ws/src/astrex_ros_bridge/astrex_ros_bridge/balance_hold_controller.py',
             'config/isaac_baseline.env')
    return {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in paths}


def atomic_json(path, value):
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


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



def launcher_command(case, *, headless=False, reference_speed=0.05):
    command = [str(ROOT / 'scripts/start_isaac_ros.sh')]
    if headless:
        command.append('--headless')
    command += [
        '--trial-x0', repr(case.x0), '--trial-theta0', repr(case.theta0),
        '--trial-hold-position', repr(case.hold_position),
    ]
    if case.task != 'hold':
        command += ['--trial-task', case.task, '--trial-target', repr(case.target),
                    '--trial-reference-speed', repr(reference_speed)]
    return command


def _validate_isaac_dir(path, case, *, headless, reference_speed):
    run = path.resolve(strict=True)
    if run.parent != ISAAC_ROOT.resolve(strict=True):
        raise RuntimeError(f'Unexpected Isaac output directory: {run}')
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    argv = manifest.get('argv', [])
    if manifest.get('profile') != 'ros' or not any(
            str(item).endswith('/run_ros_cartpole.py') for item in argv):
        raise RuntimeError('Launcher did not use the official ROS Cartpole profile')
    expected = {
        '--trial-x0': case.x0, '--trial-theta0': case.theta0,
        '--trial-hold-position': case.hold_position,
    }
    if case.task != 'hold':
        expected.update({'--trial-target': case.target, '--trial-reference-speed': reference_speed})
    for flag, value in expected.items():
        try:
            actual = float(argv[argv.index(flag) + 1])
            if not math.isclose(actual, value, abs_tol=1e-9):
                raise ValueError('value mismatch')
        except (ValueError, IndexError, TypeError) as exc:
            raise RuntimeError(f'Launcher manifest has wrong {flag}') from exc
    if case.task != 'hold':
        try:
            if argv[argv.index('--trial-task') + 1] != case.task:
                raise ValueError('task mismatch')
        except (ValueError, IndexError) as exc:
            raise RuntimeError('Launcher manifest has wrong trial task') from exc
    if ('--headless' in argv) != headless:
        raise RuntimeError('Launcher GUI/headless mode differs from requested profile')
    return run


def _wait_for_ready(launcher, case, output_queue, deadline, *, headless, reference_speed):
    run = None
    identity = None
    while time.monotonic() < deadline:
        if run is None:
            try:
                supplied = Path(output_queue.get(timeout=0.25))
                run = _validate_isaac_dir(supplied, case, headless=headless,
                                          reference_speed=reference_speed)
                print(f'ISAAC_OUTPUT={run}', flush=True)
            except queue.Empty:
                if launcher.poll() is not None:
                    raise RuntimeError(f'Isaac exited {launcher.returncode} before OUTPUT')
                continue
        if identity is None:
            identity = _remember_child(launcher, run)
        ready_path = run / 'ready.json'
        if ready_path.exists():
            ready = json.loads(ready_path.read_text(encoding='utf-8'))
            if not str(ready.get('graph', '')).startswith('/World/AstrEXROSGraph_'):
                raise RuntimeError('Isaac ready graph is not the expected graph')
            if identity is None:
                identity = _remember_child(launcher, run)
            if identity is None or identity['pid'] != int(ready['pid']) or not _exact_child_alive(identity):
                raise RuntimeError('Isaac ready PID is not our exact launcher child')
            if ready.get('device') != 'cpu' or bool(ready.get('gui')) != (not headless):
                raise RuntimeError('Isaac did not report requested CPU/GUI profile')
            return run, identity
        if launcher.poll() is not None:
            raise RuntimeError(f'Isaac exited {launcher.returncode} before ready.json')
        time.sleep(0.25)
    raise TimeoutError('Isaac did not become ready inside its 600-second limit')


def _system_ros_command(isaac_dir, trial_dir, case, *, dry_run, reference_speed):
    overlay = ROOT / 'ros2_ws/install/setup.bash'
    if not overlay.is_file():
        raise RuntimeError('ROS overlay is missing; build astrex_ros_bridge first')
    script = ('set -eo pipefail; source /opt/ros/jazzy/setup.bash; '
              'source "$1"; shift; set -u; '
              'exec /usr/bin/python3 -B -m astrex_ros_bridge.balance_hold_closed_loop_node "$@"')
    command = ['bash', '--noprofile', '--norc', '-c', script, 'cartpole-trial', str(overlay),
               '--isaac-run-dir', str(isaac_dir), '--trial-dir', str(trial_dir),
               '--x0', repr(case.x0), '--theta0', repr(case.theta0),
               '--hold-position', repr(case.hold_position), '--task', case.task,
               '--reference-speed', repr(reference_speed)]
    if case.task != 'hold':
        command += ['--target', repr(case.target)]
    if dry_run:
        command.append('--dry-run')
    return command


def _zero_evidence(isaac_dir, trial_dir, deadline):
    """Require a flushed zero Controller sample followed by a 0.2-s simulation tail."""
    try:
        result = json.loads((trial_dir / 'result.json').read_text(encoding='utf-8'))
        exit_sim = int(result['final_state']['stamp_ns']) / 1e9
    except (OSError, ValueError, KeyError, TypeError):
        return False
    while time.monotonic() < deadline:
        late = zero = False
        try:
            with (isaac_dir / 'ros_samples.jsonl').open(encoding='utf-8') as stream:
                for line in stream:
                    try:
                        row = json.loads(line)
                        t = float(row['simulation_time'])
                        if t >= exit_sim + 0.2:
                            late = True
                        if not exit_sim < t <= exit_sim + 0.2 + 1e-9:
                            continue
                        names, forces = row['joint_names'], row['controller_effort']
                        selected = [float(forces[names.index(name)]) for name in (
                            'slider_to_cart', 'cart_to_pole')]
                        if all(math.isfinite(value) and abs(value) <= 1e-9 for value in selected):
                            zero = True
                    except (json.JSONDecodeError, KeyError, ValueError, TypeError, IndexError):
                        continue
        except OSError:
            pass
        if zero and late:
            return True
        time.sleep(0.25)
    return False


def _wait_gui_continue(run_dir, launcher, deadline):
    marker = run_dir / 'gui_continue.json'
    print(f'GUI_WAITING_FOR_CONFIRMATION={marker}', flush=True)
    while time.monotonic() < deadline:
        if launcher.poll() is not None:
            raise RuntimeError('Isaac exited during GUI viewing preparation')
        if marker.exists():
            value = json.loads(marker.read_text(encoding='utf-8'))
            if not isinstance(value, dict) or value.get('continue') is not True:
                raise RuntimeError('gui_continue.json must contain {"continue": true}')
            print('GUI_VIEWING_READY', flush=True)
            return
        time.sleep(0.25)
    raise TimeoutError('GUI viewing preparation exhausted the process limit')


def _run_controller(attempt, isaac_dir, case, deadline, *, dry_run, reference_speed):
    trial_dir = attempt / 'trial'
    ros_logs = attempt / 'ros_logs'
    ros_logs.mkdir(exist_ok=False)
    env = {'HOME': os.environ['HOME'], 'PATH': '/usr/bin:/bin', 'LANG': 'C.UTF-8',
           'ROS_DOMAIN_ID': FIXED_DOMAIN, 'RMW_IMPLEMENTATION': FIXED_RMW,
           'ROS_LOG_DIR': str(ros_logs), 'PYTHONDONTWRITEBYTECODE': '1'}
    command = _system_ros_command(isaac_dir, trial_dir, case, dry_run=dry_run,
                                  reference_speed=reference_speed)
    (attempt / 'controller_argv.json').write_text(json.dumps(command, indent=2) + '\n')
    process = None
    wait_error = None
    try:
        with (attempt / 'controller_console.log').open('x', encoding='utf-8') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdin=subprocess.DEVNULL,
                                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                process.wait(timeout=max(0.1, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                wait_error = 'ROS node exhausted the 600-second Isaac process limit'
                _stop_controller(process)
        code = process.returncode
    finally:
        _stop_controller(process)
    zero = _zero_evidence(isaac_dir, trial_dir,
                          min(deadline, time.monotonic() + MAX_ZERO_EVIDENCE_WALL_SEC))
    try:
        report = assess_trial(trial_dir)
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        report = {'trial_id': isaac_dir.name, 'task': case.task, 'dry_run': dry_run,
                  'passed': False, 'end_reason': 'ANALYSIS_ERROR',
                  'issues': [f'partial_evidence:{type(exc).__name__}:{exc}']}
        if not trial_dir.exists():
            trial_dir.mkdir(exist_ok=False)
    atomic_json(trial_dir / 'assessment.json', report)
    error = wait_error
    if error is None and code != 0:
        error = f'ROS node exited {code}'
    elif error is None and not zero:
        error = 'No flushed Controller zero and 0.2 simulation-second tail'
    elif error is None and not report['passed']:
        error = 'Independent assessment failed: ' + ', '.join(report['issues'])
    return {'case': asdict(case), 'status': 'PASS' if error is None else 'FAIL',
            'trial_dir': str(trial_dir), 'isaac_run_dir': str(isaac_dir),
            'controller_returncode': code, 'post_exit_zero_observed': zero,
            'assessment': report, 'error': error}


def run_session(run_dir, cases, *, dry_run=False, reference_speed=0.05,
                wait_before_start=False, keep_gui_seconds=0.0):
    """One Sim for dry-run; one Sim for each GUI case. Never reuse a controlled scene."""
    attempt = run_dir / ('dry_session' if dry_run else cases[0].name)
    attempt.mkdir(exist_ok=False)
    scene_case = Case('dry_scene', 'move', 0.0) if dry_run else cases[0]
    launcher = reader = identity = isaac_dir = None
    results = []
    cleanup_ok = False
    error = None
    try:
        command = launcher_command(scene_case, headless=dry_run,
                                   reference_speed=reference_speed)
        (attempt / 'launcher_argv.json').write_text(json.dumps(command, indent=2) + '\n')
        outputs = queue.Queue()
        launcher = subprocess.Popen(command, cwd=ROOT, stdin=subprocess.DEVNULL,
                                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                    text=True, encoding='utf-8', errors='replace', bufsize=1,
                                    start_new_session=True)
        deadline = time.monotonic() + MAX_ISAAC_WALL_SEC
        reader = threading.Thread(target=_read_output,
                                  args=(launcher.stdout, attempt / 'launcher_console.log', outputs),
                                  daemon=True)
        reader.start()
        isaac_dir, identity = _wait_for_ready(launcher, scene_case, outputs, deadline,
                                             headless=dry_run, reference_speed=reference_speed)
        if wait_before_start and not dry_run:
            _wait_gui_continue(run_dir, launcher, deadline)
        for case in cases:
            case_attempt = attempt / case.name if dry_run else attempt
            if dry_run:
                case_attempt.mkdir(exist_ok=False)
            print(f'START {case.name}: task={case.task}, target={case.target:+.3f} m, '
                  f'theta0={math.degrees(case.theta0):+.1f} deg, output={case_attempt}', flush=True)
            if case.task == 'move_then_hold' and not dry_run:
                print('GUI_FINAL_DEMONSTRATION_STARTING: observe movement and the 3-second hold; '
                      'physical acceptance comes from logs', flush=True)
            record = _run_controller(case_attempt, isaac_dir, case, deadline,
                                     dry_run=dry_run, reference_speed=reference_speed)
            results.append(record)
            print(f"{record['status']} {case.name}: {record['error'] or record['trial_dir']}", flush=True)
            atomic_json(case_attempt / 'runner_result.json', record)
            if record['status'] != 'PASS':
                break
        if not dry_run and results and results[-1]['status'] == 'PASS' and keep_gui_seconds:
            print(f'GUI_SUCCESS_OBSERVATION={keep_gui_seconds:.1f}s; control has ended at zero', flush=True)
            until = min(deadline, time.monotonic() + keep_gui_seconds)
            while time.monotonic() < until:
                if launcher.poll() is not None:
                    raise RuntimeError('Isaac exited during final GUI observation')
                time.sleep(0.25)
    except (OSError, ValueError, KeyError, RuntimeError, TimeoutError, subprocess.TimeoutExpired) as exc:
        error = f'{type(exc).__name__}: {exc}'
    finally:
        cleanup_ok = _stop_isaac(launcher, identity, isaac_dir)
        if reader is not None:
            reader.join(timeout=5)
        if not cleanup_ok:
            error = (error + '; ' if error else '') + 'Exact spawned Isaac process did not stop'
    if error is not None:
        results.append({'case': asdict(cases[len(results)]) if len(results) < len(cases) else asdict(cases[-1]),
                        'status': 'FAIL', 'error': error,
                        'isaac_run_dir': str(isaac_dir) if isaac_dir else None,
                        'cleanup_verified': cleanup_ok})
    for result in results:
        result['cleanup_verified'] = cleanup_ok
        if not cleanup_ok:
            result['status'] = 'FAIL'
    atomic_json(attempt / 'session_result.json', {'results': results, 'cleanup_verified': cleanup_ok})
    return results


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('gui', 'dry-run'), default='gui')
    parser.add_argument('--case', choices=('all', 'M1', 'M2', 'M3', 'hold', 'hold_regression'), default='all')
    parser.add_argument('--reference-speed', type=float, default=0.05)
    parser.add_argument('--hold-position', type=float, default=0.0)
    parser.add_argument('--x0', type=float, default=0.0)
    parser.add_argument('--theta0', type=float, default=0.0, help='radians, hold representative only')
    parser.add_argument('--wait-before-start', action='store_true')
    parser.add_argument('--keep-gui-seconds', type=float, default=15.0)
    parser.add_argument('--list-plan', action='store_true')
    args = parser.parse_args(argv)
    if args.reference_speed not in (0.05, 0.03):
        parser.error('reference speed must be 0.05 or the explicitly selected 0.03 m/s')
    if not math.isfinite(args.keep_gui_seconds) or not 0 <= args.keep_gui_seconds <= 60:
        parser.error('GUI observation duration must be in [0, 60] local seconds')
    if args.mode == 'dry-run' and args.case != 'all':
        parser.error('strict dry-run always runs its three fixed calculations')
    cases = DRY_CASES if args.mode == 'dry-run' else CASES
    if args.mode == 'gui' and args.case == 'hold_regression':
        cases = (Case('hold_regression', 'hold', 0.0, theta0=math.radians(2.0)),)
    elif args.mode == 'gui' and args.case == 'hold':
        if not all(math.isfinite(v) for v in (args.hold_position, args.x0, args.theta0)):
            parser.error('hold initial state and target must be finite')
        cases = (Case('hold', 'hold', args.hold_position, args.x0,
                      args.theta0, args.hold_position),)
    elif args.mode == 'gui' and args.case != 'all':
        cases = tuple(case for case in cases if case.name == args.case)
    if args.list_plan:
        print(json.dumps({'mode': args.mode, 'reference_speed': args.reference_speed,
                          'cases': [asdict(case) for case in cases]}, indent=2))
        return 0
    run_dir = RUN_ROOT / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10])
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {'created_utc': utc_now(), 'mode': args.mode, 'fingerprints': fingerprints(),
                'reference_speed': args.reference_speed, 'cases': [asdict(case) for case in cases],
                'results': [], 'status': 'IN_PROGRESS'}
    atomic_json(run_dir / 'manifest.json', manifest)
    print(f'CARTPOLE_OUTPUT={run_dir}', flush=True)
    if args.mode == 'dry-run':
        records = run_session(run_dir, cases, dry_run=True, reference_speed=args.reference_speed)
        manifest['results'].extend(records)
    else:
        for index, case in enumerate(cases):
            records = run_session(run_dir, (case,), reference_speed=args.reference_speed,
                                  wait_before_start=args.wait_before_start and index == 0,
                                  keep_gui_seconds=args.keep_gui_seconds if case.task == 'move_then_hold' else 0)
            manifest['results'].extend(records)
            atomic_json(run_dir / 'manifest.json', manifest)
            if not records or any(record['status'] != 'PASS' for record in records):
                break
    complete = len(manifest['results']) == len(cases) and all(
        result['status'] == 'PASS' for result in manifest['results'])
    manifest['status'] = 'PASS' if complete else 'FAIL'
    manifest['finished_utc'] = utc_now()
    atomic_json(run_dir / 'manifest.json', manifest)
    print(f"CARTPOLE_STATUS={manifest['status']}", flush=True)
    return 0 if complete else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, KeyboardInterrupt) as exc:
        print(f'CARTPOLE RUN ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc
