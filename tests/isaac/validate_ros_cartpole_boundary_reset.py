"""Boundary and reset acceptance for the production Cartpole ROS runtime (Isaac side).

Run through the production launcher environment, with the GUI display available:

    bash --noprofile --norc -c 'source scripts/lib/isaac_common.sh; \
        exec python -B tests/isaac/validate_ros_cartpole_boundary_reset.py --work-dir <dir>'

Finite-state boundary placement exists only in this file. The production runtime, the
official Cartpole task, its thresholds and the daily ROS path are untouched. The external
driver is ``tests/isaac/ros_system_regression.py`` started as a clean system-Python
subprocess: the two Python environments never import each other.

Accepted invariants (repeated resets):
  * AstrEX ROS graph prims = 1 (only /World/AstrEXROSGraph_* OmniGraph prims are asserted
    on; other OmniGraph prims are recorded separately), graph_path constant, graph
    creations = 1, reset counter increasing.
  * /clock publisher = 1, /joint_states publisher = 1, /joint_command subscriber = 1.
  * Every reset: command_gate_closed before the boundary, command_gate_opened only after
    the state verification, no reset_failed.
  * After reset with no new ROS command: >=2 s static cart and zero articulation effort
    target; then a new same-value command controls, and the reverse command reverses.
  * /clock strictly increasing over the whole run.
"""

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/isaac_baseline.env'
DRIVER = ROOT / 'tests/isaac/ros_system_regression.py'
PRODUCTION = ROOT / 'sim/scripts/run_ros_cartpole.py'

PHYSICS_DT = 1.0 / 120.0
BOUNDARY_STEP_LIMIT = 300
IDLE_STEPS = 260
RECOVERY_STEPS = 30
SETTLE_STEPS = 45
POSITION_GATE = 1e-3
VELOCITY_GATE = 1e-2
MOTION_POSITION_GATE = 1e-3
MOTION_VELOCITY_GATE = 1e-2
EFFORT_TARGET_GATE = 1e-2
HOLD_SECONDS = 0.3
WINDOW_BURST_SECONDS = 0.12
GRAPH_PREFIX = '/World/AstrEXROSGraph_'
RESET_EFFORT_NOTE = ('N/A: PhysX applied/computed effort is not scoped to the '
                     'ROS->Controller path; the reset gate uses the articulation effort target '
                     'read-back, state zeroing, the no-replay window and command recovery')
_STAGE = {}


def write_json(path, value):
    path = Path(path)
    tmp = Path(str(path) + '.tmp')
    tmp.write_text(json.dumps(value, indent=2, allow_nan=False))
    tmp.replace(path)


def read_json(path):
    path = Path(path)
    if not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except (OSError, json.JSONDecodeError):
        return None


def read_jsonl(path):
    path = Path(path)
    if not path.exists():
        return []
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            continue
    return rows


def config_value(key):
    for line in CONFIG.read_text().splitlines():
        line = line.strip()
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip().strip("'\"")
    raise RuntimeError(f'{key} missing from {CONFIG}')


def prepare_ros_environment():
    """Mirror the Sim-side ROS environment that scripts/lib/isaac_entry.py builds."""
    import importlib.metadata as metadata
    bridge = Path(metadata.distribution('isaacsim').locate_file('isaacsim')) / 'exts/isaacsim.ros2.bridge'
    if not (bridge / 'jazzy/lib').is_dir():
        raise RuntimeError('bundled Jazzy bridge libraries missing: ' + str(bridge))
    os.environ['ROS_DISTRO'] = config_value('ASTREX_ROS_DISTRO')
    os.environ['ROS_DOMAIN_ID'] = config_value('ASTREX_ROS_DOMAIN_ID')
    os.environ['RMW_IMPLEMENTATION'] = config_value('ASTREX_RMW_IMPLEMENTATION')
    os.environ['LD_LIBRARY_PATH'] = str(bridge / 'jazzy/lib')
    return {'python': sys.executable, 'conda_env': os.environ.get('ASTREX_ISAAC_CONDA_ENV'),
            'domain': os.environ['ROS_DOMAIN_ID'], 'rmw': os.environ['RMW_IMPLEMENTATION'],
            'bridge': str(bridge)}


def load_production():
    spec = importlib.util.spec_from_file_location('run_ros_cartpole', PRODUCTION)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command(work, effort, publish, seq=None):
    payload = {'effort': effort, 'publish': publish}
    if seq is not None:
        payload['seq'] = seq
    write_json(Path(work) / 'command.json', payload)


def burst(work, effort, seconds, seq):
    write_json(Path(work) / 'command.json',
               {'effort': effort, 'publish': True, 'burst_seconds': seconds, 'seq': seq})


def start_driver(work):
    env = {'HOME': os.environ['HOME'], 'PATH': '/usr/bin:/bin',
           'ROS_DOMAIN_ID': os.environ['ROS_DOMAIN_ID'],
           'RMW_IMPLEMENTATION': os.environ['RMW_IMPLEMENTATION']}
    argv = ['bash', '--noprofile', '--norc', '-c',
            'source /opt/ros/' + os.environ['ROS_DISTRO'] + '/setup.bash; exec /usr/bin/python3 -B "$1" '
            '--mode passive --work-dir "$2"', 'driver', str(DRIVER), str(work)]
    log = (Path(work) / 'system.log').open('w')
    return subprocess.Popen(argv, env=env, stdout=log, stderr=subprocess.STDOUT), log


def step(runtime, pace=True):
    started = time.monotonic()
    row = runtime.step()
    if pace:
        time.sleep(max(0.0, PHYSICS_DT - (time.monotonic() - started)))
    return row


def step_until(runtime, predicate, limit, pace=True):
    for _ in range(limit):
        row = step(runtime, pace)
        if predicate(row):
            return row
    return None


def inject(runtime, cart, pole):
    """Test-only finite-state placement; nothing here is part of the production runtime."""
    torch = runtime.torch
    position = torch.zeros_like(runtime.base.cartpole.data.joint_pos)
    velocity = torch.zeros_like(position)
    position[0, runtime.cart] = cart
    position[0, runtime.pole] = pole
    runtime.zero_effort()
    runtime.base.cartpole.write_joint_state_to_sim(position, velocity)
    return {'cart_position': cart, 'pole_angle': pole,
            'position': position[0].tolist(), 'velocity': velocity[0].tolist(),
            'reset_effort_target': [0.0] * len(runtime.names)}


def graph_prims(runtime):
    """(AstrEX ROS graph roots, other OmniGraph prims) taken from the live stage."""
    if '_stage' not in _STAGE:
        import omni.usd
        _STAGE['_stage'] = omni.usd.get_context().get_stage()
    ours, others = [], []
    for prim in _STAGE['_stage'].Traverse():
        if prim.GetTypeName() != 'OmniGraph':
            continue
        path = str(prim.GetPath())
        (ours if path.startswith(GRAPH_PREFIX) else others).append(path)
    return sorted(ours), sorted(others)


def node_present(runtime, path, attribute):
    try:
        return runtime.og.Controller.get(path + attribute) is not None
    except Exception:
        return False


def effort_target(runtime):
    return runtime.articulation.get_applied_joint_efforts().detach().cpu().tolist()


def gate_state(runtime):
    return {'command_gate': runtime.command_gate, 'links': runtime.command_acceptance_links()}


def event_slice(work, since):
    events = read_jsonl(Path(work) / 'ros_events.jsonl')
    return events[since:], len(events)


def run_branch(work, runtime, label, reason, placement, effort, seq, window_burst=False):
    record = {'label': label, 'expected_reason': reason, 'placement': placement, 'effort': effort,
              'status': 'FAIL', 'applied_effort': None, 'effort_note': RESET_EFFORT_NOTE}
    events_before = len(read_jsonl(Path(work) / 'ros_events.jsonl'))
    path_before = runtime.path
    creations_before = runtime.graph_creations
    resets_before = runtime.resets
    time_before = runtime.snapshot()['simulation_time']

    command(work, effort, True, seq=seq)
    reached = step_until(runtime, lambda row: row['controller_effort'] == [effort], 600)
    if reached is None:
        record['error'] = 'command never reached the official controller'
        return record
    record['command_reached'] = {'subscriber_effort': reached['subscriber_effort'],
                                 'controller_effort': reached['controller_effort']}
    command(work, 0.0, False, seq=seq + 1)
    step(runtime)
    record['injection'] = inject(runtime, placement['cart_position'], placement['pole_angle'])
    if window_burst:
        record['reset_window_command'] = {'effort': effort, 'burst_seconds': WINDOW_BURST_SECONDS}
        burst(work, effort, WINDOW_BURST_SECONDS, seq + 2)

    row = None
    for index in range(1, BOUNDARY_STEP_LIMIT + 1):
        row = step(runtime)
        if row['boundary']:
            break
    command(work, 0.0, False, seq=seq + 3)
    if row is None or not row['boundary']:
        record['error'] = 'boundary did not trigger within %d steps' % BOUNDARY_STEP_LIMIT
        return record
    record['boundary'] = {'reason': list(row['boundary']), 'physics_steps': index,
                          'previous_position': row['position'], 'previous_velocity': row['velocity']}
    record['reason_exact'] = list(row['boundary']) == [reason]

    events, events_after = event_slice(work, events_before)
    order = [event.get('event') for event in events]
    record['events'] = order
    closed = [event for event in events if event.get('event') == 'command_gate_closed']
    opened = [event for event in events if event.get('event') == 'command_gate_opened']
    boundary_events = [event for event in events if event.get('event') == 'boundary']
    ready_events = [event for event in events if event.get('event') == 'reset_ready']
    failed_events = [event for event in events if event.get('event') == 'reset_failed']
    if not boundary_events or not ready_events or not closed or not opened:
        record['error'] = 'expected gate/boundary/reset_ready events missing: ' + repr(order)
        return record
    record['gate_closed_before_boundary'] = (order.index('command_gate_closed') < order.index('boundary'))
    record['gate_opened_after_verification'] = (order.index('command_gate_opened') < order.index('reset_ready')
                                                and order.index('reset_ready') > order.index('boundary'))
    record['gate_closed_links'] = closed[-1].get('links')
    record['gate_opened_links'] = opened[-1].get('links')
    record['gate_links_ok'] = (
        isinstance(record['gate_closed_links'], list)
        and all('Subscribe.outputs:execOut' not in link for link in record['gate_closed_links'])
        and isinstance(record['gate_opened_links'], list)
        and any(link.endswith('/Subscribe.outputs:execOut') for link in record['gate_opened_links']))
    record['no_reset_failed'] = not failed_events
    ready = ready_events[-1]
    record['reset_ready'] = {key: ready.get(key) for key in
                             ('graph_path', 'graph_creations', 'astrex_graph_count', 'command_gate',
                              'verified', 'checks', 'other_omnigraph_prims')}
    record['hold_seconds'] = ready['wall_monotonic'] - boundary_events[-1]['wall_monotonic']
    record['hold_pass'] = record['hold_seconds'] >= HOLD_SECONDS - 0.02
    record['reset_count_incremented'] = runtime.resets == resets_before + 1
    record['graph_creations_is_one'] = runtime.graph_creations == creations_before == 1
    record['graph_path_constant'] = ready.get('graph_path') == runtime.graph_path == runtime.path == path_before
    ours, others = graph_prims(runtime)
    record['astrex_graph_prims'] = ours
    record['other_omnigraph_prims'] = others
    record['astrex_graph_count_is_one'] = len(ours) == 1 and ours[0] == runtime.graph_path
    record['subscriber_reused'] = node_present(runtime, runtime.path, '/Subscribe.inputs:topicName')
    record['simulation_time_not_rewound'] = ready['state']['simulation_time'] >= time_before - 1e-9
    state = ready['state']
    record['reset_state'] = {'position': state['position'], 'velocity': state['velocity'],
                             'subscriber_effort': state['subscriber_effort'],
                             'controller_effort': state['controller_effort']}
    record['state_zeroed'] = (all(abs(value) < POSITION_GATE for value in state['position'])
                              and all(abs(value) < VELOCITY_GATE for value in state['velocity']))
    record['effort_target_after_reset'] = effort_target(runtime)
    record['effort_target_zero'] = all(abs(value) < EFFORT_TARGET_GATE for value in record['effort_target_after_reset'])
    record['gate_state_after_reset'] = gate_state(runtime)

    idle = []
    idle_targets = []
    for step_index in range(IDLE_STEPS):
        idle.append(step(runtime))
        if step_index % 10 == 0:
            idle_targets.append(max(abs(value) for value in effort_target(runtime)))
    record['idle'] = {'samples': len(idle), 'seconds': round(len(idle) * PHYSICS_DT, 3),
                      'max_abs_position': max(abs(sample['position'][runtime.cart]) for sample in idle),
                      'max_abs_velocity': max(abs(sample['velocity'][runtime.cart]) for sample in idle),
                      'max_abs_effort_target': max(idle_targets) if idle_targets else None,
                      'subscriber_effort_seen': sorted({tuple(sample['subscriber_effort']) for sample in idle})}
    record['no_replay_after_reset'] = (record['idle']['max_abs_position'] < POSITION_GATE
                                       and record['idle']['max_abs_velocity'] < VELOCITY_GATE
                                       and record['idle']['max_abs_effort_target'] < EFFORT_TARGET_GATE)
    live = read_json(Path(work) / 'system_live.json') or {}
    counts = live.get('counts') or {}
    record['dds_counts'] = counts
    record['dds_singleton'] = all((counts.get(key) or 0) == 1 for key in
                                 ('clock_publishers', 'joint_state_publishers', 'command_subscribers'))
    # Timing attribution: commands sent inside the reset window must not be published after reopen.
    publishes = [row for row in read_jsonl(Path(work) / 'commands.jsonl') if 'burst_seconds' not in row]
    gate_closed_time = closed[-1].get('wall_time')
    gate_opened_time = opened[-1].get('wall_time')
    record['publishes_in_reset_window'] = len([row for row in publishes
                                               if gate_closed_time and gate_opened_time
                                               and gate_closed_time <= row['wall'] <= gate_opened_time])
    record['publishes_after_reopen'] = len([row for row in publishes
                                            if gate_opened_time and row['wall'] > gate_opened_time])
    record['no_publish_after_reopen'] = record['publishes_after_reopen'] == 0
    if window_burst:
        record['window_command_exercised'] = record['publishes_in_reset_window'] > 0
    else:
        record['window_command_exercised'] = True

    def recovery(value, sign, label_text):
        command(work, value, True, seq=seq + 10 + int(sign))
        arrival = step_until(runtime, lambda row: row['controller_effort'] == [value], 600)
        if arrival is None:
            return {'pass': False, 'reason': 'command did not reach the controller'}
        start = {'position': arrival['position'][runtime.cart], 'velocity': arrival['velocity'][runtime.cart]}
        rows = [step(runtime) for _ in range(RECOVERY_STEPS)]
        end = rows[-1]
        dp = end['position'][runtime.cart] - start['position']
        dv = end['velocity'][runtime.cart] - start['velocity']
        return {'effort': value, 'label': label_text, 'dp': dp, 'dv': dv, 'steps': RECOVERY_STEPS,
                'boundary_hit': bool(end['boundary']),
                'pass': sign * dp > MOTION_POSITION_GATE and sign * dv > MOTION_VELOCITY_GATE}

    record['same_value_recovery'] = recovery(effort, math.copysign(1, effort), 'same value re-command')
    command(work, 0.0, False, seq=seq + 20)
    for _ in range(SETTLE_STEPS):
        step(runtime)
    record['reverse_recovery'] = recovery(-effort, -math.copysign(1, effort), 'reverse command')
    command(work, 0.0, False, seq=seq + 30)
    step(runtime)

    checks = (record['reason_exact'], record['hold_pass'], record['reset_count_incremented'],
              record['graph_creations_is_one'], record['graph_path_constant'],
              record['astrex_graph_count_is_one'], record['subscriber_reused'],
              record['gate_closed_before_boundary'], record['gate_opened_after_verification'],
              record['gate_links_ok'], record['no_reset_failed'], record['state_zeroed'],
              record['effort_target_zero'], record['no_replay_after_reset'], record['dds_singleton'],
              record['simulation_time_not_rewound'], record['window_command_exercised'],
              record['no_publish_after_reopen'],
              record['same_value_recovery']['pass'], record['reverse_recovery']['pass'])
    record['failed_checks'] = [name for name, value in zip(
        ('reason_exact', 'hold_pass', 'reset_count_incremented', 'graph_creations_is_one',
         'graph_path_constant', 'astrex_graph_count_is_one', 'subscriber_reused',
         'gate_closed_before_boundary', 'gate_opened_after_verification', 'gate_links_ok',
         'no_reset_failed', 'state_zeroed', 'effort_target_zero', 'no_replay_after_reset',
         'dds_singleton', 'simulation_time_not_rewound', 'window_command_exercised',
         'no_publish_after_reopen', 'same_value_recovery', 'reverse_recovery'),
        checks) if not value]
    record['status'] = 'PASS' if not record['failed_checks'] else 'FAIL'
    return record


def run_fault_latch_phase(work, runtime, seq):
    """Fault injection from the test side: a failed reset must latch acceptance CLOSED."""
    record = {'label': 'fault_latch', 'status': 'FAIL', 'applied_effort': None,
              'effort_note': RESET_EFFORT_NOTE}
    events_before = len(read_jsonl(Path(work) / 'ros_events.jsonl'))
    original = runtime.verify_reset_state
    runtime.verify_reset_state = lambda boundary_time: (
        False, {'forced_verification_failure': True}, runtime.snapshot(), [], [])
    try:
        command(work, 5.0, True, seq=seq)
        arrival = step_until(runtime, lambda row: row['controller_effort'] == [5.0], 600)
        if arrival is None:
            record['error'] = 'command never reached the official controller'
            return record
        command(work, 0.0, False, seq=seq + 1)
        step(runtime)
        record['injection'] = inject(runtime, runtime.base.cfg.max_cart_pos + 0.05, 0.0)
        row = None
        for index in range(1, BOUNDARY_STEP_LIMIT + 1):
            row = step(runtime)
            if row['boundary']:
                break
        record['boundary_triggered'] = bool(row and row['boundary'])
        command(work, 0.0, False, seq=seq + 2)
        events, _ = event_slice(work, events_before)
        record['events'] = [event.get('event') for event in events]
        record['reset_failed_recorded'] = any(event.get('event') == 'reset_failed' for event in events)
        record['fault_file'] = (Path(work) / 'fault.json').exists()
        record['reset_ready_absent'] = not any(event.get('event') == 'reset_ready' for event in events)
        record['fault_state'] = runtime.fault
        record['gate_closed'] = (runtime.command_gate == 'closed'
                                 and gate_state(runtime)['links'] == [])
        start_position = runtime.base.cartpole.data.joint_pos[0].tolist()
        command(work, 5.0, True, seq=seq + 3)
        rows, targets = [], []
        for step_index in range(120):
            rows.append(step(runtime))
            if step_index % 10 == 0:
                targets.append(max(abs(value) for value in effort_target(runtime)))
        command(work, 0.0, False, seq=seq + 4)
        record['post_fault'] = {
            'window_steps': len(rows),
            'max_abs_position_delta': max(abs(sample['position'][runtime.cart] - start_position[runtime.cart])
                                          for sample in rows),
            'max_abs_effort_target': max(targets) if targets else None}
        record['commands_ignored'] = (record['post_fault']['max_abs_effort_target'] < EFFORT_TARGET_GATE
                                      and record['post_fault']['max_abs_position_delta'] < 1e-2)
        checks = (record['boundary_triggered'], record['reset_failed_recorded'], record['fault_file'],
                  record['reset_ready_absent'], record['gate_closed'], record['commands_ignored'])
        record['failed_checks'] = [name for name, value in zip(
            ('boundary_triggered', 'reset_failed_recorded', 'fault_file', 'reset_ready_absent',
             'gate_closed', 'commands_ignored'), checks) if not value]
        record['status'] = 'PASS' if not record['failed_checks'] else 'FAIL'
    finally:
        runtime.verify_reset_state = original
    return record


def clock_and_duplicate_summary(work, runtime):
    messages = read_jsonl(Path(work) / 'system_messages.jsonl')
    clocks = [row['stamp'] for row in messages if row.get('kind') == 'clock']
    ours, others = graph_prims(runtime)
    return {'clock_count': len(clocks),
            'clock_strictly_increasing': all(b > a for a, b in zip(clocks, clocks[1:])),
            'astrex_graph_prims': ours, 'other_omnigraph_prims': others,
            'graph_creations': runtime.graph_creations, 'graph_path': runtime.graph_path,
            'reset_count': runtime.resets, 'fault': runtime.fault}


def main():
    parser = argparse.ArgumentParser(description='AstrEX Cartpole boundary/reset acceptance (Isaac side)')
    parser.add_argument('--work-dir', required=True)
    parser.add_argument('--timeout', type=float, default=900.0)
    parser.add_argument('--skip-window-burst', action='store_true',
                        help='attribution only: do not publish a command inside the reset window')
    args = parser.parse_args()
    work = Path(args.work_dir)
    work.mkdir(parents=True, exist_ok=True)
    result = {'status': 'FAIL', 'profile': 'ros_boundary_reset', 'branches': []}
    started = time.monotonic()
    driver = log = runtime = None
    try:
        result['environment'] = prepare_ros_environment()
        os.environ['ASTREX_RUN_DIR'] = str(work)
        module = load_production()
        runtime = module.CartpoleROS(headless=False)
        ours, others = graph_prims(runtime)
        result['startup'] = {'ready': read_json(work / 'ready.json'),
                             'python': sys.version.split()[0], 'prefix': sys.prefix,
                             'cart_limit': runtime.base.cfg.max_cart_pos,
                             'joint_names': list(runtime.names), 'device': str(runtime.base.sim.device),
                             'clone_in_fabric': bool(runtime.base.cfg.scene.clone_in_fabric),
                             'gui': True, 'graph_path': runtime.graph_path,
                             'graph_creations': runtime.graph_creations,
                             'astrex_graph_prims': ours, 'other_omnigraph_prims': others,
                             'command_gate': runtime.command_gate}
        driver, log = start_driver(work)
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline and not (work / 'system_ready.json').exists():
            if driver.poll() is not None:
                raise RuntimeError('external driver exited early; see system.log')
            time.sleep(0.2)
        if not (work / 'system_ready.json').exists():
            raise RuntimeError('external driver did not report ready')
        limit = runtime.base.cfg.max_cart_pos
        branches = (('cart_boundary', 'cart_position',
                     {'cart_position': limit + 0.05, 'pole_angle': 0.0}, 5.0,
                     not args.skip_window_burst),
                    ('pole_boundary_positive', 'pole_angle',
                     {'cart_position': 0.0, 'pole_angle': math.pi / 2 + 0.02}, -5.0, False),
                    ('pole_boundary_negative', 'pole_angle',
                     {'cart_position': 0.0, 'pole_angle': -(math.pi / 2 + 0.02)}, 5.0, False))
        for index, (label, reason, placement, effort, window_burst) in enumerate(branches):
            record = run_branch(work, runtime, label, reason, placement, effort,
                                seq=100 * (index + 1), window_burst=window_burst)
            result['branches'].append(record)
            print('BRANCH_RESULT ' + json.dumps({key: value for key, value in record.items()
                                                 if key not in ('reset_state', 'reset_ready', 'injection')}),
                  flush=True)
            if record['status'] != 'PASS':
                break
        result['fault_latch'] = run_fault_latch_phase(work, runtime, seq=900)
        print('FAULT_LATCH ' + json.dumps({key: value for key, value in result['fault_latch'].items()
                                           if key != 'fault_state'}), flush=True)
        result['summary'] = clock_and_duplicate_summary(work, runtime)
        graph_ok = (len(result['summary']['astrex_graph_prims']) == 1
                    and result['summary']['graph_creations'] == 1
                    and result['summary']['clock_strictly_increasing'])
        branches_ok = (len(result['branches']) == len(branches)
                       and all(record['status'] == 'PASS' for record in result['branches']))
        duplicates = (len(result['summary']['astrex_graph_prims']) != 1
                      or any((record.get('dds_counts') or {}).get(key, 0) > 1 for record in result['branches']
                             for key in ('clock_publishers', 'joint_state_publishers', 'command_subscribers')))
        if duplicates:
            result['status'] = 'BLOCK'
        elif branches_ok and graph_ok and result['fault_latch']['status'] == 'PASS':
            result['status'] = 'PASS'
        else:
            result['status'] = 'FAIL'
    except BaseException as exc:
        import traceback
        result['error'] = traceback.format_exc()
        result['status'] = 'FAIL'
        print('HARNESS_ERROR ' + repr(exc), flush=True)
    finally:
        command(work, 0.0, False, seq=9999)
        if driver is not None and driver.poll() is None:
            (work / 'stop_system').touch()
            try:
                driver.wait(timeout=15)
            except subprocess.TimeoutExpired:
                driver.terminate()
                driver.wait(timeout=15)
        if log is not None:
            log.close()
        result['wall_seconds'] = round(time.monotonic() - started, 3)
        result['repeated_reset_graph_lifecycle'] = 'PASS' if result['status'] == 'PASS' else result['status']
        result['fault_latch_status'] = result.get('fault_latch', {}).get('status', 'NOT RUN')
        write_json(work / 'boundary_reset_result.json', result)
        print('BOUNDARY_RESET_RESULT ' + json.dumps({key: value for key, value in result.items()
                                                    if key not in ('branches', 'fault_latch')}), flush=True)
        # Kit teardown can hang; the evidence is already on disk before we attempt it.
        if runtime is not None:
            runtime.close()
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
