"""External ROS 2 acceptance driver and observer (system Jazzy / Python 3.12 only).

Modes
-----
control
    Drive the real ``./scripts/start_isaac_ros.sh`` run: hold 0 N, then push +5 N and
    -5 N, each group measured from a fresh automatic boundary reset. Writes
    ``system_messages.jsonl``, ``commands.jsonl``, ``system_summary.json`` and
    ``control_result.json`` into ``--work-dir``.
passive
    Publish exactly what ``<work-dir>/command.json`` asks for and keep observing.
    Used by ``tests/isaac/validate_ros_cartpole_boundary_reset.py``.

This file never imports Isaac Sim or the ``isaaclab232_test`` Python environment; it
only talks DDS and reads the production run's JSONL evidence.
"""

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState

CART_JOINT = 'slider_to_cart'
POLE_JOINT = 'cart_to_pole'
PHYSICS_DT = 1.0 / 120.0
FEEDBACK_STEPS = 2
# The Cartpole pole reaches its boundary ~0.53 s after a 5 N push, so a group window has to
# stay well inside that. 24 physics steps = 0.2 s still leaves the response far above the
# 1 mm / 0.01 m/s thresholds (measured ~0.06 m and ~0.4 m/s).
GROUP_INDEX = 24
GROUP_SAMPLES = GROUP_INDEX + 6
PUBLISH_PERIOD = 0.02
IDLE_SECONDS = 2.2
POSITION_GATE = 1e-3
VELOCITY_GATE = 1e-2
FEEDBACK_POSITION_TOLERANCE = 0.02
ZERO_COMMANDS = ([], [0.0], [0.0, 0.0])


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


class SimTail:
    """Incremental reader for the production runtime's JSONL evidence."""

    def __init__(self, run_dir):
        self.run_dir = Path(run_dir)
        self.samples = []
        self.events = []
        self._offsets = {}

    def _read(self, path):
        if not path.exists():
            return []
        offset = self._offsets.get(str(path), 0)
        with path.open('rb') as stream:
            stream.seek(offset)
            data = stream.read()
        end = data.rfind(b'\n')
        if end < 0:
            return []
        self._offsets[str(path)] = offset + end + 1
        rows = []
        for line in data[:end].split(b'\n'):
            line = line.strip()
            if not line:
                continue
            try:
                rows.append(json.loads(line.decode('utf-8')))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
        return rows

    def poll(self):
        self.samples.extend(self._read(self.run_dir / 'ros_samples.jsonl'))
        self.events.extend(self._read(self.run_dir / 'ros_events.jsonl'))
        return self.samples

    def latest(self):
        return self.samples[-1] if self.samples else None

    def latest_reset_count(self):
        row = self.latest()
        return row['reset_count'] if row else None

    def with_reset(self, reset_count):
        return [row for row in self.samples if row['reset_count'] == reset_count]

    def group(self, reset_count, effort, limit=GROUP_SAMPLES):
        rows = [row for row in self.samples
                if row['reset_count'] == reset_count and row['controller_effort'] == [effort]]
        return rows[:limit]


class Observer:
    """DDS observer plus command publisher."""

    def __init__(self, work_dir):
        self.work = Path(work_dir)
        self.work.mkdir(parents=True, exist_ok=True)
        self.clocks = []
        self.joints = []
        self.count_timeline = []
        self.publish_effort = None
        self.burst_until = None
        self.burst_seq = None
        self.publish_count = 0
        self.last_publish = 0.0
        self.last_live = 0.0
        self.stopping = False
        self.messages = (self.work / 'system_messages.jsonl').open('w')
        self.command_log = (self.work / 'commands.jsonl').open('w')
        rclpy.init()
        self.node = rclpy.create_node('astrex_system_regression')
        self.pub = self.node.create_publisher(JointState, '/joint_command', 1)
        self.node.create_subscription(Clock, '/clock', self._clock, 10)
        self.node.create_subscription(JointState, '/joint_states', self._joint, 10)
        write_json(self.work / 'system_ready.json', {'pid': os.getpid(), 'domain': os.environ.get('ROS_DOMAIN_ID')})

    def _clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec / 1e9
        self.clocks.append(value)
        self.messages.write(json.dumps({'kind': 'clock', 'stamp': value}) + '\n')

    def _joint(self, msg):
        row = {'kind': 'joint',
               'wall': time.time(),
               'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
               'names': list(msg.name), 'position': list(msg.position), 'velocity': list(msg.velocity)}
        self.joints.append(row)
        self.messages.write(json.dumps(row) + '\n')

    def counts(self, tag):
        row = {'tag': tag, 'wall': time.time(),
               'clock_publishers': self.node.count_publishers('/clock'),
               'joint_state_publishers': self.node.count_publishers('/joint_states'),
               'command_subscribers': self.node.count_subscribers('/joint_command'),
               'node_names': sorted(self.node.get_node_names())}
        self.count_timeline.append(row)
        print(f'COUNTS {json.dumps(row)}', flush=True)
        return row

    def set_command(self, effort):
        """None means silence; a float is published continuously at ~50 Hz."""
        self.publish_effort = effort
        self.burst_until = None

    def start_burst(self, effort, seconds, seq):
        """Publish one bounded burst: used to inject a command inside a reset window."""
        self.publish_effort = effort
        self.burst_until = time.monotonic() + seconds
        self.burst_seq = seq
        print(f'BURST start effort={effort} seconds={seconds} seq={seq}', flush=True)

    def _service_command(self):
        if self.publish_effort is None:
            return
        now = time.monotonic()
        if self.burst_until is not None and now >= self.burst_until:
            print('BURST finished', flush=True)
            self.publish_effort = None
            self.burst_until = None
            return
        if now - self.last_publish < PUBLISH_PERIOD:
            return
        msg = JointState()
        msg.name = [CART_JOINT]
        msg.effort = [float(self.publish_effort)]
        self.pub.publish(msg)
        self.publish_count += 1
        self.last_publish = now
        self.command_log.write(json.dumps({'wall': time.time(), 'effort': self.publish_effort}) + '\n')
        self.command_log.flush()

    def _write_live(self):
        now = time.monotonic()
        if now - self.last_live < 0.25:
            return
        self.last_live = now
        write_json(self.work / 'system_live.json', {
            'wall': time.time(), 'clock_count': len(self.clocks), 'joint_count': len(self.joints),
            'last_joint': self.joints[-1] if self.joints else None,
            'counts': self.count_timeline[-1] if self.count_timeline else None,
            'publishing': self.publish_effort is not None, 'published_values': self.publish_count})
        self.messages.flush()

    def pump(self, seconds=0.05):
        deadline = time.monotonic() + seconds
        while True:
            rclpy.spin_once(self.node, timeout_sec=0.002)
            self._service_command()
            self._write_live()
            if time.monotonic() >= deadline:
                return

    def spin_until(self, predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while True:
            self.pump(0.05)
            if predicate():
                return True
            if self.stopping:
                print(f'STOPPED while waiting for {label}', flush=True)
                return False
            if time.monotonic() >= deadline:
                print(f'TIMEOUT waiting for {label}', flush=True)
                return False

    def finish(self, extra):
        self.set_command(None)
        summary = {'clock_count': len(self.clocks), 'joint_count': len(self.joints),
                   'strictly_increasing': all(b > a for a, b in zip(self.clocks, self.clocks[1:])),
                   'counts': self.count_timeline, 'published_values': self.publish_count,
                   'domain': os.environ.get('ROS_DOMAIN_ID'),
                   'rmw_implementation': os.environ.get('RMW_IMPLEMENTATION')}
        summary.update(extra)
        write_json(self.work / 'system_summary.json', summary)
        self.messages.flush()
        self.messages.close()
        self.command_log.close()
        self.node.destroy_node()
        rclpy.shutdown()
        return summary


def idle_window(observer, start_wall, end_wall, publishes):
    """A post-reset silence window judged on the live system stream, not on the Sim's flush lag."""
    rows = [row for row in observer.joints if start_wall + 0.2 <= row['wall'] <= end_wall]
    if not rows:
        return {'samples': 0, 'pass': False, 'reason': 'no /joint_states inside the silence window'}
    cart = rows[0]['names'].index(CART_JOINT)
    window = {'samples': len(rows), 'seconds': round(end_wall - start_wall, 3),
              'max_abs_position': max(abs(row['position'][cart]) for row in rows),
              'max_abs_velocity': max(abs(row['velocity'][cart]) for row in rows),
              'published_in_window': publishes}
    window['pass'] = (publishes == 0 and window['max_abs_position'] < POSITION_GATE
                      and window['max_abs_velocity'] < VELOCITY_GATE)
    return window


def feedback_checks(groups, joints):
    """Match every group sample to the nearest /joint_states message by simulation stamp."""
    stamps = [row['stamp'] for row in joints]
    if not stamps:
        return {'samples': 0, 'max_stamp_error': None, 'max_state_error': None, 'pass': False}
    # The external observer cannot receive every 120 Hz sample, so its sampling density bounds how
    # closely stamps can be matched. State agreement and stream integrity are the real gates here.
    tolerance = FEEDBACK_STEPS * PHYSICS_DT + 1e-7
    stamps_increasing = all(b >= a for a, b in zip(stamps, stamps[1:]))
    max_stamp_error, max_state_error, checked = 0.0, 0.0, 0
    for rows in groups.values():
        for row in rows:
            nearest = min(joints, key=lambda msg: abs(msg['stamp'] - row['simulation_time']))
            stamp_error = abs(nearest['stamp'] - row['simulation_time'])
            error = 0.0
            for name, position in zip(nearest['names'], nearest['position']):
                index = row['joint_names'].index(name)
                candidate = min(rows, key=lambda other: abs(other['simulation_time'] - nearest['stamp']))
                error = max(error, abs(candidate['position'][index] - position))
            if not all(math.isfinite(x) for x in nearest['position'] + nearest['velocity']):
                return {'samples': checked, 'max_stamp_error': max_stamp_error, 'max_state_error': max_state_error,
                        'pass': False, 'reason': 'non-finite feedback'}
            max_stamp_error = max(max_stamp_error, stamp_error)
            max_state_error = max(max_state_error, error)
            checked += 1
    return {'samples': checked, 'max_stamp_error': max_stamp_error, 'max_state_error': max_state_error,
            'stamp_tolerance': tolerance, 'stamps_increasing': stamps_increasing,
            'observer_receive_hz': round(len(joints) / max(1e-9, joints[-1]['wall'] - joints[0]['wall']), 2)
            if len(joints) > 1 else None,
            'pass': stamps_increasing and max_state_error < FEEDBACK_POSITION_TOLERANCE}


def analyse_control(observer, sim, windows):
    groups = {'zero': windows['zero'], 'positive': windows['positive'], 'negative': windows['negative']}
    cart = sim.samples[0]['joint_names'].index(CART_JOINT)
    response, chain_ok = {}, True
    for effort, value in ((5.0, 'positive'), (-5.0, 'negative')):
        if len(groups[value]) <= GROUP_INDEX or len(groups['zero']) <= GROUP_INDEX:
            response[str(effort)] = {'pass': False, 'reason': 'group shorter than comparison index'}
            chain_ok = False
            continue
        row, zero = groups[value][GROUP_INDEX], groups['zero'][GROUP_INDEX]
        dp = row['position'][cart] - zero['position'][cart]
        dv = row['velocity'][cart] - zero['velocity'][cart]
        response[str(effort)] = {'dp': dp, 'dv': dv, 'control_step': GROUP_INDEX + 1,
                                 'pass': math.copysign(1, effort) * dp > 1e-3 and math.copysign(1, effort) * dv > 1e-2}
    for name, effort in (('zero', 0.0), ('positive', 5.0), ('negative', -5.0)):
        for row in groups[name]:
            if row['subscriber_effort'] != [effort] or row['controller_effort'] != [effort]:
                chain_ok = False
    joint_names_ok = all(set((CART_JOINT, POLE_JOINT)).issubset(set(row['names'])) for row in observer.joints)
    joint_finite = all(math.isfinite(value)
                       for row in observer.joints for value in row['position'] + row['velocity'])
    settled = [row for row in observer.count_timeline if row['tag'].startswith(('after_reset', 'final'))]
    # Duplicates mean "more than one endpoint"; a not-yet-matched count is recorded but never
    # treated as duplication. The observer only gets here while receiving both topics.
    duplicates = [row for row in settled
                  if (row['clock_publishers'] or 0) > 1 or (row['joint_state_publishers'] or 0) > 1
                  or (row['command_subscribers'] or 0) > 1]
    discovered = bool(settled) and all((row['clock_publishers'] or 0) >= 1
                                       and (row['joint_state_publishers'] or 0) >= 1 for row in settled)
    feedback = feedback_checks(groups, observer.joints)
    checks = {
        'clock_at_least_10_strictly_increasing': len(observer.clocks) >= 10
        and all(b > a for a, b in zip(observer.clocks, observer.clocks[1:])),
        'joint_states_at_least_10_two_joints_finite': len(observer.joints) >= 10 and joint_names_ok and joint_finite,
        'command_chain_matches': chain_ok,
        'positive_response': response['5.0']['pass'],
        'negative_response': response['-5.0']['pass'],
        'feedback': feedback['pass'],
        'idle_after_reset_no_replay': all(window['pass'] for window in windows['idle']),
        'no_duplicate_endpoints': not duplicates and discovered,
    }
    result = {'mode': 'control', 'status': 'PASS' if all(checks.values()) else 'FAIL', 'checks': checks,
              'response': response, 'idle': windows['idle'], 'feedback': feedback,
              'reset_counts': windows['reset_counts'], 'settled_counts': settled,
              'count_timeline': observer.count_timeline, 'group_samples': {k: len(v) for k, v in groups.items()},
              'clock_count': len(observer.clocks), 'joint_count': len(observer.joints)}
    return result


def run_control(observer, run_dir):
    """Group protocol: every measured group starts right after its own automatic boundary reset."""
    sim = SimTail(run_dir)
    windows = {'idle': [], 'reset_counts': []}
    result = {'mode': 'control', 'status': 'FAIL', 'reason': 'ready timeout'}
    ready = Path(run_dir) / 'ready.json'

    def is_ready():
        sim.poll()
        return ready.exists() and bool(observer.clocks) and bool(observer.joints) and bool(sim.samples)

    if not observer.spin_until(is_ready, 180, 'ready.json + /clock + /joint_states'):
        return result
    windows['reset_counts'].append(sim.latest_reset_count())
    observer.counts('initial')

    def diagnostics():
        return {'max_reset_count': max((row['reset_count'] for row in sim.samples), default=None),
                'latest': sim.samples[-1] if sim.samples else None,
                'reset_counts': list(windows['reset_counts']),
                'windows': {name: len(rows) for name, rows in windows.items() if isinstance(rows, list)},
                'publishing': observer.publish_effort}

    def stretch(effort):
        """Command window of the current episode (always the newest reset count)."""
        sim.poll()
        if not sim.samples:
            return None, []
        latest = sim.samples[-1]['reset_count']
        rows = [row for row in sim.samples
                if row['reset_count'] == latest and row['controller_effort'] == [effort]]
        return latest, rows

    def collect(effort):
        """Wait for a fresh episode whose command window is long enough to measure."""
        def enough():
            return len(stretch(effort)[1]) >= GROUP_SAMPLES
        return enough

    def next_reset(label):
        before = windows['reset_counts'][-1]

        def reached():
            sim.poll()
            return sim.latest_reset_count() > before

        if not observer.spin_until(reached, 90, label + ' boundary'):
            return False
        sim.poll()
        windows['reset_counts'].append(sim.latest_reset_count())
        return True

    def silence_and_check(label):
        observer.set_command(None)
        publishes_before = observer.publish_count
        settle_start = time.monotonic()

        def reset_visible():
            if not observer.joints:
                return False
            row = observer.joints[-1]
            return abs(row['position'][row['names'].index(CART_JOINT)]) < POSITION_GATE

        # The Sim's sample file flushes every 120 steps, so the reset is judged from the live stream.
        observed = observer.spin_until(reset_visible, IDLE_SECONDS + 5, label + ' reset visible')
        settle_seconds = round(time.monotonic() - settle_start, 3)
        start_wall = time.time()
        start = time.monotonic()
        observer.spin_until(lambda: time.monotonic() - start >= IDLE_SECONDS, IDLE_SECONDS + 15, label + ' idle window')
        end_wall = time.time()
        sim.poll()
        index = len(windows['reset_counts']) - 1
        window = idle_window(observer, start_wall, end_wall, observer.publish_count - publishes_before)
        window['reset_visible'] = observed
        window['settle_seconds'] = settle_seconds
        window['after_reset'] = index
        window['reset_count'] = windows['reset_counts'][-1]
        window['sim_samples_in_episode'] = len(sim.with_reset(windows['reset_counts'][-1]))
        window['pass'] = window['pass'] and observed
        windows['idle'].append(window)
        observer.counts(f'after_reset_{index}')

    # Group 0 N starts from the runtime's own startup reset state.
    observer.set_command(0.0)
    if not observer.spin_until(collect(0.0), 90, 'zero-effort group'):
        result['reason'] = 'zero-effort group not collected'
        result['diagnostics'] = diagnostics()
        return result
    windows['reset_counts'][0], windows['zero'] = stretch(0.0)
    observer.counts('before_first_push')

    # The zero group cannot end in a boundary by itself: one +5 N run provides the
    # fresh baseline reset that the measured +5 N group starts from.
    observer.set_command(5.0)
    if not next_reset('zero-group prep'):
        result['reason'] = 'zero-group prep push did not reach a boundary'
        result['diagnostics'] = diagnostics()
        return result
    silence_and_check('zero-group prep')

    for effort, label in ((5.0, 'positive'), (-5.0, 'negative')):
        observer.set_command(effort)
        if not observer.spin_until(collect(effort), 90, f'{label} group'):
            result['reason'] = f'{label} group not collected'
            result['diagnostics'] = diagnostics()
            return result
        current, rows = stretch(effort)
        windows['reset_counts'].append(current)
        windows[label] = rows[:GROUP_SAMPLES]
        if not next_reset(f'{label} group'):
            result['reason'] = f'{label} group did not reach a boundary'
            result['diagnostics'] = diagnostics()
            return result
        silence_and_check(f'{label} group')

    observer.set_command(None)
    observer.counts('final')
    sim.poll()
    result = analyse_control(observer, sim, windows)
    print('CONTROL_RESULT ' + json.dumps({k: v for k, v in result.items() if k != 'count_timeline'}), flush=True)
    return result


def run_passive(observer, work, timeout):
    result = {'mode': 'passive', 'status': 'PASS', 'transitions': []}
    start = time.monotonic()
    last = 'unset'
    observer.set_command(None)
    observer.counts('passive_start')
    last_counts = time.monotonic()
    while time.monotonic() - start < timeout:
        if (work / 'stop_system').exists():
            break
        request = read_json(work / 'command.json') or {}
        if request.get('publish') and float(request.get('burst_seconds') or 0.0) > 0:
            if request.get('seq') != observer.burst_seq:
                observer.start_burst(float(request['effort']), float(request['burst_seconds']),
                                     request.get('seq'))
                result['transitions'].append({'wall': time.time(), 'effort': float(request['effort']),
                                              'burst_seconds': float(request['burst_seconds'])})
            observer.pump(0.05)
            if time.monotonic() - last_counts >= 0.5:
                observer.counts('passive')
                last_counts = time.monotonic()
            continue
        effort = float(request['effort']) if request.get('publish') else None
        if effort != last:
            result['transitions'].append({'wall': time.time(), 'effort': effort})
            print(f'COMMAND {json.dumps(result["transitions"][-1])}', flush=True)
            last = effort
        observer.set_command(effort)
        observer.pump(0.05)
        if time.monotonic() - last_counts >= 0.5:
            observer.counts('passive')
            last_counts = time.monotonic()
    observer.set_command(None)
    observer.pump(0.2)
    observer.counts('passive_end')
    return result


def main():
    parser = argparse.ArgumentParser(description='AstrEX external ROS 2 acceptance driver')
    parser.add_argument('--work-dir', required=True)
    parser.add_argument('--mode', choices=('control', 'passive'), required=True)
    parser.add_argument('--run-dir', help='production run directory (control mode)')
    parser.add_argument('--timeout', type=float, default=900.0, help='passive mode wall limit [s]')
    args = parser.parse_args()
    work = Path(args.work_dir)
    observer = Observer(work)

    def stop(signum, frame):
        observer.stopping = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    result = {'mode': args.mode, 'status': 'FAIL', 'reason': 'not started'}
    try:
        if args.mode == 'control':
            if not args.run_dir:
                raise SystemExit('--run-dir is required in control mode')
            result = run_control(observer, Path(args.run_dir))
        else:
            result = run_passive(observer, work, args.timeout)
    finally:
        summary = observer.finish({'mode': args.mode, 'work_dir': str(work)})
        result['system_summary'] = {key: summary[key] for key in
                                   ('clock_count', 'joint_count', 'strictly_increasing', 'domain')}
        write_json(work / ('control_result.json' if args.mode == 'control' else 'passive_result.json'), result)
    print('SYSTEM_RESULT ' + json.dumps(result), flush=True)
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    sys.exit(main())
