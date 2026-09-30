#!/usr/bin/env python3
"""Assess one Cartpole B trial from ROS and Isaac logs, without starting either.

Input directory: config.json, trace.jsonl, result.json. ``isaac_run_dir`` in
config.json contains ros_samples.jsonl and ros_events.jsonl. The assessment is
written to assessment.json and printed as JSON. The four B tolerances are
locked here; a config may tighten them but cannot relax them.
"""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path


FORCE_LIMIT_N = 5.0
MAX_SIM_SEC = 10.0
MAX_SATURATION_SIM_SEC = 0.5
MIN_STABLE_SIM_SEC = 1.0
MAX_STATE_GAP_SIM_SEC = 0.05
MAX_STATE_AGE_LOCAL_SEC = 0.2
ABORT_ANGLE_RAD = math.radians(10.0)
ABORT_POSITION_ERROR_M = 0.25
LIMITS = {
    'stable_position_m': 0.05,
    'stable_velocity_mps': 0.10,
    'stable_angle_rad': math.radians(2.0),
    'stable_angular_velocity_radps': 0.20,
}
FAULT_EVENTS = {'boundary', 'test_reset', 'reset_ready', 'reset_failed',
                'command_gate_closed'}
ELIGIBLE_END_REASONS = {'SIM_WINDOW_END', 'STABLE_WINDOW', 'SUCCESS'}
EPS = 1e-9


def _jsonl(path):
    rows = []
    with path.open(encoding='utf-8') as stream:
        for number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f'{path}:{number}: invalid JSON') from exc
            if not isinstance(row, dict):
                raise ValueError(f'{path}:{number}: expected JSON object')
            rows.append(row)
    return rows


def _finite(row, key):
    value = row[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{key} is not numeric')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{key} is not finite')
    return value


def _ns(row, key):
    value = row[key]
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{key} must be a nonnegative integer nanosecond stamp')
    return value


def _state(row):
    return {key: _finite(row, key) for key in ('x', 'x_dot', 'theta', 'theta_dot')}


def _read_config(path):
    config = json.loads(path.read_text(encoding='utf-8'))
    for key in ('x0', 'theta0', 'hold_position', 'force_limit_n', 'max_local_sec'):
        _finite(config, key)
    if abs(config['force_limit_n'] - FORCE_LIMIT_N) > EPS:
        raise ValueError('B force_limit_n must be exactly 5 N')
    max_sim = _finite(config, 'max_sim_sec') if 'max_sim_sec' in config else MAX_SIM_SEC
    if max_sim <= 0 or max_sim > MAX_SIM_SEC + EPS:
        raise ValueError('max_sim_sec must be in (0, 10]')
    if config['max_local_sec'] <= 0:
        raise ValueError('max_local_sec must be positive')
    for key, ceiling in (('abort_angle_rad', ABORT_ANGLE_RAD),
                         ('abort_position_error_m', ABORT_POSITION_ERROR_M),
                         ('state_timeout_local_sec', MAX_STATE_AGE_LOCAL_SEC)):
        if key in config and (_finite(config, key) <= 0 or _finite(config, key) > ceiling + EPS):
            raise ValueError(f'{key} exceeds the fixed B exit limit')
    limits = {}
    for key, ceiling in LIMITS.items():
        value = _finite(config, key) if key in config else ceiling
        if value <= 0 or value > ceiling + EPS:
            raise ValueError(f'{key} exceeds the fixed B tolerance')
        limits[key] = value
    for key, ceiling in (('stable_window_sim_sec', MIN_STABLE_SIM_SEC),
                         ('max_state_gap_sim_sec', MAX_STATE_GAP_SIM_SEC),
                         ('max_state_age_local_sec', MAX_STATE_AGE_LOCAL_SEC),
                         ('max_saturation_sim_sec', MAX_SATURATION_SIM_SEC)):
        value = _finite(config, key) if key in config else ceiling
        if key == 'stable_window_sim_sec':
            if value < ceiling - EPS:
                raise ValueError('stable_window_sim_sec cannot be shorter than 1 s')
        elif value <= 0 or value > ceiling + EPS:
            raise ValueError(f'{key} exceeds the fixed B limit')
        limits[key] = value
    if 'isaac_run_dir' not in config:
        raise ValueError('config.json needs isaac_run_dir')
    return config, limits, max_sim


def _stable(state, config, limits):
    if abs(state['x'] - config['hold_position']) > limits['stable_position_m'] + EPS:
        return False
    if abs(state['x_dot']) > limits['stable_velocity_mps'] + EPS:
        return False
    if abs(state['theta']) > limits['stable_angle_rad'] + EPS:
        return False
    if abs(state['theta_dot']) > limits['stable_angular_velocity_radps'] + EPS:
        return False
    if abs(config['theta0']) > EPS and not abs(state['theta']) < 0.5 * abs(config['theta0']):
        return False
    initial_error = abs(config['x0'] - config['hold_position'])
    if initial_error >= 0.05 - EPS and not abs(state['x'] - config['hold_position']) < 0.025:
        return False
    return True


def _stability(feedback, config, limits):
    first_stamp = feedback[0][0]
    streak_start = None
    previous_stamp = None
    for stamp, state in feedback:
        if previous_stamp is not None and (stamp - previous_stamp) / 1e9 > limits['max_state_gap_sim_sec'] + EPS:
            streak_start = None
        if _stable(state, config, limits):
            if streak_start is None:
                streak_start = stamp
            duration = (stamp - streak_start) / 1e9
            if duration + EPS >= limits['stable_window_sim_sec']:
                return {
                    'start_sim_sec': streak_start / 1e9,
                    'end_sim_sec': stamp / 1e9,
                    'duration_sim_sec': duration,
                }, (streak_start - first_stamp) / 1e9
        else:
            streak_start = None
        previous_stamp = stamp
    return None, None


def _saturation(commands, final_stamp, limit):
    total = 0.0
    longest = 0.0
    streak_start = None
    previous = None
    for stamp, raw, _ in commands:
        if previous is not None:
            gap = (stamp - previous[0]) / 1e9
            if previous[1] and gap > 0:
                total += gap
            if gap > MAX_STATE_GAP_SIM_SEC + EPS:
                streak_start = None
        saturated = abs(raw) > limit + EPS
        if saturated and streak_start is None:
            streak_start = stamp
        if not saturated and streak_start is not None:
            longest = max(longest, (stamp - streak_start) / 1e9)
            streak_start = None
        if saturated and streak_start is not None:
            longest = max(longest, (stamp - streak_start) / 1e9)
        previous = (stamp, saturated)
    if previous is not None and previous[1] and final_stamp >= previous[0]:
        tail = (final_stamp - previous[0]) / 1e9
        total += tail
        if streak_start is not None:
            longest = max(longest, (final_stamp - streak_start) / 1e9)
    return total, longest


def _sim_time(row):
    state = row.get('state', row)
    return _finite(state, 'simulation_time')


def assess_trial(trial_dir):
    """Return an evidence-based assessment; malformed required input raises ValueError."""
    trial_dir = Path(trial_dir)
    config, limits, max_sim = _read_config(trial_dir / 'config.json')
    trace = _jsonl(trial_dir / 'trace.jsonl')
    result = json.loads((trial_dir / 'result.json').read_text(encoding='utf-8'))
    isaac_dir = Path(config['isaac_run_dir'])
    if not isaac_dir.is_absolute():
        isaac_dir = trial_dir / isaac_dir
    samples = _jsonl(isaac_dir / 'ros_samples.jsonl')
    events = _jsonl(isaac_dir / 'ros_events.jsonl')
    issues = []

    init_rows = [i for i, row in enumerate(trace) if row.get('event') == 'initial_feedback']
    exit_rows = [i for i, row in enumerate(trace) if row.get('event') == 'exit']
    if len(init_rows) != 1 or len(exit_rows) != 1 or exit_rows[0] <= init_rows[0]:
        raise ValueError('trace needs exactly one initial_feedback followed by one exit')
    initial = trace[init_rows[0]]
    exit_row = trace[exit_rows[0]]
    initial_stamp = _ns(initial, 'stamp_ns')
    final_state = exit_row.get('final_state', {})
    final_stamp = _ns(exit_row, 'stamp_ns') if 'stamp_ns' in exit_row else (
        _ns(final_state, 'stamp_ns') if 'stamp_ns' in final_state else None)
    first_local = _ns(initial, 'received_monotonic_ns')
    exit_local_key = 'local_monotonic_ns' if 'local_monotonic_ns' in exit_row else 'received_monotonic_ns'
    exit_local = _ns(exit_row, exit_local_key)
    initial_state = _state(initial)
    ready_rows = [row for row in trace[:init_rows[0]] if row.get('event') == 'controller_ready']
    init_trace_rows = [row for row in trace[:init_rows[0]] if row.get('event') == 'trial_initialized']
    if len(ready_rows) != 1 or len(init_trace_rows) != 1:
        issues.append('missing_or_duplicate_controller_ready_or_trial_initialized')
    elif (exit_local - _ns(ready_rows[0], 'local_monotonic_ns')) / 1e9 > config['max_local_sec'] + EPS:
        issues.append('local_execution_timeout')
    if abs(initial_state['x'] - config['x0']) > 0.005:
        issues.append('initial_feedback_x_differs_from_requested_x0')
    if abs(initial_state['theta'] - config['theta0']) > math.radians(0.2):
        issues.append('initial_feedback_theta_differs_from_requested_theta0')
    if abs(initial_state['x_dot']) > 0.02 or abs(initial_state['theta_dot']) > 0.05:
        issues.append('initial_feedback_velocity_not_zero')
    if exit_local < first_local or (exit_local - first_local) / 1e9 > config['max_local_sec'] + EPS:
        issues.append('local_execution_timeout')

    feedback = []
    commands = []  # Only phase=control has a state stamp and raw force.
    published_commands = []  # Includes unstamped stop/final_zero rows.
    previous_stamp = -1
    previous_local = -1
    for row in trace[init_rows[0]:exit_rows[0]]:
        kind = row.get('event')
        if kind in ('initial_feedback', 'feedback'):
            try:
                stamp = _ns(row, 'stamp_ns')
                local = _ns(row, 'received_monotonic_ns')
                state = _state(row)
            except (KeyError, ValueError) as exc:
                issues.append(f'invalid_feedback:{exc}')
                continue
            if stamp <= previous_stamp or local < previous_local:
                issues.append('duplicate_or_out_of_order_feedback')
            if previous_local >= 0 and (local - previous_local) / 1e9 > limits['max_state_age_local_sec'] + EPS:
                issues.append('local_state_update_timeout')
            if abs(state['theta']) > ABORT_ANGLE_RAD + EPS:
                issues.append('ros_pole_angle_boundary')
            if abs(state['x'] - config['hold_position']) > ABORT_POSITION_ERROR_M + EPS:
                issues.append('ros_cart_position_boundary')
            if stamp > previous_stamp:
                feedback.append((stamp, state))
            previous_stamp, previous_local = stamp, local
        elif kind == 'command':
            try:
                cmd = _finite(row, 'f_cmd')
                phase = row['phase']
                if abs(_finite(row, 'hold_position') - config['hold_position']) > EPS:
                    issues.append('command_hold_position_mismatch')
                if 'pole_torque' in row and abs(_finite(row, 'pole_torque')) > EPS:
                    issues.append('nonzero_pole_torque_command')
                published_commands.append(cmd)
                if abs(cmd) > FORCE_LIMIT_N + EPS:
                    issues.append('published_force_over_5_n')
                if phase == 'control':
                    stamp = _ns(row, 'stamp_ns')
                    raw = _finite(row, 'f_raw')
                    commands.append((stamp, raw, cmd))
                    if abs(cmd - max(-FORCE_LIMIT_N, min(FORCE_LIMIT_N, raw))) > 1e-6:
                        issues.append('published_force_not_clipped_raw')
                elif phase not in ('stop', 'final_zero') or abs(cmd) > EPS:
                    issues.append('invalid_exit_zero_command')
            except (KeyError, ValueError) as exc:
                issues.append(f'invalid_command:{exc}')
        elif kind not in ('initial_feedback', 'feedback'):
            issues.append(f'unexpected_trace_event:{kind}')
    if not feedback:
        raise ValueError('trial has no accepted feedback')
    if not published_commands:
        issues.append('no_published_command')
    if not commands:
        issues.append('no_closed_loop_control_commands')
    if final_stamp is None:
        final_stamp = feedback[-1][0]
        issues.append('exit_missing_stamp_ns')
    if final_stamp < feedback[-1][0] or final_stamp < initial_stamp:
        issues.append('exit_stamp_precedes_feedback')
    if (exit_local - previous_local) / 1e9 > limits['max_state_age_local_sec'] + EPS:
        issues.append('local_state_update_timeout')
    if (final_stamp - initial_stamp) / 1e9 > max_sim + EPS:
        issues.append('simulation_execution_timeout')
    if any(commands[i][0] < commands[i - 1][0] or
           (commands[i][0] - commands[i - 1][0]) / 1e9 > MAX_STATE_GAP_SIM_SEC + EPS
           for i in range(1, len(commands))):
        issues.append('command_sim_time_gap_or_disorder')
    max_published = max((abs(cmd) for cmd in published_commands), default=0.0)
    saturation_total, saturation_max = _saturation(commands, final_stamp, FORCE_LIMIT_N)
    if saturation_max > limits['max_saturation_sim_sec'] + EPS:
        issues.append('continuous_saturation_over_0_5_sim_sec')
    if not published_commands or abs(published_commands[-1]) > EPS:
        issues.append('missing_zero_command_at_exit')
    if not any(row.get('event') == 'command' and row.get('phase') == 'stop'
               for row in trace[init_rows[0]:exit_rows[0]]):
        issues.append('missing_explicit_stop_zero')
    for row in trace[exit_rows[0] + 1:]:
        if row.get('event') == 'command':
            try:
                post_cmd = _finite(row, 'f_cmd')
                max_published = max(max_published, abs(post_cmd))
                if abs(post_cmd) > EPS:
                    issues.append('nonzero_command_after_exit')
            except (KeyError, ValueError):
                issues.append('invalid_command_after_exit')

    trial_id = isaac_dir.name
    init_events = [(i, event) for i, event in enumerate(events)
                   if event.get('event') == 'trial_initialized' and event.get('trial_id') == trial_id]
    if len(init_events) != 1:
        raise ValueError(f'expected one Isaac trial_initialized event for {trial_id}')
    init_index, init_event = init_events[0]
    init_sim = _sim_time(init_event)
    requested = init_event.get('requested', {})
    if requested and any(abs(_finite(requested, key) - config[key]) > EPS
                         for key in ('x0', 'theta0', 'hold_position')):
        issues.append('isaac_initialization_request_mismatch')
    init_reset = init_event.get('reset_count')
    if init_reset is None:
        init_reset = init_event.get('state', {}).get('reset_count')
    if init_reset is None:
        raise ValueError('trial_initialized needs reset_count')
    if not init_sim <= initial_stamp / 1e9 + 0.02:
        issues.append('initial_feedback_precedes_isaac_initialization')
    next_init_index = next((i for i in range(init_index + 1, len(events))
                            if events[i].get('event') == 'trial_initialized'), len(events))
    for event in events[init_index + 1:next_init_index]:
        if event.get('event') not in FAULT_EVENTS:
            continue
        if 'wall_monotonic' in event and _finite(event, 'wall_monotonic') > exit_local / 1e9 + 0.05:
            continue
        issues.append(f"isaac_event:{event['event']}")

    in_trial = []
    post_exit = []
    for row in samples:
        try:
            t = _finite(row, 'simulation_time')
        except (KeyError, ValueError):
            issues.append('nonfinite_isaac_simulation_time')
            continue
        if t < init_sim - EPS:
            continue
        if t <= final_stamp / 1e9 + EPS:
            in_trial.append(row)
        elif t <= final_stamp / 1e9 + 0.20 + EPS:
            post_exit.append(row)
    if not in_trial:
        raise ValueError('no Isaac samples in trial simulation-time range')
    max_controller = 0.0
    sample_times = []
    parsed_post_exit = []
    for active, rows in ((True, in_trial), (False, post_exit)):
      for row in rows:
        try:
            names = row['joint_names']
            cart = names.index('slider_to_cart')
            pole = names.index('cart_to_pole')
            forces = row['controller_effort']
            cart_force = float(forces[cart])
            pole_force = float(forces[pole])
            t = _finite(row, 'simulation_time')
            values = [float(row['position'][cart]), float(row['position'][pole]),
                      float(row['velocity'][cart]), float(row['velocity'][pole])]
            if not all(math.isfinite(value) for value in (cart_force, pole_force, *values)):
                raise ValueError('nonfinite Isaac state or controller_effort')
            if row.get('reset_count') != init_reset and active:
                issues.append('isaac_reset_count_changed')
            if row.get('boundary') and active:
                issues.append('isaac_boundary')
            max_controller = max(max_controller, abs(cart_force))
            if active:
                if abs(values[1]) > ABORT_ANGLE_RAD + EPS:
                    issues.append('isaac_pole_angle_boundary')
                if abs(values[0] - config['hold_position']) > ABORT_POSITION_ERROR_M + EPS:
                    issues.append('isaac_cart_position_boundary')
                if abs(cart_force) > FORCE_LIMIT_N + EPS:
                    issues.append('isaac_controller_input_over_5_n')
                if abs(pole_force) > EPS:
                    issues.append('isaac_pole_controller_input_nonzero')
                sample_times.append(t)
            else:
                if abs(cart_force) > FORCE_LIMIT_N + EPS:
                    issues.append('isaac_controller_input_over_5_n_after_exit')
                if abs(pole_force) > EPS:
                    issues.append('isaac_pole_controller_input_nonzero_after_exit')
                parsed_post_exit.append((t, cart_force, pole_force))
        except (KeyError, ValueError, TypeError, IndexError) as exc:
            issues.append(f'invalid_isaac_sample:{exc}')
    for stamp, _ in feedback:
        t = stamp / 1e9
        i = bisect_left(sample_times, t)
        nearest = min((abs(sample_times[j] - t) for j in (i - 1, i)
                       if 0 <= j < len(sample_times)), default=math.inf)
        if nearest > 0.02 + EPS:
            issues.append('feedback_stamp_without_aligned_isaac_sample')
            break
    zero_seen = False
    for _, cart, pole in parsed_post_exit:
        if abs(cart) <= EPS and abs(pole) <= EPS:
            zero_seen = True
        elif zero_seen:
            issues.append('isaac_controller_nonzero_after_exit_zero')
    if not zero_seen:
        issues.append('isaac_controller_zero_after_exit_unverified')

    stable_window, recovery_time = _stability(feedback, config, limits)
    if stable_window is None:
        issues.append('no_one_sim_second_recovery_window')
    reason = exit_row.get('reason')
    result_reason = result.get('reason', result.get('end_reason'))
    if reason != result_reason:
        issues.append('trace_result_reason_mismatch')
    if reason not in ELIGIBLE_END_REASONS:
        issues.append('trial_exit_failure')
    issues = list(dict.fromkeys(issues))
    return {
        'trial_id': trial_id,
        'passed': not issues,
        'initial_state_requested': {'x': config['x0'], 'x_dot': 0.0,
                                    'theta': config['theta0'], 'theta_dot': 0.0},
        'initial_state_feedback': {**initial_state, 'stamp_ns': initial_stamp},
        'hold_position': config['hold_position'],
        'max_published_force_n': max_published,
        'max_controller_input_n': max_controller,
        'saturation_duration_sim_sec': saturation_total,
        'max_continuous_saturation_sim_sec': saturation_max,
        'recovery_time_sim_sec': recovery_time,
        'stable_window': stable_window,
        'end_reason': reason,
        'issues': issues,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_dir', type=Path)
    parser.add_argument('--output', type=Path, help='default: trial_dir/assessment.json')
    args = parser.parse_args(argv)
    try:
        report = assess_trial(args.trial_dir)
        code = 0 if report['passed'] else 1
    except (OSError, ValueError, KeyError, TypeError) as exc:
        report = {'trial_id': args.trial_dir.name, 'passed': False,
                  'end_reason': 'ANALYSIS_ERROR',
                  'issues': [f'analysis_input_error:{exc}']}
        code = 2
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    output = args.output or args.trial_dir / 'assessment.json'
    output.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
