#!/usr/bin/env python3
"""Independently check current CartPole ROS/Isaac evidence without starting either.

The equations, force limits and continuous windows are recomputed from raw logs.
No retired B/C assessor or application control function is imported here.
Controller input is not treated as a measured PhysX applied effort.
"""

import argparse
from bisect import bisect_left
import json
import math
from pathlib import Path


EPS = 1e-9
FORCE_LIMIT_N = 5.0
TOLERANCES = {
    'stable_position_m': 0.05, 'stable_velocity_mps': 0.10,
    'stable_angle_rad': math.radians(2.0),
    'stable_angular_velocity_radps': 0.20,
}
FAULT_EVENTS = {'boundary', 'reset_ready', 'reset_failed', 'test_reset'}


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
                raise ValueError(f'{path}:{number}: expected an object')
            rows.append(row)
    return rows


def _finite(row, key):
    value = row[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{key} must be a finite number')
    return float(value)


def _ns(row, key):
    value = row[key]
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'{key} must be a nonnegative integer stamp')
    return value


def _state(row):
    return {key: _finite(row, key) for key in ('x', 'x_dot', 'theta', 'theta_dot')}


def _reference(start, target, elapsed, speed):
    """Independent quintic reference, including its analytic velocity."""
    if start == target:
        return target, 0.0, 0.0
    duration = max(2.0, 1.875 * abs(target - start) / speed)
    s = max(0.0, min(1.0, elapsed / duration))
    blend = 10 * s**3 - 15 * s**4 + 6 * s**5
    velocity = (target - start) * 30 * s**2 * (1 - s)**2 / duration
    return start + (target - start) * blend, velocity, duration


def _stable(state, target, config):
    return (abs(state['x'] - target) <= config['stable_position_m'] + EPS
            and abs(state['x_dot']) <= config['stable_velocity_mps'] + EPS
            and abs(state['theta']) <= config['stable_angle_rad'] + EPS
            and abs(state['theta_dot']) <= config['stable_angular_velocity_radps'] + EPS)


def _window(feedback, target, config, required, *, not_before=0, historical=False):
    """Return the first complete streak from strictly new feedback stamps."""
    start = previous = None
    for row in feedback:
        stamp = row['stamp_ns']
        if previous is not None and stamp - previous > round(config['max_state_gap_sim_sec'] * 1e9):
            start = None
        good = stamp >= not_before and _stable(row, target, config)
        if historical:
            if abs(config['theta0']) > EPS:
                good = good and abs(row['theta']) < abs(config['theta0']) / 2
            if abs(config['x0'] - target) >= 0.05 - EPS:
                good = good and abs(row['x'] - target) < 0.025
        if not good:
            start = None
        elif start is None:
            start = stamp
        if start is not None and stamp - start >= round(required * 1e9):
            return {'start_stamp_ns': start, 'end_stamp_ns': stamp,
                    'duration_sim_sec': (stamp - start) / 1e9}
        previous = stamp
    return None


def _window_matches(reported, expected):
    if not isinstance(reported, dict) or expected is None:
        return False
    try:
        start = (_ns(reported, 'start_stamp_ns') if 'start_stamp_ns' in reported
                 else round(_finite(reported, 'start_sim_sec') * 1e9))
        end = (_ns(reported, 'end_stamp_ns') if 'end_stamp_ns' in reported
               else round(_finite(reported, 'end_sim_sec') * 1e9))
        return (start == expected['start_stamp_ns'] and end == expected['end_stamp_ns']
                and math.isclose(_finite(reported, 'duration_sim_sec'),
                                 expected['duration_sim_sec'], abs_tol=1e-9))
    except (KeyError, ValueError, TypeError):
        return False


def _saturation(calculations, final_stamp):
    start = previous = None
    longest = total = 0.0
    for row in calculations:
        stamp = row['stamp_ns']
        saturated = abs(row['f_raw']) > FORCE_LIMIT_N + EPS
        if previous is not None:
            gap = (stamp - previous['stamp_ns']) / 1e9
            if previous['saturated'] and gap > 0:
                total += gap
            if gap > 0.05 + EPS or previous['task_phase'] != row.get('task_phase'):
                start = None
        if saturated and start is None:
            start = stamp
        if start is not None:
            longest = max(longest, (stamp - start) / 1e9)
        if not saturated:
            start = None
        previous = {'stamp_ns': stamp, 'saturated': saturated,
                    'task_phase': row.get('task_phase')}
    if previous is not None and previous['saturated'] and final_stamp >= previous['stamp_ns']:
        total += (final_stamp - previous['stamp_ns']) / 1e9
        if start is not None:
            longest = max(longest, (final_stamp - start) / 1e9)
    return total, longest


def _aligned_measured_state(times, states, timestamp, max_offset_sec):
    """Time-align measured samples; never extrapolate or integrate dynamics.

    ROS publishes each physics step. A GUI sample can cover two physics steps.
    At an unlogged midpoint, linearly resample the two measured endpoints.
    This derived state is not an additional raw measurement.
    """
    index = bisect_left(times, timestamp)
    precision_sec = 1e-8  # Timestamp float/nanosecond representation only.
    exact = [j for j in (index-1, index) if 0 <= j < len(times)
             and abs(times[j] - timestamp) <= precision_sec]
    if exact:
        selected = min(exact, key=lambda j: abs(times[j] - timestamp))
        return states[selected], abs(times[selected] - timestamp), 0.0, 'exact'
    if index <= 0 or index >= len(times):
        return None
    left, right = index-1, index
    span = times[right] - times[left]
    if (span <= 0 or span > max_offset_sec + precision_sec
            or timestamp - times[left] > max_offset_sec + precision_sec
            or times[right] - timestamp > max_offset_sec + precision_sec):
        return None
    fraction = (timestamp - times[left]) / span
    state = tuple(a + fraction * (b-a) for a, b in zip(states[left], states[right]))
    offset = max(timestamp-times[left], times[right]-timestamp)
    return state, offset, span, 'bracket_linear_resampling'


def _config(path):
    config = json.loads(path.read_text(encoding='utf-8'))
    task = config.get('task', 'hold')
    if task not in ('hold', 'move', 'move_then_hold'):
        raise ValueError('unknown CartPole task')
    config['task'] = task
    for key in ('x0', 'theta0', 'hold_position', 'force_limit_n'):
        _finite(config, key)
    if config['force_limit_n'] != FORCE_LIMIT_N:
        raise ValueError('force limit must be exactly 5 N')
    for key, maximum in TOLERANCES.items():
        if key not in config:
            config[key] = maximum
        if not 0 < _finite(config, key) <= maximum:
            raise ValueError(f'{key} relaxes the fixed tolerance')
    for key, maximum in (('max_state_gap_sim_sec', 0.05), ('state_timeout_local_sec', 0.2),
                         ('abort_angle_rad', math.radians(10)),
                         ('abort_position_error_m', 0.25), ('max_saturation_sim_sec', 0.5)):
        if key not in config:
            config[key] = maximum
        if not 0 < _finite(config, key) <= maximum:
            raise ValueError(f'{key} relaxes the fixed protection')
    if task != 'hold':
        _finite(config, 'target')
    if 'reference_speed' not in config:
        config['reference_speed'] = config.get('reference_speed_mps', 0.05)
    if not 0 < _finite(config, 'reference_speed') <= 0.05:
        raise ValueError('reference speed exceeds 0.05 m/s')
    return config


def assess_trial(trial_dir):
    """Return a verdict based on raw traces; no file is changed by this function."""
    trial_dir = Path(trial_dir)
    config = _config(trial_dir / 'config.json')
    trace = _jsonl(trial_dir / 'trace.jsonl')
    result = json.loads((trial_dir / 'result.json').read_text(encoding='utf-8'))
    isaac_dir = Path(config['isaac_run_dir'])
    if not isaac_dir.is_absolute():
        isaac_dir = trial_dir / isaac_dir
    dense_path = isaac_dir / 'ros_physics_samples.jsonl'
    dense_samples = dense_path.exists()
    sample_path = dense_path if dense_samples else isaac_dir / 'ros_samples.jsonl'
    # A present dense source is authoritative. Invalid or incomplete dense
    # evidence cannot be hidden by falling back to the older GUI snapshots.
    samples = _jsonl(sample_path)
    events = _jsonl(isaac_dir / 'ros_events.jsonl')
    ready = json.loads((isaac_dir / 'ready.json').read_text(encoding='utf-8'))
    dry = config.get('dry_run', False)
    if not isinstance(dry, bool):
        raise ValueError('dry_run must be boolean')
    task = config['task']
    target = config['hold_position'] if task == 'hold' else config['target']
    issues = []
    initial_rows = [row for row in trace if row.get('event') == 'initial_feedback']
    exit_rows = [row for row in trace if row.get('event') == 'exit']
    controller_ready = [row for row in trace if row.get('event') == 'controller_ready']
    if len(initial_rows) != 1 or len(exit_rows) != 1 or len(controller_ready) != 1:
        raise ValueError('trace needs exactly one controller_ready, initial_feedback and exit')
    initial, exit_row = initial_rows[0], exit_rows[0]
    first_stamp = _ns(initial, 'stamp_ns')
    final_stamp = _ns(result['final_state'], 'stamp_ns')
    final_local = _ns(exit_row, 'local_monotonic_ns')
    feedback = [row for row in trace if row.get('event') in ('initial_feedback', 'feedback')]
    if not feedback:
        raise ValueError('no accepted feedback')
    previous_stamp = previous_local = None
    for row in feedback:
        stamp, local = _ns(row, 'stamp_ns'), _ns(row, 'received_monotonic_ns')
        _state(row)
        if previous_stamp is not None and stamp <= previous_stamp:
            issues.append('duplicate_or_out_of_order_feedback')
        if previous_local is not None and (local < previous_local or
                (local - previous_local) / 1e9 > config['state_timeout_local_sec'] + EPS):
            issues.append('local_state_update_timeout')
        if abs(row['theta']) > config['abort_angle_rad'] + EPS:
            issues.append('pole_angle_boundary')
        previous_stamp, previous_local = stamp, local
    if final_stamp != feedback[-1]['stamp_ns']:
        issues.append('final_state_is_not_last_feedback')
    if (final_local - feedback[-1]['received_monotonic_ns']) / 1e9 > config['state_timeout_local_sec'] + EPS:
        issues.append('final_state_stale')
    if not dry and (abs(initial['x'] - config['x0']) > 0.005
                    or abs(initial['theta'] - config['theta0']) > math.radians(0.2)
                    or abs(initial['x_dot']) > 0.05 or abs(initial['theta_dot']) > 0.1):
        issues.append('initial_feedback_differs_from_requested_state')
    if any(row.get('event') == 'feedback_rejected' for row in trace):
        issues.append('rejected_feedback_during_trial')
    gain = controller_ready[0].get('gain', config.get('lqr_gain', config.get('gain')))
    if isinstance(gain, list) and len(gain) == 1 and isinstance(gain[0], list):
        gain = gain[0]
    if not isinstance(gain, list) or len(gain) != 4 or not all(
            isinstance(value, (int, float)) and math.isfinite(value) for value in gain):
        raise ValueError('controller_ready/config must record the four finite LQR gains')
    reference_start = initial['x']
    _, _, duration = _reference(reference_start, target, 0.0, config['reference_speed'])
    reference_end = first_stamp + round(duration * 1e9)
    scene_limits = ready.get('cart_joint_limits')
    scene_max = ready.get('scene_max_cart_pos')
    if (not isinstance(scene_limits, list) or len(scene_limits) != 2
            or not all(isinstance(v, (int, float)) and math.isfinite(v) for v in scene_limits)
            or not isinstance(scene_max, (int, float)) or not math.isfinite(scene_max)
            or scene_max <= 0 or scene_limits[0] >= scene_limits[1]):
        raise ValueError('ready.json must record valid actual cart and scene limits')
    track_low, track_high = max(scene_limits[0], -scene_max), min(scene_limits[1], scene_max)
    if config.get('scene_bounds') is not None and config['scene_bounds'] != [track_low, track_high]:
        issues.append('config_scene_bounds_differs_from_isaac')
    physics_dt = _finite(ready, 'dt') if 'dt' in ready else 1.0 / 120.0
    if physics_dt <= 0:
        raise ValueError('Isaac physics timestep must be positive')
    corridor_low = max(track_low, min(reference_start, target) - 0.25)
    corridor_high = min(track_high, max(reference_start, target) + 0.25)
    if not track_low < target < track_high:
        issues.append('target_outside_actual_track_limits')
    if task != 'hold':
        recorded_reference = result.get('reference', {})
        try:
            if (_ns(recorded_reference, 'start_stamp_ns') != first_stamp
                    or abs(_ns(recorded_reference, 'end_stamp_ns') - reference_end) > 1
                    or not math.isclose(_finite(recorded_reference, 'duration_sec'), duration, abs_tol=1e-9)
                    or not math.isclose(_finite(recorded_reference, 'x_start'), reference_start, abs_tol=1e-12)
                    or not math.isclose(_finite(recorded_reference, 'target'), target, abs_tol=1e-12)
                    or not math.isclose(_finite(recorded_reference, 'max_speed_mps'), config['reference_speed'], abs_tol=1e-12)):
                issues.append('reference_metadata_differs_from_actual_inputs')
        except (KeyError, ValueError, TypeError):
            issues.append('missing_or_invalid_reference_metadata')
    calculations = [row for row in trace if row.get('event') == 'calculation'
                    or (row.get('event') == 'command' and row.get('phase') == 'control')]
    commands = [row for row in trace if row.get('event') == 'command']
    if not calculations:
        issues.append('missing_control_calculations')
    max_command = max_raw = max_reference_speed = 0.0
    for row in calculations:
        stamp = _ns(row, 'stamp_ns')
        state = _state(row)
        raw, command = _finite(row, 'f_raw'), _finite(row, 'f_cmd')
        x_ref = _finite(row, 'x_ref')
        phase = row.get('task_phase', 'HOLD' if task == 'hold' else 'MOVE')
        if phase == 'MOVE':
            expected, velocity, _ = _reference(reference_start, target,
                                                (stamp - first_stamp) / 1e9,
                                                config['reference_speed'])
        else:
            expected, velocity = target, 0.0
        if not math.isclose(x_ref, expected, abs_tol=1e-9):
            issues.append('reference_differs_from_independent_quintic')
        expected_force = -sum(k * e for k, e in zip(gain, (
            state['x'] - x_ref, state['x_dot'], state['theta'], state['theta_dot'])))
        if not math.isclose(raw, expected_force, abs_tol=1e-8, rel_tol=1e-9):
            issues.append('raw_force_differs_from_recorded_gain_and_state')
        if not math.isclose(command, max(-FORCE_LIMIT_N, min(FORCE_LIMIT_N, raw)), abs_tol=1e-9):
            issues.append('command_not_clipped_raw_force')
        if abs(command) > FORCE_LIMIT_N + EPS:
            issues.append('command_force_over_5_n')
        if abs(_finite(row, 'pole_torque')) > EPS:
            issues.append('nonzero_pole_torque')
        if 'reference_velocity_mps' in row and not math.isclose(
                _finite(row, 'reference_velocity_mps'), velocity, abs_tol=1e-9):
            issues.append('reference_velocity_differs_from_quintic')
        max_raw = max(max_raw, abs(raw))
        max_reference_speed = max(max_reference_speed, abs(velocity))
        if not dry:
            max_command = max(max_command, abs(command))
            if abs(state['x'] - x_ref) > config['abort_position_error_m'] + EPS:
                issues.append('cart_reference_error_boundary')
            if not corridor_low - EPS <= state['x'] <= corridor_high + EPS:
                issues.append('cart_path_corridor_boundary')
        if (_ns(row, 'local_monotonic_ns') - _ns(row, 'received_monotonic_ns')) / 1e9 > config['state_timeout_local_sec'] + EPS:
            issues.append('command_uses_stale_state')
    if max_reference_speed > config['reference_speed'] + EPS:
        issues.append('reference_speed_limit')
    for row in commands:
        force = _finite(row, 'f_cmd')
        max_command = max(max_command, abs(force))
        if abs(force) > FORCE_LIMIT_N + EPS:
            issues.append('published_force_over_5_n')
        if row.get('phase') not in ('control', 'transition_zero', 'stop', 'final_zero'):
            issues.append('unknown_command_phase')
        if row.get('phase') != 'control' and abs(force) > EPS:
            issues.append('nonzero_transition_or_exit_command')
    control_commands = [row for row in commands if row.get('phase') == 'control']
    for previous, current in zip(control_commands, control_commands[1:]):
        if current.get('task_phase') != previous.get('task_phase'):
            continue
        if (current['stamp_ns'] < previous['stamp_ns'] or
                (current['stamp_ns'] - previous['stamp_ns']) / 1e9 > 0.05 + EPS):
            issues.append('control_command_time_gap_or_disorder')
    saturation_total, saturation_max = _saturation(calculations, final_stamp)
    if not dry and saturation_max > config['max_saturation_sim_sec'] + EPS:
        issues.append('continuous_saturation_over_0_5_sim_sec')
    if dry:
        if commands or controller_ready[0].get('publisher_created') is not False or result.get('publisher_created') is not False:
            issues.append('dry_run_created_command_publisher_or_published')
        if (isaac_dir / 'trial_ready.json').exists() or any(
                event.get('event') == 'trial_initialized' for event in events):
            issues.append('dry_run_released_initialization_handshake')
        if result.get('reason') != 'DRY_RUN_PASS':
            issues.append('dry_run_result_not_diagnostic_pass')
        if final_stamp < reference_end:
            issues.append('dry_run_ended_before_reference_endpoint')
    else:
        expected_reason = 'DEMO_SUCCESS' if task == 'move_then_hold' else (
            'SUCCESS' if task == 'hold' else 'MOVE_SUCCESS')
        if result.get('reason') != expected_reason:
            issues.append('online_result_not_expected_success')
        if not commands or commands[-1]['f_cmd'] != 0 or not any(
                row.get('phase') == 'stop' for row in commands):
            issues.append('missing_explicit_exit_zero')
        init_events = [event for event in events if event.get('event') == 'trial_initialized'
                       and event.get('trial_id') == isaac_dir.name]
        if len(init_events) != 1:
            issues.append('missing_or_duplicate_isaac_initialization')
        elif any(abs(_finite(init_events[0]['requested'], key) - config[key]) > EPS
                 for key in ('x0', 'theta0', 'hold_position')):
            issues.append('isaac_initialization_request_mismatch')
    if result.get('reason') != exit_row.get('reason'):
        issues.append('trace_result_reason_mismatch')

    move_window = hold_window = None
    move_stamp = result.get('move_success_stamp_ns')
    hold_stamp = result.get('hold_success_stamp_ns', result.get('success_stamp_ns'))
    if not dry and task in ('move', 'move_then_hold'):
        selected = [row for row in feedback if isinstance(move_stamp, int) and row['stamp_ns'] <= move_stamp]
        move_window = _window(selected, target, config, 1.0, not_before=reference_end)
        if not move_window or move_window['end_stamp_ns'] != move_stamp:
            issues.append('move_success_not_first_complete_1s_window_after_reference')
        if not _window_matches(result.get('move_stable_window'), move_window):
            issues.append('move_online_window_mismatch')
        if selected and result.get('move_success_state') != {key: selected[-1][key] for key in (
                'stamp_ns', 'received_monotonic_ns', 'x', 'x_dot', 'theta', 'theta_dot')}:
            # State dictionaries may contain extra diagnostic fields.
            saved = result.get('move_success_state', {})
            if any(saved.get(key) != selected[-1][key] for key in (
                    'stamp_ns', 'received_monotonic_ns', 'x', 'x_dot', 'theta', 'theta_dot')):
                issues.append('move_success_state_mismatch')
        if task == 'move' and final_stamp != move_stamp:
            issues.append('move_result_not_terminal_success_feedback')
    if not dry and task in ('hold', 'move_then_hold'):
        hold_start = result.get('hold_start_stamp_ns', first_stamp)
        selected = [row for row in feedback if isinstance(hold_start, int) and row['stamp_ns'] >= hold_start]
        hold_window = _window(selected, target, config, 3.0,
                              historical=task == 'hold' and config.get(
                                  'legacy_recovery', config.get('historical_recovery', True)))
        if not hold_window or hold_window['end_stamp_ns'] != hold_stamp:
            issues.append('hold_success_not_first_complete_3s_window')
        reported = result.get('hold_stable_window', result.get('stable_window'))
        if not _window_matches(reported, hold_window):
            issues.append('hold_online_window_mismatch')
        if final_stamp != hold_stamp:
            issues.append('hold_result_not_terminal_success_feedback')
        if task == 'move_then_hold':
            s1 = result.get('s1_state', result.get('hold_start_state', {}))
            zeros = [row for row in trace if row.get('event') == 'phase_zero'
                     or (row.get('event') == 'command' and row.get('phase') == 'transition_zero')]
            if (not isinstance(move_stamp, int) or not isinstance(hold_start, int)
                    or hold_start <= move_stamp or s1.get('stamp_ns') != hold_start):
                issues.append('s1_not_strictly_newer_than_move_success')
            if not zeros or not isinstance(s1.get('received_monotonic_ns'), int):
                issues.append('missing_transition_zero_or_s1_receive_time')
            else:
                zero_local = _ns(zeros[0], 'local_monotonic_ns')
                wait = (s1['received_monotonic_ns'] - zero_local) / 1e9
                if not 0 < wait <= 0.5 + EPS:
                    issues.append('s1_not_received_after_zero_within_wait_limit')
                if any(row.get('event') == 'command' and row.get('phase') == 'control'
                       and zero_local < row['local_monotonic_ns'] < s1['received_monotonic_ns']
                       for row in trace):
                    issues.append('control_continued_while_waiting_for_s1')
            hold_events = [row for row in trace if row.get('event') == 'hold_started']
            if len(hold_events) != 1:
                issues.append('missing_or_duplicate_hold_started_event')
    active_local_start = initial['received_monotonic_ns']
    expected_sim_budget = 10 if task == 'hold' else duration + 8 + (10 if task == 'move_then_hold' else 0)
    if not dry and (final_stamp - first_stamp) / 1e9 > expected_sim_budget + EPS:
        issues.append('task_simulation_timeout')
    local_budget = max(60.0, 3 * expected_sim_budget)
    if (final_local - active_local_start) / 1e9 > local_budget + EPS:
        issues.append('active_local_execution_timeout')

    max_controller = 0.0
    dense_logging_sources = set()
    sample_times = []
    sample_states = []
    zero_seen = False
    reset_counts = set()
    initialization_anchor = None
    alignment_start = first_stamp / 1e9 - 2.0 * physics_dt
    if not dry and len(init_events) == 1:
        initialization_state = init_events[0].get('state', {})
        initialized_time = initialization_state.get('simulation_time')
        if isinstance(initialized_time, (int, float)) and math.isfinite(initialized_time):
            alignment_start = max(alignment_start, initialized_time)
            names = initialization_state.get('joint_names', [])
            measured = init_events[0].get('physx_readback', initialization_state)
            if names.count('slider_to_cart') == 1 and names.count('cart_to_pole') == 1:
                cart, pole = names.index('slider_to_cart'), names.index('cart_to_pole')
                values = tuple(float(measured[key][index]) for key, index in (
                    ('position', cart), ('velocity', cart), ('position', pole), ('velocity', pole)))
                if not all(math.isfinite(value) for value in values):
                    raise ValueError('initialization measured-state anchor is not finite')
                initialization_anchor = (initialized_time, values)
    for row in samples:
        t = _finite(row, 'simulation_time')
        if not alignment_start - EPS <= t <= final_stamp / 1e9 + 0.2 + EPS:
            continue
        if dense_samples:
            source = row.get('logging_source')
            if source != 'physx_post_physics_step_single_articulation':
                issues.append('dense_source_not_post_physics_measurements')
            if isinstance(source, str):
                dense_logging_sources.add(source)
        # A pre-reset rest sample can have the same simulation timestamp as the
        # discontinuous state write. The post-write readback is its only anchor.
        if initialization_anchor is not None and t <= initialization_anchor[0] + EPS:
            continue
        active_sample = first_stamp / 1e9 - EPS <= t <= final_stamp / 1e9 + EPS
        names = row['joint_names']
        if names.count('slider_to_cart') != 1 or names.count('cart_to_pole') != 1:
            issues.append('isaac_joint_names_invalid')
            continue
        cart, pole = names.index('slider_to_cart'), names.index('cart_to_pole')
        values = [float(row[key][index]) for key in ('position', 'velocity', 'controller_effort')
                  for index in (cart, pole)]
        if not all(math.isfinite(value) for value in values):
            issues.append('nonfinite_isaac_state_or_controller_input')
            continue
        cart_force, pole_force = values[4], values[5]
        if active_sample:
            max_controller = max(max_controller, abs(cart_force))
        if abs(cart_force) > FORCE_LIMIT_N + EPS:
            issues.append('isaac_controller_input_over_5_n')
        if abs(pole_force) > EPS:
            issues.append('isaac_pole_controller_input_nonzero')
        if dry and (abs(cart_force) > EPS or abs(pole_force) > EPS):
            issues.append('dry_run_controller_input_nonzero')
        # Post-exit samples may bracket the final half-step feedback. They do
        # not extend the active-task fault, reset or peak-force interval.
        sample_times.append(t)
        sample_states.append((values[0], values[2], values[1], values[3]))
        if active_sample:
            reset_counts.add(row.get('reset_count'))
            if not dry and not corridor_low - EPS <= values[0] <= corridor_high + EPS:
                issues.append('isaac_cart_path_or_track_boundary')
            if abs(values[1]) > config['abort_angle_rad'] + EPS:
                issues.append('isaac_pole_angle_boundary')
            if row.get('boundary'):
                issues.append('isaac_boundary')
        elif t > final_stamp / 1e9 + EPS and abs(cart_force) <= EPS and abs(pole_force) <= EPS:
            zero_seen = True
        elif t > final_stamp / 1e9 + EPS and zero_seen:
            issues.append('isaac_effort_replayed_after_exit_zero')
    if initialization_anchor is not None and not dense_samples:
        sample_times.insert(0, initialization_anchor[0])
        sample_states.insert(0, initialization_anchor[1])
    if not sample_times:
        issues.append('no_aligned_isaac_samples')
    if len(reset_counts) != 1:
        issues.append('isaac_reset_count_changed')
    max_feedback_time_error = 0.0
    max_feedback_state_error = [0.0, 0.0, 0.0, 0.0]
    exact_match_count = resampled_match_count = 0
    max_resampling_span = 0.0
    for row in feedback:
        t = row['stamp_ns'] / 1e9
        if dense_samples:
            index = bisect_left(sample_times, t)
            exact = [j for j in (index-1, index) if 0 <= j < len(sample_times)
                     and abs(sample_times[j] - t) <= 1e-8]
            if exact:
                selected = min(exact, key=lambda j: abs(sample_times[j] - t))
                aligned = (sample_states[selected], abs(sample_times[selected]-t), 0.0, 'exact')
            else:
                aligned = None
        else:
            aligned = _aligned_measured_state(sample_times, sample_states, t, 2.0 * physics_dt)
        if aligned is None:
            issues.append('dense_feedback_without_exact_physics_sample' if dense_samples
                          else 'feedback_without_aligned_isaac_sample')
            break
        measured_state, offset, span, method = aligned
        observed = tuple(row[key] for key in ('x', 'x_dot', 'theta', 'theta_dot'))
        if not all(abs(a-b) <= 0.0002 for a, b in zip(observed, measured_state)):
            issues.append('feedback_state_differs_from_time_aligned_isaac_joint_state')
            break
        exact_match_count += method == 'exact'
        resampled_match_count += method == 'bracket_linear_resampling'
        max_resampling_span = max(max_resampling_span, span)
        max_feedback_time_error = max(max_feedback_time_error, offset)
        max_feedback_state_error = [max(previous, abs(a-b)) for previous, a, b in zip(
            max_feedback_state_error, observed, measured_state)]
    if not zero_seen:
        issues.append('isaac_exit_zero_unverified')
    for event in events:
        local = event.get('wall_monotonic')
        if (event.get('event') in FAULT_EVENTS and isinstance(local, (int, float))
                and active_local_start / 1e9 <= local <= final_local / 1e9):
            issues.append('isaac_fault_event:' + event['event'])
    if ready.get('device') != 'cpu' or not str(ready.get('graph', '')).startswith('/World/AstrEXROSGraph_'):
        issues.append('incorrect_official_isaac_profile')
    if not dry and task in ('move', 'move_then_hold') and abs(
            result['final_state']['x'] - initial['x']) <= 0.001:
        issues.append('no_actual_cart_motion')
    issues = list(dict.fromkeys(issues))
    return {'trial_id': isaac_dir.name, 'task': task, 'dry_run': dry, 'passed': not issues,
            'end_reason': result.get('reason'), 'issues': issues,
            'initial_state_feedback': {**_state(initial), 'stamp_ns': first_stamp},
            'target': target, 'reference_duration_sec': duration,
            'max_reference_speed_mps': max_reference_speed,
            'max_raw_suggested_force_n': max_raw, 'max_published_force_n': max_command,
            'max_controller_input_n': max_controller,
            'saturation_duration_sim_sec': saturation_total,
            'max_continuous_saturation_sim_sec': saturation_max,
            'move_stable_window': move_window, 'hold_stable_window': hold_window,
            'move_success_stamp_ns': move_stamp, 'hold_start_stamp_ns': result.get('hold_start_stamp_ns'),
            'hold_success_stamp_ns': hold_stamp,
            'elapsed_sim_sec': (final_stamp - first_stamp) / 1e9,
            'controller_exit_zero_verified': zero_seen,
            'max_feedback_sample_time_error_sec': max_feedback_time_error,
            'max_feedback_state_error': dict(zip(('x', 'x_dot', 'theta', 'theta_dot'), max_feedback_state_error)),
            'feedback_sample_time_limit_sec': 2.0 * physics_dt,
            'feedback_alignment_method': ('exact per-physics measured sample; no resampling'
                                          if dense_samples else 'exact timestamp or bracket-linear measured-state resampling; no extrapolation'),
            'feedback_alignment_source': sample_path.name,
            'dense_physics_source_present': dense_samples,
            'dense_logging_sources': sorted(dense_logging_sources),
            'feedback_exact_sample_count': exact_match_count,
            'feedback_derived_resampling_count': resampled_match_count,
            'max_feedback_resampling_span_sec': max_resampling_span,
            'initialization_measured_anchor_time_sec': (initialization_anchor[0]
                                                      if initialization_anchor and not dense_samples else None),
            'feedback_state_tolerance': 0.0002,
            'applied_effort': 'N/A: Controller input is not measured PhysX applied effort'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_dir', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    try:
        report = assess_trial(args.trial_dir)
        code = 0 if report['passed'] else 1
    except (OSError, ValueError, KeyError, TypeError, IndexError) as exc:
        report = {'trial_id': args.trial_dir.name, 'passed': False,
                  'end_reason': 'ANALYSIS_ERROR', 'issues': [f'analysis_input_error:{exc}']}
        code = 2
    encoded = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
    output = args.output or args.trial_dir / 'assessment.json'
    output.write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
