"""Synthetic message/log checks only. No dynamics integration, ROS or Isaac."""

import json
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import analyze_cartpole_trial as assessor  # noqa: E402


def _write_rows(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')


def _dataset(tmp_path, *, dry=False, combined=False):
    """Explicit valid feedback sequence; it does not model physics."""
    trial, isaac = tmp_path / 'trial', tmp_path / 'isaac'
    trial.mkdir()
    isaac.mkdir()
    target, duration = 0.3, 11.25
    gain = [-10.0, -27.0, 115.0, 26.0]
    config = {'task': 'move_then_hold' if combined else 'move', 'dry_run': dry,
              'x0': 0.0, 'theta0': 0.0, 'hold_position': 0.0, 'target': target,
              'force_limit_n': 5.0, 'isaac_run_dir': str(isaac), 'reference_speed_mps': 0.05,
              'legacy_recovery': False}
    trace = [{'event': 'controller_ready', 'local_monotonic_ns': 100_990_000_000,
              'gain': [gain], 'publisher_created': not dry}]
    samples = []
    feedback = []
    reference_end = round((1 + duration) * 1e9)
    move_success = reference_end + 1_000_000_000
    hold_start = move_success + 25_000_000
    final_stamp = hold_start + 3_000_000_000 if combined else move_success
    steps = round((final_stamp / 1e9 - 1) / 0.025)
    for step in range(steps + 1):
        stamp = 1_000_000_000 + step * 25_000_000
        elapsed = step * 0.025
        s = min(1.0, elapsed / duration)
        reference = target * (10*s**3 - 15*s**4 + 6*s**5)
        reference_velocity = target * 30*s**2 * (1-s)**2 / duration
        x, velocity = (0.0, 0.0) if dry else (reference, reference_velocity)
        phase = 'HOLD' if combined and stamp >= hold_start else 'MOVE'
        received = stamp + 100_000_000_000
        state = {'stamp_ns': stamp, 'received_monotonic_ns': received,
                 'x': x, 'x_dot': velocity, 'theta': 0.0, 'theta_dot': 0.0}
        feedback.append(state)
        trace.append({'event': 'initial_feedback' if step == 0 else 'feedback',
                      'local_monotonic_ns': received + 1000, 'task_phase': phase, **state})
        raw = -(gain[0] * (x-reference) + gain[1] * velocity)
        command = max(-5.0, min(5.0, raw))
        if stamp < final_stamp and not (combined and stamp == move_success):
            trace.append({'event': 'calculation' if dry else 'command', 'phase': 'control',
                          'local_monotonic_ns': received + 2000, 'task_phase': phase,
                          'x_ref': reference, 'target': target, 'reference_velocity_mps': reference_velocity,
                          'f_raw': raw, 'f_cmd': command, 'pole_torque': 0.0, **state})
        if combined and stamp == move_success:
            trace.append({'event': 'move_success', 'local_monotonic_ns': received + 3000, **state})
            trace.append({'event': 'command', 'phase': 'transition_zero', 'f_raw': None,
                          'f_cmd': 0.0, 'pole_torque': 0.0,
                          'local_monotonic_ns': received + 5000, 'task_phase': 'WAIT_S1'})
            trace.append({'event': 'phase_zero', 'local_monotonic_ns': received + 6000,
                          'stamp_ns': stamp})
        if combined and stamp == hold_start:
            trace.append({'event': 'hold_started', 'local_monotonic_ns': received + 3000, **state})
        samples.append({'simulation_time': stamp/1e9, 'reset_count': 0,
                        'joint_names': ['cart_to_pole', 'slider_to_cart'],
                        'position': [0.0, x], 'velocity': [0.0, velocity],
                        'controller_effort': [0.0, 0.0 if dry else command], 'boundary': []})
    reason = 'DRY_RUN_PASS' if dry else ('DEMO_SUCCESS' if combined else 'MOVE_SUCCESS')
    move_window = {'start_stamp_ns': reference_end, 'end_stamp_ns': move_success,
                   'duration_sim_sec': 1.0}
    hold_window = {'start_stamp_ns': hold_start, 'end_stamp_ns': final_stamp,
                   'duration_sim_sec': 3.0}
    result = {'reason': reason, 'final_state': feedback[-1], 'publisher_created': not dry,
              'reference': {'start_stamp_ns': 1_000_000_000, 'end_stamp_ns': reference_end,
                            'duration_sec': duration, 'x_start': 0.0, 'target': target,
                            'max_speed_mps': 0.05}}
    if not dry:
        result.update({'move_success_stamp_ns': move_success,
                       'move_success_state': next(row for row in feedback if row['stamp_ns'] == move_success),
                       'move_stable_window': move_window})
        if combined:
            result.update({'hold_start_stamp_ns': hold_start,
                           's1_state': next(row for row in feedback if row['stamp_ns'] == hold_start),
                           'hold_success_stamp_ns': final_stamp, 'hold_stable_window': hold_window})
        trace.extend([{'event': 'command', 'phase': phase, 'f_raw': None, 'f_cmd': 0.0,
                       'pole_torque': 0.0, 'local_monotonic_ns': final_stamp + 100_000_005_000}
                      for phase in ('stop', 'final_zero')])
    trace.append({'event': 'exit', 'reason': reason, 'final_state': feedback[-1],
                  'local_monotonic_ns': final_stamp + 100_010_000_000})
    samples.append({'simulation_time': final_stamp/1e9 + 0.025, 'reset_count': 0,
                    'joint_names': ['slider_to_cart', 'cart_to_pole'],
                    'position': [feedback[-1]['x'], 0.0], 'velocity': [0.0, 0.0],
                    'controller_effort': [0.0, 0.0], 'boundary': []})
    events = [] if dry else [{'event': 'trial_initialized', 'trial_id': isaac.name,
                              'requested': {key: config[key] for key in ('x0', 'theta0', 'hold_position')},
                              'wall_monotonic': 100.99}]
    return trial, isaac, config, trace, samples, events, result


def _assess(data):
    trial, isaac, config, trace, samples, events, result = data
    (trial/'config.json').write_text(json.dumps(config))
    (trial/'result.json').write_text(json.dumps(result))
    (isaac/'ready.json').write_text(json.dumps({'device': 'cpu', 'graph': '/World/AstrEXROSGraph_test',
                                              'cart_joint_limits': [-4.0, 4.0], 'scene_max_cart_pos': 3.0}))
    _write_rows(trial/'trace.jsonl', trace)
    _write_rows(isaac/'ros_samples.jsonl', samples)
    _write_rows(isaac/'ros_events.jsonl', events)
    return assessor.assess_trial(trial)


@pytest.mark.parametrize('dry,combined', [(False, False), (False, True), (True, False)])
def test_independent_complete_evidence_passes(tmp_path, dry, combined):
    report = _assess(_dataset(tmp_path, dry=dry, combined=combined))
    assert report['passed'], report['issues']
    assert report['max_controller_input_n'] <= 5
    if dry:
        assert report['max_published_force_n'] == 0
        assert report['end_reason'] == 'DRY_RUN_PASS'
        assert report['move_stable_window'] is None
    elif combined:
        assert report['hold_stable_window']['duration_sim_sec'] == 3


@pytest.mark.parametrize('source,issue', [
    ('raw', 'raw_force_differs_from_recorded_gain_and_state'),
    ('reference', 'reference_differs_from_independent_quintic'),
    ('controller', 'isaac_controller_input_over_5_n'),
    ('pole', 'isaac_pole_controller_input_nonzero'),
    ('stale', 'local_state_update_timeout'),
    ('feedback', 'feedback_state_differs_from_time_aligned_isaac_joint_state'),
    ('zero', 'isaac_exit_zero_unverified'),
])
def test_tampered_evidence_fails(tmp_path, source, issue):
    data = _dataset(tmp_path)
    if source in ('raw', 'reference'):
        row = next(row for row in data[3] if row.get('phase') == 'control')
        row['f_raw' if source == 'raw' else 'x_ref'] += 0.1
    elif source in ('controller', 'pole'):
        data[4][5]['controller_effort'][1 if source == 'controller' else 0] = 5.1 if source == 'controller' else 0.1
    elif source == 'stale':
        next(row for row in data[3] if row.get('event') == 'feedback')['received_monotonic_ns'] += 300_000_000
    elif source == 'feedback':
        next(row for row in data[3] if row.get('event') == 'feedback')['theta'] = 0.02
    else:
        data[4][-1]['controller_effort'] = [1.0, 0.0]
    report = _assess(data)
    assert not report['passed']
    assert issue in report['issues']


@pytest.mark.parametrize('tamper,issue', [
    ('publisher', 'dry_run_created_command_publisher_or_published'),
    ('effort', 'dry_run_controller_input_nonzero'),
    ('handshake', 'dry_run_released_initialization_handshake'),
])
def test_dry_run_cannot_claim_no_output_without_evidence(tmp_path, tamper, issue):
    data = _dataset(tmp_path, dry=True)
    if tamper == 'publisher':
        data[3][0]['publisher_created'] = True
    elif tamper == 'effort':
        data[4][5]['controller_effort'][1] = 0.01
    else:
        (data[1]/'trial_ready.json').write_text('{}')
    report = _assess(data)
    assert not report['passed']
    assert issue in report['issues']


@pytest.mark.parametrize('tamper,issue', [
    ('s1_stamp', 's1_not_strictly_newer_than_move_success'),
    ('s1_receive', 's1_not_received_after_zero_within_wait_limit'),
    ('hold_window', 'hold_online_window_mismatch'),
    ('move_window', 'move_online_window_mismatch'),
])
def test_sequential_transition_cannot_borrow_old_state_or_stable_time(tmp_path, tamper, issue):
    data = _dataset(tmp_path, combined=True)
    result = data[-1]
    if tamper == 's1_stamp':
        result['s1_state'] = dict(result['s1_state'], stamp_ns=result['move_success_stamp_ns'])
    elif tamper == 's1_receive':
        result['s1_state'] = dict(result['s1_state'], received_monotonic_ns=100_000_000_000)
    elif tamper == 'hold_window':
        result['hold_stable_window']['duration_sim_sec'] = 1.0
    else:
        result['move_stable_window']['start_stamp_ns'] -= 25_000_000
    report = _assess(data)
    assert not report['passed']
    assert issue in report['issues']


def test_reference_has_known_midpoint_peak_velocity_and_fixed_endpoint():
    x, velocity, duration = assessor._reference(0.0, 0.3, 5.625, 0.05)
    assert (x, velocity, duration) == pytest.approx((0.15, 0.05, 11.25))
    assert assessor._reference(0.0, -0.3, 11.25, 0.05)[:2] == (-0.3, 0.0)
    assert assessor._reference(0.0, 0.5, 100.0, 0.05)[:2] == (0.5, 0.0)
    assert assessor._reference(0.3, 0.3, 0.0, 0.05) == (0.3, 0.0, 0.0)
