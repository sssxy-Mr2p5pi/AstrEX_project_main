"""Synthetic C evidence checks. No ROS, Isaac, or historical trial writes."""

import json
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import analyze_balance_hold_c_trial as assessor  # noqa: E402


def _write_jsonl(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')


def _case(tmp_path):
    trial = tmp_path / 'trial'
    isaac = tmp_path / 'isaac'
    trial.mkdir()
    isaac.mkdir()
    theta0 = math.radians(2)
    config = {
        'x0': 0.0, 'theta0': theta0, 'hold_position': 0.0,
        'force_limit_n': 5.0, 'max_sim_sec': 10.0, 'max_local_sec': 30.0,
        'stable_position_m': 0.05, 'stable_velocity_mps': 0.10,
        'stable_angle_rad': math.radians(2), 'stable_angular_velocity_radps': 0.20,
        'stable_window_sim_sec': 3.0, 'max_state_gap_sim_sec': 0.05,
        'isaac_run_dir': str(isaac),
    }
    trace = [
        {'event': 'controller_ready', 'local_monotonic_ns': 9_900_000_000},
        {'event': 'trial_initialized', 'local_monotonic_ns': 9_990_000_000},
    ]
    samples = []
    for step in range(122):
        stamp = 1_000_000_000 + step * 25_000_000
        theta = theta0 if step == 0 else theta0 / 4
        state = {'stamp_ns': stamp, 'received_monotonic_ns': 10_000_000_000 + step * 25_000_000,
                 'x': 0.0, 'x_dot': 0.0, 'theta': theta, 'theta_dot': 0.0}
        trace.append({'event': 'initial_feedback' if step == 0 else 'feedback', **state})
        force = -4.0 if step == 0 else 0.0
        if step < 121:
            trace.append({'event': 'command', 'phase': 'control', 'f_raw': force,
                          'f_cmd': force, 'hold_position': 0.0, 'stamp_ns': stamp})
        samples.append({'simulation_time': stamp / 1e9, 'reset_count': 0,
                        'joint_names': ['slider_to_cart', 'cart_to_pole'],
                        'position': [0.0, theta], 'velocity': [0.0, 0.0],
                        'controller_effort': [force, 0.0], 'boundary': []})
    final = state
    result = {'reason': 'SUCCESS', 'success_stamp_ns': final['stamp_ns'],
              'success_state': final, 'final_state': final,
              'stable_window': {'start_stamp_ns': 1_025_000_000,
                                'end_stamp_ns': final['stamp_ns'], 'duration_sim_sec': 3.0}}
    for phase in ('stop', 'final_zero'):
        trace.append({'event': 'command', 'phase': phase, 'f_raw': None,
                      'f_cmd': 0.0, 'hold_position': 0.0})
    trace.append({'event': 'exit', 'reason': result['reason'], 'final_state': final,
                  'local_monotonic_ns': 13_040_000_000})
    samples.append({**samples[-1], 'simulation_time': 4.05, 'controller_effort': [0.0, 0.0]})
    events = [{'event': 'trial_initialized', 'trial_id': isaac.name,
               'wall_monotonic': 9.99, 'reset_count': 0,
               'state': {'simulation_time': 0.999, 'reset_count': 0}}]
    return trial, isaac, config, result, trace, samples, events


def _assess(case):
    trial, isaac, config, result, trace, samples, events = case
    (trial / 'config.json').write_text(json.dumps(config), encoding='utf-8')
    (trial / 'result.json').write_text(json.dumps(result), encoding='utf-8')
    _write_jsonl(trial / 'trace.jsonl', trace)
    _write_jsonl(isaac / 'ros_samples.jsonl', samples)
    _write_jsonl(isaac / 'ros_events.jsonl', events)
    return assessor.assess_trial(trial)


def test_pass_needs_online_success_and_independent_three_second_window(tmp_path):
    report = _assess(_case(tmp_path))
    assert report['passed'], report['issues']
    assert report['valid_control_trajectory']
    assert report['independent_stable_window']['duration_sim_sec'] == pytest.approx(3)
    assert report['success_stamp_ns'] == 4_025_000_000
    assert report['max_published_force_n'] == 4


@pytest.mark.parametrize('change,issue', [
    ('stamp', 'online_success_stamp_not_one_new_feedback'),
    ('state', 'online_success_state_differs_from_feedback'),
    ('window', 'online_window_differs_from_independent_feedback_window'),
    ('short', 'c_stable_window_shorter_than_3_sim_sec'),
    ('reason', 'online_result_not_success'),
])
def test_result_fields_are_not_trusted_without_raw_evidence(tmp_path, change, issue):
    case = _case(tmp_path)
    result = case[3]
    if change == 'stamp':
        result['success_stamp_ns'] += 1
    elif change == 'state':
        result['success_state'] = {**result['success_state'], 'theta': 0.0}
    elif change == 'window':
        result['stable_window']['start_stamp_ns'] -= 100_000_000
    elif change == 'short':
        case[2]['stable_window_sim_sec'] = 1.0
    else:
        result['reason'] = case[4][-1]['reason'] = 'SIM_TIMEOUT'
    report = _assess(case)
    assert not report['passed']
    assert issue in report['issues']
    if change != 'reason':
        # A malformed claimed SUCCESS is not a physical capability failure.
        assert not report['valid_control_trajectory']


@pytest.mark.parametrize('change', ['duplicate', 'gap', 'condition_break'])
def test_cache_duplicates_or_interrupted_feedback_cannot_prove_success(tmp_path, change):
    case = _case(tmp_path)
    trace = case[4]
    if change == 'duplicate':
        index = next(i for i, row in enumerate(trace) if row.get('event') == 'feedback')
        trace.insert(index + 1, dict(trace[index]))
    elif change == 'gap':
        trace[:] = [row for row in trace if not (row.get('event') == 'feedback'
                                                and 2_000_000_000 <= row['stamp_ns'] <= 2_100_000_000)]
    else:
        row = next(row for row in trace if row.get('event') == 'feedback'
                   and row['stamp_ns'] == 2_000_000_000)
        row['x_dot'] = 0.11
    assert not _assess(case)['passed']


def test_valid_bounded_control_failure_remains_measurement_not_startup_failure(tmp_path):
    case = _case(tmp_path)
    case[3]['reason'] = case[4][-1]['reason'] = 'SIM_TIMEOUT'
    for key in ('success_stamp_ns', 'success_state', 'stable_window'):
        case[3].pop(key)
    report = _assess(case)
    assert not report['passed']
    assert report['valid_control_trajectory']
    # Broken Controller evidence is not a capability failure that permits continuation.
    case[5][5]['controller_effort'][0] = 5.1
    assert not _assess(case)['valid_control_trajectory']


def test_boundary_fault_overrides_success_claim(tmp_path):
    case = _case(tmp_path)
    case[6].append({'event': 'boundary', 'wall_monotonic': 12.0})
    report = _assess(case)
    assert not report['passed']
    assert 'isaac_event:boundary' in report['issues']
