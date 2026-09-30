"""Synthetic log tests for the offline B assessor; never start ROS or Isaac."""

import json
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from analyze_balance_hold_trial import assess_trial  # noqa: E402


def _write_jsonl(path, rows):
    path.write_text(''.join(json.dumps(row) + '\n' for row in rows), encoding='utf-8')


def _case(tmp_path):
    trial = tmp_path / 'case_001'
    isaac = tmp_path / 'isaac_run_001'
    trial.mkdir()
    isaac.mkdir()
    theta0 = math.radians(1)
    config = {
        'x0': 0.0, 'theta0': theta0, 'hold_position': 0.0,
        'force_limit_n': 5.0, 'max_sim_sec': 10.0, 'max_local_sec': 30.0,
        'stable_position_m': 0.05, 'stable_velocity_mps': 0.10,
        'stable_angle_rad': math.radians(2),
        'stable_angular_velocity_radps': 0.20,
        'stable_window_sim_sec': 1.0, 'max_state_gap_sim_sec': 0.05,
        'isaac_run_dir': str(isaac),
    }
    trace = [
        {'event': 'controller_ready', 'local_monotonic_ns': round(9.9e9)},
        {'event': 'trial_initialized', 'local_monotonic_ns': round(9.99e9)},
    ]
    samples = []
    for step in range(49):  # 1.000 through 2.200 simulated seconds.
        t = 1 + step * 0.025
        stamp = round(t * 1e9)
        local = round((10 + t - 1) * 1e9)
        theta = theta0 if step == 0 else theta0 * 0.25
        state = {
            'stamp_ns': stamp, 'received_monotonic_ns': local,
            'x': 0.0, 'x_dot': 0.0, 'theta': theta, 'theta_dot': 0.0,
        }
        trace.append({'event': 'initial_feedback' if step == 0 else 'feedback', **state})
        force = -2.0 if step == 0 else 0.0
        if step < 48:
            trace.append({
                'event': 'command', 'stamp_ns': stamp, 'received_monotonic_ns': local,
                'f_raw': force, 'f_cmd': force, 'hold_position': 0.0,
                'phase': 'control',
            })
        samples.append({
            'physics_step': step, 'simulation_time': t, 'reset_count': 0,
            'joint_names': ['slider_to_cart', 'cart_to_pole'],
            'position': [0.0, theta], 'velocity': [0.0, 0.0],
            'controller_effort': [force, 0.0], 'boundary': [],
        })
    trace.append({
        'event': 'command', 'phase': 'stop', 'f_raw': None, 'f_cmd': 0.0,
        'hold_position': 0.0,
    })
    trace.append({
        'event': 'command', 'phase': 'final_zero', 'f_raw': None, 'f_cmd': 0.0,
        'hold_position': 0.0,
    })
    trace.append({
        'event': 'exit', 'reason': 'SIM_WINDOW_END',
        'final_state': {'stamp_ns': round(2.2e9)},
        'local_monotonic_ns': round(11.21e9),
    })
    samples.append({
        'physics_step': 49, 'simulation_time': 2.225, 'reset_count': 0,
        'joint_names': ['slider_to_cart', 'cart_to_pole'],
        'position': [0.0, theta0 * 0.25], 'velocity': [0.0, 0.0],
        'controller_effort': [0.0, 0.0], 'boundary': [],
    })
    events = [{
        'event': 'trial_initialized', 'trial_id': isaac.name,
        'wall_monotonic': 9.99, 'reset_count': 0,
        'state': {'simulation_time': 0.999, 'reset_count': 0},
    }]
    return trial, isaac, config, trace, samples, events


def _assess(case):
    trial, isaac, config, trace, samples, events = case
    (trial / 'config.json').write_text(json.dumps(config), encoding='utf-8')
    (trial / 'result.json').write_text(json.dumps({'reason': trace[-1]['reason']}), encoding='utf-8')
    _write_jsonl(trial / 'trace.jsonl', trace)
    _write_jsonl(isaac / 'ros_samples.jsonl', samples)
    _write_jsonl(isaac / 'ros_events.jsonl', events)
    return assess_trial(trial)


def test_pass_uses_actual_feedback_and_sim_time_window(tmp_path):
    report = _assess(_case(tmp_path))
    assert report['passed'], report['issues']
    assert report['trial_id'] == 'isaac_run_001'
    assert report['max_published_force_n'] == 2.0
    assert report['max_controller_input_n'] == 2.0
    assert report['recovery_time_sim_sec'] == pytest.approx(0.025)
    assert report['stable_window']['duration_sim_sec'] >= 1.0


@pytest.mark.parametrize('source,issue', [
    ('published', 'published_force_over_5_n'),
    ('controller', 'isaac_controller_input_over_5_n'),
    ('pole', 'isaac_pole_controller_input_nonzero'),
])
def test_force_checks_cover_both_command_layers_and_pole(tmp_path, source, issue):
    case = _case(tmp_path)
    _, _, _, trace, samples, _ = case
    if source == 'published':
        control = next(row for row in trace if row.get('phase') == 'control')
        control['f_raw'] = control['f_cmd'] = 5.1
    elif source == 'controller':
        samples[1]['controller_effort'][0] = 5.1
    else:
        samples[1]['controller_effort'][1] = 0.1
    report = _assess(case)
    assert not report['passed']
    assert issue in report['issues']


def test_saturation_is_measured_in_sim_seconds_and_not_sample_count(tmp_path):
    case = _case(tmp_path)
    trace = case[3]
    for row in trace:
        if row['event'] == 'command' and row.get('phase') == 'control' and row['stamp_ns'] < round(1.55e9):
            row['f_raw'], row['f_cmd'] = 6.0, 5.0
    report = _assess(case)
    assert not report['passed']
    assert report['max_continuous_saturation_sim_sec'] == pytest.approx(0.55)
    assert 'continuous_saturation_over_0_5_sim_sec' in report['issues']


def test_stable_window_requires_recovery_and_unbroken_feedback(tmp_path):
    case = _case(tmp_path)
    for row in case[3]:
        if row['event'] == 'feedback':
            row['theta'] = math.radians(0.6)  # Inside 2°, above half of initial 1°.
    report = _assess(case)
    assert not report['passed']
    assert 'no_one_sim_second_recovery_window' in report['issues']


def test_feedback_gap_resets_stability_window(tmp_path):
    case = _case(tmp_path)
    trace = case[3]
    trace[:] = [row for row in trace if not (row['event'] == 'feedback'
                                            and round(1.5e9) <= row['stamp_ns'] <= round(1.6e9))]
    report = _assess(case)
    assert not report['passed']
    assert 'no_one_sim_second_recovery_window' in report['issues']


@pytest.mark.parametrize('fault,issue', [
    ('boundary', 'isaac_boundary'),
    ('reset', 'isaac_reset_count_changed'),
    ('event', 'isaac_event:boundary'),
    ('zero', 'isaac_controller_zero_after_exit_unverified'),
    ('stale', 'local_state_update_timeout'),
])
def test_faults_and_exit_zero_cannot_be_counted_as_recovery(tmp_path, fault, issue):
    case = _case(tmp_path)
    _, _, _, trace, samples, events = case
    if fault == 'boundary':
        samples[10]['boundary'] = ['cart_position']
    elif fault == 'reset':
        samples[10]['reset_count'] = 1
    elif fault == 'event':
        events.append({'event': 'boundary', 'wall_monotonic': 10.2})
    elif fault == 'zero':
        samples[-1]['controller_effort'][0] = 1.0
    else:
        for row in trace:
            if row['event'] == 'feedback' and row['stamp_ns'] == round(1.5e9):
                row['received_monotonic_ns'] += round(0.3e9)
                break
    report = _assess(case)
    assert not report['passed']
    assert issue in report['issues']


def test_config_cannot_relax_fixed_b_tolerances(tmp_path):
    case = _case(tmp_path)
    case[2]['stable_angle_rad'] = math.radians(3)
    with pytest.raises(ValueError, match='stable_angle_rad'):
        _assess(case)


def test_exact_half_second_saturation_is_allowed(tmp_path):
    case = _case(tmp_path)
    for row in case[3]:
        if (row.get('phase') == 'control'
                and row['stamp_ns'] < round(1.5e9)):
            row['f_raw'], row['f_cmd'] = 6.0, 5.0
    report = _assess(case)
    assert report['passed'], report['issues']
    assert report['max_continuous_saturation_sim_sec'] == pytest.approx(0.5)


def test_five_centimeter_start_requires_position_recovery(tmp_path):
    case = _case(tmp_path)
    case[2]['x0'] = 0.05
    for row in case[3]:
        if row['event'] in ('initial_feedback', 'feedback'):
            row['x'] = 0.05 if row['event'] == 'initial_feedback' else 0.03
    case[4][0]['position'][0] = 0.05
    for row in case[4][1:-1]:
        row['position'][0] = 0.03
    report = _assess(case)
    assert not report['passed']
    assert 'no_one_sim_second_recovery_window' in report['issues']


@pytest.mark.parametrize('source,issue', [
    ('raw', 'invalid_command:'),
    ('isaac_state', 'invalid_isaac_sample:'),
])
def test_nonfinite_log_values_reject_trial(tmp_path, source, issue):
    case = _case(tmp_path)
    if source == 'raw':
        control = next(row for row in case[3] if row.get('phase') == 'control')
        control['f_raw'] = float('inf')
    else:
        case[4][3]['velocity'][0] = float('nan')
    report = _assess(case)
    assert not report['passed']
    assert any(value.startswith(issue) for value in report['issues'])


def test_commandless_zero_trajectory_does_not_prove_closed_loop(tmp_path):
    case = _case(tmp_path)
    case[3][:] = [row for row in case[3] if row.get('phase') != 'control']
    report = _assess(case)
    assert not report['passed']
    assert 'no_closed_loop_control_commands' in report['issues']
