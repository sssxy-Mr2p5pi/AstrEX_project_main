#!/usr/bin/env python3
"""Check online Step 3D/C success against the original ROS and Isaac evidence.

The B assessor still checks the force limits, initialization, stop commands,
and Isaac Controller inputs. This layer independently recomputes the 3-second
feedback window and checks the online result. It never starts a simulator.
"""

import argparse
import json
import math
from pathlib import Path

import analyze_balance_hold_trial as base


MIN_STABLE_SIM_SEC = 3.0
PHYSICAL_END_REASONS = {'SUCCESS', 'SIM_TIMEOUT', 'TIMEOUT', 'SATURATION_TIMEOUT',
                        'TRIAL_BOUNDARY'}
EXPECTED_FAILURE_ISSUES = {
    'trial_exit_failure', 'no_one_sim_second_recovery_window',
    'continuous_saturation_over_0_5_sim_sec', 'ros_pole_angle_boundary',
    'ros_cart_position_boundary', 'isaac_pole_angle_boundary',
    'isaac_cart_position_boundary',
}


def _read_feedback(trace):
    return [(base._ns(row, 'stamp_ns'), base._state(row)) for row in trace
            if row.get('event') in ('initial_feedback', 'feedback')]


def _continuous_window(feedback, config, limits):
    """Compute the final streak from distinct feedback, not cached timer ticks."""
    start = None
    previous = None
    first_complete = None
    duplicate_count = 0
    gap_count = 0
    for stamp, state in feedback:
        if previous is not None and stamp <= previous:
            duplicate_count += 1
            continue
        if previous is not None and stamp - previous > round(limits['max_state_gap_sim_sec'] * 1e9):
            start = None
            gap_count += 1
        if not base._stable(state, config, limits):
            start = None
        elif start is None:
            start = stamp
        if start is not None and stamp - start >= round(limits['stable_window_sim_sec'] * 1e9):
            if first_complete is None:
                first_complete = stamp
        previous = stamp
    window = None if start is None or previous is None else {
        'start_stamp_ns': start, 'end_stamp_ns': previous,
        'duration_sim_sec': (previous - start) / 1e9,
    }
    return window, first_complete, duplicate_count, gap_count


def assess_trial(trial_dir):
    """Return a C verdict; malformed or missing required evidence raises an error."""
    trial_dir = Path(trial_dir)
    report = base.assess_trial(trial_dir)
    config, limits, _ = base._read_config(trial_dir / 'config.json')
    trace = base._jsonl(trial_dir / 'trace.jsonl')
    result = json.loads((trial_dir / 'result.json').read_text(encoding='utf-8'))
    feedback = _read_feedback(trace)
    window, first_complete, duplicates, gaps = _continuous_window(feedback, config, limits)
    issues = list(report['issues'])
    if limits['stable_window_sim_sec'] < MIN_STABLE_SIM_SEC:
        issues.append('c_stable_window_shorter_than_3_sim_sec')
    if result.get('reason') != 'SUCCESS':
        issues.append('online_result_not_success')
    if duplicates:
        issues.append('duplicate_or_out_of_order_feedback_in_c_window')
    if not window or window['duration_sim_sec'] + base.EPS < limits['stable_window_sim_sec']:
        issues.append('no_independent_3_sim_second_terminal_window')

    success_stamp = result.get('success_stamp_ns')
    valid_stamp = isinstance(success_stamp, int) and not isinstance(success_stamp, bool) and success_stamp >= 0
    if not valid_stamp:
        issues.append('missing_or_invalid_online_success_stamp')
    else:
        selected = [(stamp, state) for stamp, state in feedback if stamp == success_stamp]
        if len(selected) != 1:
            issues.append('online_success_stamp_not_one_new_feedback')
        else:
            success_state = result.get('success_state', {})
            try:
                if (base._ns(success_state, 'stamp_ns') != success_stamp
                        or any(not math.isclose(base._finite(success_state, key), state_value,
                                                abs_tol=1e-12, rel_tol=1e-12)
                               for key, state_value in selected[0][1].items())):
                    issues.append('online_success_state_differs_from_feedback')
            except (KeyError, ValueError, TypeError):
                issues.append('missing_or_invalid_online_success_state')
        if not feedback or feedback[-1][0] != success_stamp:
            issues.append('online_success_stamp_is_not_terminal_feedback')
        if first_complete != success_stamp:
            issues.append('online_success_not_first_complete_3_second_window')
        try:
            if base._ns(result.get('final_state', {}), 'stamp_ns') != success_stamp:
                issues.append('final_state_stamp_differs_from_success_stamp')
        except (KeyError, ValueError, TypeError):
            issues.append('missing_or_invalid_final_state_stamp')
        reported_window = result.get('stable_window', {})
        try:
            if (window is None or base._ns(reported_window, 'start_stamp_ns') != window['start_stamp_ns']
                    or base._ns(reported_window, 'end_stamp_ns') != success_stamp
                    or not math.isclose(base._finite(reported_window, 'duration_sim_sec'),
                                        window['duration_sim_sec'], abs_tol=1e-9, rel_tol=1e-9)):
                issues.append('online_window_differs_from_independent_feedback_window')
        except (KeyError, ValueError, TypeError):
            issues.append('missing_or_invalid_online_stable_window')

    valid_trajectory = (len(feedback) >= 2 and not duplicates
                        and result.get('reason') in PHYSICAL_END_REASONS
                        and not (set(report['issues']) - EXPECTED_FAILURE_ISSUES)
                        and (result.get('reason') != 'SUCCESS' or not issues))
    report.update({
        'stage': 'C', 'passed': not issues, 'valid_control_trajectory': valid_trajectory,
        'success_stamp_ns': success_stamp if valid_stamp else None,
        'success_state': result.get('success_state'),
        'online_stable_window': result.get('stable_window'),
        'independent_stable_window': window,
        'first_complete_window_stamp_ns': first_complete,
        'feedback_gap_count': gaps, 'duplicate_feedback_count': duplicates,
        'elapsed_sim_sec': (feedback[-1][0] - feedback[0][0]) / 1e9 if feedback else None,
        'issues': list(dict.fromkeys(issues)),
    })
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial_dir', type=Path)
    parser.add_argument('--output', type=Path, help='default: trial_dir/c_assessment.json')
    args = parser.parse_args(argv)
    try:
        report = assess_trial(args.trial_dir)
        code = 0 if report['passed'] else 1
    except (OSError, ValueError, KeyError, TypeError) as exc:
        report = {'trial_id': args.trial_dir.name, 'stage': 'C', 'passed': False,
                  'valid_control_trajectory': False, 'end_reason': 'ANALYSIS_ERROR',
                  'issues': [f'analysis_input_error:{exc}']}
        code = 2
    encoded = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    (args.output or args.trial_dir / 'c_assessment.json').write_text(encoded + '\n', encoding='utf-8')
    print(encoded)
    return code


if __name__ == '__main__':
    raise SystemExit(main())
