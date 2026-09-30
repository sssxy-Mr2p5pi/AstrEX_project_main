#!/usr/bin/env python3
"""Run the six Step 3D/C cases without repeating the B matrix.

Each trial uses the established B process ownership/cleanup machinery and a
new Isaac process. Valid control-capability failures remain evidence and do
not erase later planned cases. Missing data or operational failures stop the
run. Resume rechecks raw evidence before it skips a saved PASS.
"""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import sys
import uuid

import run_balance_hold_matrix as base
from analyze_balance_hold_c_trial import assess_trial


ROOT = Path(__file__).resolve().parents[1]
C_ROOT = base.SHARED_ROOT / 'balance_hold_step3d' / 'c_runs'
VERSION = 1
PREFIX_CASES = (
    base.Case('c_online_hold_0_theta_p2', 1, 0.0, 0.0, math.radians(2)),
    base.Case('c_hold_0_theta_m3', 2, 0.0, 0.0, math.radians(-3)),
    base.Case('c_hold_0_theta_p3', 2, 0.0, 0.0, math.radians(3)),
    base.Case('c_hold_0_theta_m5', 3, 0.0, 0.0, math.radians(-5)),
    base.Case('c_hold_0_theta_p5', 3, 0.0, 0.0, math.radians(5)),
)


def fingerprints():
    paths = (*base.FINGERPRINT_PATHS, 'scripts/run_balance_hold_c.py',
             'scripts/analyze_balance_hold_c_trial.py')
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in paths}


def _config_matches(trial_dir, case):
    config = json.loads((trial_dir / 'config.json').read_text(encoding='utf-8'))
    return base._matches_case(config, case) and float(config.get('stable_window_sim_sec', 0)) >= 3.0


def verified_pass(trial_dir, case):
    try:
        if not _config_matches(trial_dir, case):
            return None
        saved = json.loads((trial_dir / 'c_assessment.json').read_text(encoding='utf-8'))
        current = assess_trial(trial_dir)
        if not saved.get('passed') or not current['passed'] or saved.get('trial_id') != current['trial_id']:
            return None
        return current
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _latest_record(manifest, case):
    records = manifest['results'].get(case.name, [])
    return records[-1] if records else None


def _verified_completed(manifest, case):
    record = _latest_record(manifest, case)
    if not record or record.get('case') != asdict(case):
        return None
    if record.get('status') == 'PASS':
        current = verified_pass(Path(record['trial_dir']), case)
        if current:
            return {**record, 'c_assessment': current}
    if record.get('status') == 'FAIL' and record.get('valid_control_failure'):
        try:
            trial = Path(record['trial_dir'])
            current = assess_trial(trial)
            if _config_matches(trial, case) and current['valid_control_trajectory'] and not current['passed']:
                return {**record, 'c_assessment': current}
        except (OSError, ValueError, KeyError, TypeError):
            pass
    return None


def _direction_score(record):
    assessment = record.get('c_assessment') or {}
    # Failed recovery ranks as harder than a successful recovery. For two
    # failures, a longer measured run ranks first, then longer saturation.
    failed = record.get('status') != 'PASS'
    duration = assessment.get('elapsed_sim_sec') if failed else assessment.get('recovery_time_sim_sec')
    duration = float(duration) if isinstance(duration, (float, int)) and math.isfinite(duration) else -1.0
    saturation = assessment.get('max_continuous_saturation_sim_sec', 0.0)
    return (failed, duration, float(saturation))


def choose_position_case(manifest):
    minus, plus = (_verified_completed(manifest, case) for case in PREFIX_CASES[-2:])
    if minus is None or plus is None:
        raise ValueError('Both 5-degree trials need verified raw evidence before direction selection')
    chosen = minus if _direction_score(minus) >= _direction_score(plus) else plus
    sign = -1 if chosen['case']['theta0'] < 0 else 1
    case = base.Case(f'c_hold_p0p3_theta_{"m" if sign < 0 else "p"}5',
                     4, 0.3, 0.3, math.radians(sign * 5))
    explanation = {
        'selected_source_case': chosen['case']['name'],
        'rule': 'Failure ranks harder; otherwise longer recovery. Two failures: longer measured run, then saturation. Ties choose negative.',
        'minus_score': list(_direction_score(minus)), 'plus_score': list(_direction_score(plus)),
    }
    return case, explanation


def run_case(run_dir, case):
    """Reuse bounded process launch/cleanup, then add the independent C gate."""
    record = base.run_case(run_dir, case)
    trial = Path(record['trial_dir'])
    try:
        c_report = assess_trial(trial)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        c_report = {'stage': 'C', 'passed': False, 'valid_control_trajectory': False,
                    'issues': [f'analysis_input_error:{exc}'], 'end_reason': 'ANALYSIS_ERROR'}
    if trial.is_dir():
        (trial / 'c_assessment.json').write_text(
            json.dumps(c_report, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    operational_error = (not record.get('post_exit_zero_observed')
                         or 'Exact spawned Isaac process did not stop' in (record.get('error') or '')
                         or record.get('controller_returncode') not in (0, 1))
    passed = c_report['passed'] and not operational_error and record.get('controller_returncode') == 0
    record.update({
        'status': 'PASS' if passed else 'FAIL', 'c_assessment': c_report,
        'valid_control_failure': not passed and c_report.get('valid_control_trajectory', False) and not operational_error,
        'b_runner_error': record.get('error'),
        'error': None if passed else '; '.join(c_report['issues']) or record.get('error') or 'Operational failure',
    })
    (Path(record['attempt_dir']) / 'c_runner_result.json').write_text(
        json.dumps(record, ensure_ascii=False, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    return record


def _validate_run_path(path, *, exists):
    path = path.resolve(strict=exists)
    if path.parent != C_ROOT.resolve():
        raise ValueError(f'C output must be a direct child of {C_ROOT}')
    return path


def _new_manifest(path=None):
    C_ROOT.mkdir(parents=True, exist_ok=True)
    run = path or C_ROOT / (datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10])
    run = _validate_run_path(run, exists=False)
    run.mkdir(exist_ok=False)
    manifest = {'stage': 'C', 'version': VERSION, 'created_utc': base.utc_now(),
                'fingerprints': fingerprints(), 'prefix_cases': [asdict(case) for case in PREFIX_CASES],
                'selected_position_case': None, 'direction_selection': None,
                'results': {}, 'status': 'IN_PROGRESS'}
    base.atomic_json(run / 'manifest.json', manifest)
    return run, manifest


def _resume_manifest(path):
    run = _validate_run_path(path, exists=True)
    manifest = json.loads((run / 'manifest.json').read_text(encoding='utf-8'))
    if (manifest.get('stage') != 'C' or manifest.get('version') != VERSION
            or manifest.get('prefix_cases') != [asdict(case) for case in PREFIX_CASES]):
        raise ValueError('Saved plan is not the fixed C plan')
    return run, manifest


def _write_table(run, manifest, cases):
    rows = ['# BalanceHold C trials', '',
            '| Case | Hold m | Initial angle deg | Status | Reason | Evidence |',
            '| --- | ---: | ---: | --- | --- | --- |']
    for case in cases:
        record = _latest_record(manifest, case)
        assessment = record.get('c_assessment', {}) if record else {}
        rows.append(f'| {case.name} | {case.hold_position:+.1f} | {math.degrees(case.theta0):+.0f} | '
                    f'{record["status"] if record else "NOT RUN"} | {assessment.get("end_reason", "")} | '
                    f'{record.get("trial_dir", "") if record else ""} |')
    (run / 'results.md').write_text('\n'.join(rows) + '\n', encoding='utf-8')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--run-dir', type=Path, help='New direct child of the C output root')
    group.add_argument('--resume', type=Path, metavar='RUN_DIR')
    parser.add_argument('--max-cases', type=int, choices=range(1, 7), default=6,
                        help='Limit new attempts in this invocation; resume continues the remaining plan')
    parser.add_argument('--retry-failed', action='store_true', help='Explicitly repeat saved control failures after a targeted correction')
    parser.add_argument('--list-plan', action='store_true')
    args = parser.parse_args(argv)
    if args.list_plan:
        for case in PREFIX_CASES:
            print(json.dumps(asdict(case), allow_nan=False))
        print('Sixth: hold=+0.3 m, x0=+0.3 m, angle=5 degrees in the harder observed direction')
        return 0
    run, manifest = _resume_manifest(args.resume) if args.resume else _new_manifest(args.run_dir)
    print(f'C_OUTPUT={run}', flush=True)
    if manifest['fingerprints'] != fingerprints():
        print('WARNING: fingerprints changed; completed trials are checked against raw evidence', flush=True)
    cases = list(PREFIX_CASES)
    attempted = 0
    index = 0
    while index < 6:
        if index == 5:
            case, explanation = choose_position_case(manifest)
            manifest['selected_position_case'] = asdict(case)
            manifest['direction_selection'] = explanation
            cases.append(case)
            base.atomic_json(run / 'manifest.json', manifest)
        case = cases[index]
        completed = _verified_completed(manifest, case)
        if completed and (completed['status'] == 'PASS' or not args.retry_failed):
            print(f'SKIP verified {completed["status"]} {case.name}', flush=True)
            index += 1
            continue
        if attempted >= args.max_cases:
            manifest['status'] = 'IN_PROGRESS'
            break
        print(f'START {case.name}: hold={case.hold_position:+.3f}, theta0={math.degrees(case.theta0):+.1f} degrees', flush=True)
        record = run_case(run, case)
        record['fingerprints'] = fingerprints()
        manifest['results'].setdefault(case.name, []).append(record)
        attempted += 1
        manifest['status'] = 'IN_PROGRESS'
        if record['status'] != 'PASS' and not record.get('valid_control_failure'):
            manifest['status'] = 'BLOCKED_OPERATIONAL_FAILURE'
        base.atomic_json(run / 'manifest.json', manifest)
        _write_table(run, manifest, cases)
        print(f'{record["status"]} {case.name}: {record.get("error") or record["trial_dir"]}', flush=True)
        if manifest['status'] == 'BLOCKED_OPERATIONAL_FAILURE':
            return 2
        index += 1
    if index == 6:
        completed = [_verified_completed(manifest, case) for case in cases]
        manifest['status'] = ('PASS' if all(record and record['status'] == 'PASS' for record in completed)
                              else 'COMPLETE_WITH_CONTROL_FAILURES' if all(completed) else 'INCOMPLETE')
        manifest['finished_utc'] = base.utc_now()
    base.atomic_json(run / 'manifest.json', manifest)
    _write_table(run, manifest, cases)
    print(f'C_STATUS={manifest["status"]}', flush=True)
    return 0 if manifest['status'] in ('PASS', 'IN_PROGRESS') else 1


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError) as exc:
        print(f'C RUN ERROR: {exc}', file=sys.stderr)
        raise SystemExit(2) from exc
