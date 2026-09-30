"""C plan and resume checks without starting any simulator or ROS process."""

from dataclasses import asdict
import json
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_balance_hold_c as runner  # noqa: E402


def test_only_five_fixed_prefix_cases_and_one_adaptive_position_case():
    cases = runner.PREFIX_CASES
    assert len(cases) == 5
    assert [round(math.degrees(case.theta0)) for case in cases] == [2, -3, 3, -5, 5]
    assert all(case.hold_position == case.x0 == 0 for case in cases)
    assert runner.base.FORCE_LIMIT_N == 5
    assert runner.base.MAX_ISAAC_WALL_SEC == 600


@pytest.mark.parametrize('minus_status,plus_status,minus_time,plus_time,expected_sign', [
    ('PASS', 'PASS', 1.0, 1.2, 1),
    ('PASS', 'PASS', 1.2, 1.0, -1),
    ('FAIL', 'PASS', 0.5, 2.0, -1),
    ('FAIL', 'FAIL', 0.6, 0.7, 1),
    ('PASS', 'PASS', 1.0, 1.0, -1),
])
def test_nonzero_position_uses_measured_harder_direction(monkeypatch, minus_status, plus_status,
                                                         minus_time, plus_time, expected_sign):
    records = {}
    for case, status, seconds in zip(runner.PREFIX_CASES[-2:], (minus_status, plus_status),
                                     (minus_time, plus_time)):
        records[case.name] = {'case': asdict(case), 'status': status,
                              'c_assessment': {'recovery_time_sim_sec': seconds, 'elapsed_sim_sec': seconds}}
    monkeypatch.setattr(runner, '_verified_completed', lambda manifest, case: records[case.name])
    case, explanation = runner.choose_position_case({})
    assert case.hold_position == case.x0 == 0.3
    assert math.degrees(case.theta0) == pytest.approx(expected_sign * 5)
    assert explanation['selected_source_case'] in records


def test_new_manifest_does_not_overwrite_and_resume_requires_c_plan(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'C_ROOT', tmp_path / 'c runs')
    monkeypatch.setattr(runner, 'fingerprints', lambda: {'node': 'sha'})
    target = runner.C_ROOT / 'test run'
    path, manifest = runner._new_manifest(target)
    assert path == target
    assert runner._resume_manifest(path)[1] == manifest
    with pytest.raises(FileExistsError):
        runner._new_manifest(target)
    with pytest.raises(ValueError, match='direct child'):
        runner._new_manifest(tmp_path / 'outside')
    manifest['prefix_cases'][0]['theta0'] = math.radians(1)
    runner.base.atomic_json(path / 'manifest.json', manifest)
    with pytest.raises(ValueError, match='fixed C plan'):
        runner._resume_manifest(path)


def test_resume_rechecks_saved_pass_from_raw_evidence(tmp_path, monkeypatch):
    case = runner.PREFIX_CASES[0]
    config = {'x0': case.x0, 'theta0': case.theta0, 'hold_position': case.hold_position,
              'force_limit_n': 5.0, 'max_sim_sec': 10.0, 'stable_window_sim_sec': 3.0}
    (tmp_path / 'config.json').write_text(json.dumps(config))
    (tmp_path / 'c_assessment.json').write_text(json.dumps({'passed': True, 'trial_id': 'trial'}))
    monkeypatch.setattr(runner, 'assess_trial', lambda path: {'passed': True, 'trial_id': 'trial'})
    assert runner.verified_pass(tmp_path, case)
    monkeypatch.setattr(runner, 'assess_trial', lambda path: {'passed': False, 'trial_id': 'trial'})
    assert runner.verified_pass(tmp_path, case) is None


def test_failing_operational_attempt_stops_plan(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'C_ROOT', tmp_path / 'c')
    monkeypatch.setattr(runner, 'fingerprints', lambda: {'node': 'sha'})
    calls = []

    def fail(run, case):
        calls.append(case)
        return {'case': asdict(case), 'status': 'FAIL', 'valid_control_failure': False,
                'trial_dir': '/tmp/not-run', 'error': 'No valid trajectory'}

    monkeypatch.setattr(runner, 'run_case', fail)
    assert runner.main(['--run-dir', str(runner.C_ROOT / 'run')]) == 2
    assert calls == [runner.PREFIX_CASES[0]]
    manifest = json.loads((runner.C_ROOT / 'run/manifest.json').read_text())
    assert manifest['status'] == 'BLOCKED_OPERATIONAL_FAILURE'


def test_bounded_control_failure_can_continue_without_automatic_retuning(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'C_ROOT', tmp_path / 'c')
    monkeypatch.setattr(runner, 'fingerprints', lambda: {'node': 'sha'})
    calls = []

    def controlled_failure(run, case):
        calls.append(case)
        return {'case': asdict(case), 'status': 'FAIL', 'valid_control_failure': True,
                'trial_dir': '/tmp/not-run', 'error': 'SATURATION_TIMEOUT'}

    monkeypatch.setattr(runner, 'run_case', controlled_failure)
    assert runner.main(['--run-dir', str(runner.C_ROOT / 'run'), '--max-cases', '2']) == 0
    assert calls == list(runner.PREFIX_CASES[:2])
    manifest = json.loads((runner.C_ROOT / 'run/manifest.json').read_text())
    assert manifest['status'] == 'IN_PROGRESS'
    assert len(manifest['results']) == 2
