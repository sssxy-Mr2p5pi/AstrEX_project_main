"""Pure matrix-runner tests. No Isaac, ROS process, or shared-data write."""

import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_balance_hold_matrix as matrix  # noqa: E402


def test_fixed_case_order_and_limits():
    cases = matrix.CASES
    assert len(cases) == 16
    assert len({case.name for case in cases}) == 16
    assert [case.batch for case in cases] == [1] * 6 + [2] * 6 + [3] * 4
    assert cases[0].name == 'batch1_hold_m0p3_theta_m1'
    assert cases[1].name == 'batch1_hold_m0p3_theta_p1'
    assert cases[2].name == 'batch1_hold_0_theta_m1'
    assert all(abs(case.theta0) in (math.radians(1), math.radians(2)) for case in cases)
    assert all(abs(case.x0 - case.hold_position) <= 0.05 for case in cases)
    assert matrix.FORCE_LIMIT_N == 5.0
    assert matrix.MAX_ISAAC_WALL_SEC == 600.0


def test_completed_trials_must_be_a_verified_prefix(tmp_path, monkeypatch):
    first = tmp_path / 'first'
    second = tmp_path / 'second'
    first.mkdir()
    second.mkdir()
    accepted = {str(first): matrix.CASES[0].name,
                str(second): matrix.CASES[1].name}
    monkeypatch.setattr(matrix, 'MATRIX_ROOT', tmp_path / 'matrix')
    monkeypatch.setattr(matrix, 'fingerprints', lambda: {'script': 'sha'})
    monkeypatch.setattr(matrix, 'verified_pass', lambda path, case: {'passed': True}
                        if accepted.get(str(path)) == case.name else None)
    # Fake config is enough here: verified_pass owns raw-evidence validation.
    for path in (first, second):
        (path / 'config.json').write_text('{"isaac_run_dir": "/tmp/example"}')
    with pytest.raises(ValueError, match='not a verified PASS'):
        matrix._new_manifest([second])
    run_dir, manifest = matrix._new_manifest([first, second])
    assert run_dir.is_dir()
    assert len(manifest['results']) == 2
    assert matrix.CASES[2].name not in manifest['results']


def test_resume_rechecks_raw_evidence_even_if_runner_fingerprint_changed(monkeypatch):
    case = matrix.CASES[0]
    saved = {'results': {case.name: [{
        'case': matrix.asdict(case), 'status': 'PASS',
        'trial_dir': '/tmp/prior-trial', 'fingerprints': {'runner': 'old'},
    }]}, 'fingerprints': {'runner': 'old'}}
    monkeypatch.setattr(matrix, 'verified_pass', lambda path, candidate: {'passed': True})
    assert matrix._can_skip(saved, case)
    monkeypatch.setattr(matrix, 'verified_pass', lambda path, candidate: None)
    assert not matrix._can_skip(saved, case)


def test_unknown_spawned_child_cannot_be_reported_as_clean(monkeypatch):
    class Launcher:
        pid = 123
        returncode = None

        def poll(self):
            return self.returncode

        def terminate(self):
            self.returncode = -15

        def wait(self, timeout):
            return self.returncode

    monkeypatch.setattr(matrix, '_launcher_children', lambda launcher: {456: {
        'start_ticks': 789, 'state': 'S',
    }})
    monkeypatch.setattr(matrix, '_remember_child', lambda launcher, isaac_dir=None: None)
    monkeypatch.setattr(matrix, '_proc_data', lambda pid: None)
    assert not matrix._stop_isaac(Launcher(), None, Path('/tmp/trial'))
