"""Current runner contracts. No simulator, ROS process or shared-data writes."""

import json
import math
from pathlib import Path
import sys

import pytest


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
import run_cartpole_trial as runner  # noqa: E402


def test_gui_plan_and_read_only_plan_are_separate():
    assert [(case.name, case.task, case.target) for case in runner.CASES] == [
        ('M1', 'move', 0.3), ('M2', 'move', -0.3), ('M3', 'move_then_hold', 0.5)]
    assert runner.CASES[-1].theta0 == math.radians(2)
    assert all(case.x0 == case.hold_position == 0 for case in runner.CASES)
    assert all(case.theta0 == 0 and case.task == 'move' for case in runner.DRY_CASES)
    assert runner.MAX_ISAAC_WALL_SEC == 600


def test_gui_is_default_and_hold_does_not_receive_move_handshake_fields():
    gui = runner.launcher_command(runner.CASES[0])
    assert '--headless' not in gui and '--gui' not in gui
    assert '--trial-task' in gui and '--trial-target' in gui
    hold = runner.launcher_command(runner.Case('hold', 'hold', 0.0), headless=True)
    assert '--headless' in hold
    assert '--trial-target' not in hold and '--trial-task' not in hold


def test_system_ros_launch_quotes_overlay_and_selects_read_only_without_publishing(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'ROOT', tmp_path / 'project with space')
    overlay = runner.ROOT / 'ros2_ws/install/setup.bash'
    overlay.parent.mkdir(parents=True)
    overlay.write_text('')
    command = runner._system_ros_command(tmp_path/'isaac', tmp_path/'trial', runner.DRY_CASES[0],
                                         dry_run=True, reference_speed=0.05)
    assert str(overlay) in command
    assert '--dry-run' in command
    shell = command[command.index('-c') + 1]
    assert 'source "$1"' in shell and shell.index('source /opt/ros') < shell.index('set -u')
    assert '--target' in command


def test_unknown_spawned_child_is_not_reported_as_clean(monkeypatch):
    class Launcher:
        pid = 123
        returncode = None

        def poll(self):
            return self.returncode

        def terminate(self):
            self.returncode = -15

        def wait(self, timeout):
            return self.returncode

    monkeypatch.setattr(runner, '_launcher_children', lambda launcher: {456: {'start_ticks': 789, 'state': 'S'}})
    monkeypatch.setattr(runner, '_remember_child', lambda launcher, isaac_dir=None: None)
    monkeypatch.setattr(runner, '_proc_data', lambda pid: None)
    assert not runner._stop_isaac(Launcher(), None, Path('/tmp/trial'))


def test_cli_lists_cases_without_starting_simulator(capsys):
    assert runner.main(['--list-plan']) == 0
    plan = json.loads(capsys.readouterr().out)
    assert plan['mode'] == 'gui' and len(plan['cases']) == 3
    assert runner.main(['--mode', 'dry-run', '--list-plan']) == 0
    assert json.loads(capsys.readouterr().out)['cases'][2]['theta0'] == 0


def test_runner_stops_after_first_gui_failure(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'RUN_ROOT', tmp_path)
    monkeypatch.setattr(runner, 'fingerprints', lambda: {})
    launched = []

    def fake_session(run_dir, cases, **kwargs):
        launched.extend(case.name for case in cases)
        return [{'case': runner.asdict(cases[0]), 'status': 'FAIL', 'error': 'bounded failure'}]

    monkeypatch.setattr(runner, 'run_session', fake_session)
    assert runner.main(['--keep-gui-seconds', '0']) == 1
    assert launched == ['M1']
    manifest = json.loads(next(tmp_path.glob('*/manifest.json')).read_text())
    assert manifest['status'] == 'FAIL'


def test_read_only_uses_one_session_for_three_nodes(tmp_path, monkeypatch):
    monkeypatch.setattr(runner, 'RUN_ROOT', tmp_path)
    monkeypatch.setattr(runner, 'fingerprints', lambda: {})
    sessions = []

    def fake_session(run_dir, cases, **kwargs):
        sessions.append((cases, kwargs))
        return [{'case': runner.asdict(case), 'status': 'PASS'} for case in cases]

    monkeypatch.setattr(runner, 'run_session', fake_session)
    assert runner.main(['--mode', 'dry-run']) == 0
    assert len(sessions) == 1 and len(sessions[0][0]) == 3
    assert sessions[0][1]['dry_run'] is True
