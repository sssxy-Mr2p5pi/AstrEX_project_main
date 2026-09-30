"""Launcher contract tests: fast, and no Isaac Sim process is ever started.

Run with system Python 3 (works in or outside the sandbox, but the lock contract needs a
machine where the health check passes, i.e. the GPU is visible):

    /usr/bin/python3 -B tests/isaac/test_launcher_contracts.py

Contracts covered (consolidated from the earlier per-case launcher tests):
  1. target environment selection: contaminated shell and paths with spaces still resolve
     to the configured environment;
  2. RL ``--print-config`` defaults and explicit argument pass-through;
  3. ROS ``--print-config`` defaults, and printing never creates a run directory;
  4. invalid overrides and legacy callers are rejected; ``--help`` starts nothing;
  5. RL and ROS share one run lock: while it is held the entry must fail after the health
     check and before any Sim process or run directory appears.
"""

import fcntl
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / 'config/isaac_baseline.env'
ENTRIES = ('check_isaac_env.sh', 'start_isaac_rl.sh', 'start_isaac_ros.sh')


def config_value(key, default=None):
    for line in CONFIG.read_text().splitlines():
        line = line.strip()
        if line.startswith(key + '='):
            return line.split('=', 1)[1].strip().strip("'\"")
    return default


def lock_path(root=ROOT):
    """Must match the derivation inside scripts/lib/isaac_entry.py."""
    key = hashlib.sha256(str(root).encode()).hexdigest()[:16]
    return Path(tempfile.gettempdir()) / f'astrex-isaac-{os.getuid()}-{key}.lock'


class LauncherContracts(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.output_root = Path(config_value('ASTREX_ISAAC_OUTPUT_ROOT', '/nonexistent'))
        cls.configured_env = config_value('ASTREX_ISAAC_CONDA_ENV')

    # ------------------------------------------------------------------ helpers
    def run_entry(self, script, args=(), extra=None, root=ROOT, timeout=180):
        env = {'HOME': os.environ.get('HOME', '/home/sssxy'), 'PATH': os.environ.get('PATH', '/usr/bin:/bin')}
        env.update(extra or {})
        return subprocess.run(['bash', str(root / 'scripts' / script), *args],
                              env=env, text=True, capture_output=True, timeout=timeout)

    def print_config(self, script, args=(), extra=None, root=ROOT):
        result = self.run_entry(script, ['--print-config', *args], extra, root)
        self.assertEqual(result.returncode, 0, result.stderr or result.stdout)
        return json.loads(result.stdout)

    def run_dirs(self, profile):
        folder = self.output_root / profile
        if not folder.is_dir():
            return None
        return sorted(entry.name for entry in folder.iterdir())

    # ------------------------------------------------- 1. target environment selection
    def test_target_environment_selection(self):
        contamination = {'PYTHONPATH': '/invalid/ros/python', 'LD_LIBRARY_PATH': '/invalid/ros/lib',
                         'AMENT_PREFIX_PATH': '/invalid/ros'}
        for caller in ('base', 'isaaclab60', 'isaaclab60_6010test'):
            with self.subTest(caller=caller):
                value = self.print_config('start_isaac_ros.sh',
                                          extra=dict(contamination, CONDA_DEFAULT_ENV=caller))
                self.assertEqual(value['environment'], self.configured_env)
                self.assertEqual(value['caller'], caller)
                self.assertFalse(value['arguments']['headless'])
                rl = self.print_config('start_isaac_rl.sh',
                                       extra=dict(contamination, CONDA_DEFAULT_ENV=caller))
                self.assertEqual(rl['environment'], self.configured_env)

    def test_target_environment_selection_from_path_with_spaces(self):
        with tempfile.TemporaryDirectory(prefix='astrex launcher contracts ') as folder:
            alias = Path(folder) / 'AstrEX project with spaces'
            alias.symlink_to(ROOT, target_is_directory=True)
            value = self.print_config('start_isaac_ros.sh', root=alias)
            self.assertEqual(value['environment'], self.configured_env)

    # ------------------------------------------------------------- 2. RL print-config
    def test_rl_print_config_defaults(self):
        value = self.print_config('start_isaac_rl.sh')
        arguments = value['arguments']
        self.assertEqual(value['environment'], self.configured_env)
        self.assertEqual(arguments['task'], 'Isaac-Cartpole-Direct-v0')
        self.assertEqual(arguments['num_envs'], 16)
        self.assertEqual(arguments['max_iterations'], 20)
        self.assertEqual(arguments['seed'], 42)
        self.assertEqual(arguments['device'], 'cuda:0')
        self.assertFalse(arguments['gui'])
        self.assertFalse(arguments['headless'])

    def test_rl_print_config_explicit_arguments(self):
        value = self.print_config('start_isaac_rl.sh',
                                  ['--task', 'Future-Task-v0', '--num_envs', '128', '--max_iterations', '40'])
        arguments = value['arguments']
        self.assertEqual(arguments['task'], 'Future-Task-v0')
        self.assertEqual(arguments['num_envs'], 128)
        self.assertEqual(arguments['max_iterations'], 40)

    # ------------------------------------------------------------ 3. ROS print-config
    def test_ros_print_config_defaults_and_side_effects(self):
        before = self.run_dirs('ros')
        value = self.print_config('start_isaac_ros.sh')
        self.assertEqual(value['profile'], 'ros')
        self.assertFalse(value['arguments']['headless'])
        self.assertEqual(value['arguments']['trial_task'], 'hold')
        self.assertIsNone(value['arguments']['trial_target'])
        self.assertEqual(value['arguments']['trial_reference_speed'], 0.05)
        self.assertEqual(self.run_dirs('ros'), before)

    def test_hold_trial_retains_existing_options(self):
        value = self.print_config('start_isaac_ros.sh',
                                  ['--trial-x0', '0.3', '--trial-theta0', '0.03',
                                   '--trial-hold-position', '0.3'])
        self.assertEqual(value['arguments']['trial_task'], 'hold')
        self.assertEqual(value['arguments']['trial_hold_position'], 0.3)
        self.assertIsNone(value['arguments']['trial_target'])

    def test_moving_trial_keeps_initial_hold_separate_from_final_target(self):
        before = self.run_dirs('ros')
        for task, target, speed in [('move', -0.3, 0.05), ('move_then_hold', 0.5, 0.03)]:
            with self.subTest(task=task):
                value = self.print_config('start_isaac_ros.sh',
                                          ['--trial-x0', '0', '--trial-theta0', '0.034906585',
                                           '--trial-hold-position', '0', '--trial-task', task,
                                           '--trial-target', str(target),
                                           '--trial-reference-speed', str(speed)])
                self.assertEqual(value['arguments']['trial_task'], task)
                self.assertEqual(value['arguments']['trial_target'], target)
                self.assertEqual(value['arguments']['trial_hold_position'], 0)
                self.assertEqual(value['arguments']['trial_reference_speed'], speed)
                self.assertFalse(value['arguments']['headless'])
        self.assertEqual(self.run_dirs('ros'), before)

    def test_invalid_moving_trial_options_are_rejected(self):
        initial = ['--trial-x0', '0', '--trial-theta0', '0', '--trial-hold-position', '0']
        cases = [(['--trial-task', 'move', '--trial-target', '0.5']),
                 (initial + ['--trial-task', 'move']),
                 (initial + ['--trial-task', 'move', '--trial-target', 'nan']),
                 (initial + ['--trial-task', 'move', '--trial-target', 'inf']),
                 (initial + ['--trial-target', '0.5']),
                 (['--trial-x0', '0', '--trial-theta0', '0', '--trial-hold-position', '0.5',
                   '--trial-task', 'move', '--trial-target', '0.5'])]
        cases += [initial + ['--trial-task', 'move', '--trial-target', '0.5',
                             '--trial-reference-speed', value]
                  for value in ('nan', 'inf', '0', '-0.03', '0.051')]
        for args in cases:
            with self.subTest(args=args):
                result = self.run_entry('start_isaac_ros.sh', ['--print-config', *args])
                self.assertNotEqual(result.returncode, 0, args)

    # ---------------------------------------------- 4. invalid and legacy rejection
    def test_invalid_overrides_are_rejected(self):
        cases = (['--device', 'cpu'], ['--device=cuda:1'], ['--distributed'], ['env.sim.device=cpu'],
                 ['agent.device=cpu'], ['--experience', '/tmp/other'], ['--num_envs', '0'],
                 ['--max_iterations', '0'])
        for args in cases:
            with self.subTest(args=args):
                result = self.run_entry('start_isaac_rl.sh', ['--print-config', *args])
                self.assertNotEqual(result.returncode, 0, args)

    def test_legacy_caller_is_refused(self):
        for caller in ('isaaclab60', 'isaaclab60_6010test'):
            with self.subTest(caller=caller):
                result = self.run_entry('check_isaac_env.sh', extra={'CONDA_DEFAULT_ENV': caller})
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('Legacy caller', result.stderr or result.stdout)

    def test_help_starts_nothing(self):
        before = {profile: self.run_dirs(profile) for profile in ('rl', 'ros')}
        for script in ENTRIES:
            with self.subTest(script=script):
                self.assertEqual(self.run_entry(script, ['--help']).returncode, 0)
        for profile, previous in before.items():
            self.assertEqual(self.run_dirs(profile), previous)

    # --------------------------------------------------------- 5. run lock exclusivity
    def test_run_lock_is_exclusive(self):
        before = self.run_dirs('ros')
        path = lock_path()
        self.assertTrue(path.parent.is_dir(), path)
        handle = os.open(path, os.O_CREAT | os.O_RDWR, 0o600)
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.run_entry('start_isaac_ros.sh', ['--headless'])
            output = (result.stdout or '') + (result.stderr or '')
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('Another AstrEX Isaac launcher is running', output)
            self.assertEqual(self.run_dirs('ros'), before, 'the lock must be taken before a run directory is created')
        finally:
            fcntl.flock(handle, fcntl.LOCK_UN)
            os.close(handle)


class TrialHandshakeContracts(unittest.TestCase):
    """Validate request identity without creating a SimulationApp or physics scene."""

    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'run_ros_cartpole_contract', ROOT / 'sim/scripts/run_ros_cartpole.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def check_handshake(self, request, marker, should_accept):
        with tempfile.TemporaryDirectory(prefix='astrex trial handshake ') as folder:
            runtime = self.module.CartpoleROS.__new__(self.module.CartpoleROS)
            runtime.run = Path(folder)
            runtime.trial = request
            runtime.trial_initialized = False
            runtime.trial_ready_path = runtime.run / 'trial_ready.json'
            payload = dict(marker, trial_id=runtime.run.name, controller_pid=os.getpid())
            runtime.trial_ready_path.write_text(json.dumps(payload))

            class ReachedInitialization(Exception):
                pass

            def stop_before_physics():
                raise ReachedInitialization()

            runtime.hold_trial_rest = stop_before_physics
            expected = ReachedInitialization if should_accept else RuntimeError
            with self.assertRaises(expected):
                runtime.maybe_initialize_trial()

    def test_hold_preserves_exact_legacy_schema(self):
        request = {'x0': 0, 'theta0': 0.02, 'hold_position': 0}
        self.check_handshake(request, request, True)
        self.check_handshake(request, dict(request, task='hold'), False)

    def test_move_configuration_must_match_launch_request(self):
        for task in ('move', 'move_then_hold'):
            request = {'x0': 0, 'theta0': 0.02, 'hold_position': 0,
                       'task': task, 'target': 0.5, 'reference_speed_mps': 0.05}
            self.check_handshake(request, request, True)
            for update in ({'target': 0.3}, {'reference_speed_mps': 0.03}, {'task': 'hold'},
                           {'target': True}, {'reference_speed_mps': float('nan')}):
                with self.subTest(task=task, update=update):
                    self.check_handshake(request, dict(request, **update), False)
            legacy_marker = {key: request[key] for key in ('x0', 'theta0', 'hold_position')}
            self.check_handshake(request, legacy_marker, False)

    def test_final_target_uses_real_scene_and_joint_limits(self):
        request = {'x0': 0.0, 'theta0': 0.0, 'hold_position': 0.0,
                   'task': 'move_then_hold', 'target': 0.5, 'reference_speed_mps': 0.05}
        self.module.validate_trial_limits(request, [-4.0, 4.0], 3.0)
        for limits, scene_limit, update in [([-4.0, 4.0], 3.0, {'target': 3.01}),
                                          ([-0.4, 0.4], 3.0, {}),
                                          ([-4.0, 4.0], 3.0, {'x0': -4.01}),
                                          ([-4.0, 4.0], 3.0, {'hold_position': float('nan')}),
                                          ([-4.0, 4.0], 3.0, {'target': True}),
                                          ([-4.0, 4.0], 0.0, {}),
                                          ([float('nan'), 4.0], 3.0, {})]:
            with self.subTest(limits=limits, scene_limit=scene_limit, update=update):
                with self.assertRaises(RuntimeError):
                    self.module.validate_trial_limits(dict(request, **update), limits, scene_limit)

    def test_project_camera_frames_the_requested_travel(self):
        self.assertEqual(self.module.trial_viewer(None, (0.0, 0.0, 2.0)),
                         {'eye': (4.0, 0.0, 3.3), 'lookat': (0.0, 0.0, 2.5)})
        request = {'x0': 0.0, 'hold_position': 0.0, 'target': 0.5}
        view = self.module.trial_viewer(request, (0.0, 0.0, 2.0))
        self.assertEqual(view['eye'], (4.0, 0.25, 3.3))
        self.assertEqual(view['lookat'], (0.0, 0.25, 2.5))
        # Viewing metadata is not evidence that a person actually saw the GUI.


if __name__ == '__main__':
    sys.exit(0 if unittest.main(verbosity=2, exit=False).result.wasSuccessful() else 1)
