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
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

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


class PhysicsLoggingContracts(unittest.TestCase):
    """The extra observer reads actual post-step data without advancing physics."""

    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location(
            'run_ros_cartpole_physics_contract', ROOT / 'sim/scripts/run_ros_cartpole.py')
        cls.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.module)

    def make_runtime(self):
        runtime = self.module.CartpoleROS.__new__(self.module.CartpoleROS)
        runtime.closed = False
        runtime.keep_running = True
        runtime.physics_log_error = None
        runtime.physics_callback_id = None
        runtime.physics_sample_count = 0
        runtime.physics_samples = io.StringIO()
        runtime.events = io.StringIO()
        runtime.samples = io.StringIO()
        runtime.steps, runtime.resets = 7, 0
        runtime.path = '/World/ObservedGraph'
        runtime.run = Path('/not-created/readonly-test')
        runtime.names = ['slider_to_cart', 'cart_to_pole']
        runtime.cart, runtime.pole = 0, 1
        runtime.base = SimpleNamespace(cfg=SimpleNamespace(max_cart_pos=3.0))
        runtime.recorded_events = []
        runtime.event = lambda name, **values: runtime.recorded_events.append((name, values))
        runtime.live = {'position': [0.12, 0.03], 'velocity': [0.04, -0.05],
                        'time': 0.1, 'manager_time': 0.1, 'physics_step': 12}
        runtime.articulation = SimpleNamespace(
            get_joint_positions=lambda: SimpleNamespace(tolist=lambda: runtime.live['position']),
            get_joint_velocities=lambda: SimpleNamespace(tolist=lambda: runtime.live['velocity']))
        runtime.physics_manager = SimpleNamespace(
            get_simulation_time=lambda: runtime.live['manager_time'],
            get_num_physics_steps=lambda: runtime.live['physics_step'])

        def read_graph(attribute):
            if attribute.endswith('/Time.outputs:simulationTime'):
                return runtime.live['time']
            if attribute.endswith('/Subscribe.outputs:effortCommand'):
                return [0.5, 0.0]
            if attribute.endswith('/Controller.inputs:effortCommand'):
                return [0.5, 0.0]
            self.fail('Unexpected graph query: ' + attribute)

        runtime.og = SimpleNamespace(Controller=SimpleNamespace(get=read_graph))
        return runtime

    def test_each_native_substep_records_live_state_and_unmodified_time(self):
        runtime = self.make_runtime()
        runtime._record_physics_sample(1 / 120)
        runtime.live.update(position=[0.121, 0.031], velocity=[0.06, -0.07],
                            time=0.1 + 1 / 120, manager_time=0.1 + 1 / 120, physics_step=13)
        runtime._record_physics_sample(1 / 120)
        rows = [json.loads(line) for line in runtime.physics_samples.getvalue().splitlines()]
        self.assertEqual(len(rows), 2)
        self.assertEqual([row['outer_loop_step'] for row in rows], [7, 7])
        self.assertEqual([row['physics_step'] for row in rows], [12, 13])
        self.assertEqual(rows[0]['simulation_time'], 0.1)
        self.assertEqual(rows[1]['simulation_time'], runtime.live['time'])
        self.assertEqual(rows[1]['position'], [0.121, 0.031])
        self.assertEqual(rows[1]['velocity'], [0.06, -0.07])
        self.assertEqual(rows[1]['controller_effort'], [0.5, 0.0])
        self.assertEqual(rows[1]['callback_delta_sim_sec'], 1 / 120)
        self.assertEqual(rows[1]['logging_source'], 'physx_post_physics_step_single_articulation')
        self.assertIsNone(rows[1]['computed_effort'])
        self.assertIsNone(rows[1]['applied_effort'])
        self.assertTrue(runtime.keep_running)
        # This fake has no step(), scene refresh, reset or effort setters.

    def test_invalid_measurement_stops_without_writing_false_evidence(self):
        for field, value, dt in [('position', [float('nan'), 0.0], 1 / 120),
                                 ('velocity', [0.0], 1 / 120),
                                 ('time', float('inf'), 1 / 120),
                                 ('position', [0.0, 0.0], 0.0)]:
            with self.subTest(field=field, value=value, dt=dt):
                runtime = self.make_runtime()
                runtime.live[field] = value
                runtime._record_physics_sample(dt)
                self.assertFalse(runtime.keep_running)
                self.assertIsNotNone(runtime.physics_log_error)
                self.assertEqual(runtime.physics_samples.getvalue(), '')
                self.assertEqual(runtime.recorded_events[0][0], 'physics_log_failed')
                runtime.live.update(position=[0.0, 0.0], velocity=[0.0, 0.0], time=0.0)
                runtime._record_physics_sample(1 / 120)
                self.assertEqual(runtime.physics_samples.getvalue(), '')

    def test_registration_uses_post_step_and_retains_zero_callback_id(self):
        runtime = self.make_runtime()
        registered, removed = [], []
        manager = SimpleNamespace(
            register_callback=lambda callback, **values: registered.append((callback, values)) or 0,
            deregister_callback=lambda callback_id: removed.append(callback_id))
        module = ModuleType('isaacsim.core.simulation_manager')
        module.SimulationManager = manager
        module.IsaacEvents = SimpleNamespace(POST_PHYSICS_STEP='POST_PHYSICS_STEP')
        with patch.dict(sys.modules, {'isaacsim.core.simulation_manager': module}):
            runtime._register_physics_logger()
        self.assertEqual(registered[0][1]['event'], 'POST_PHYSICS_STEP')
        self.assertEqual(registered[0][1]['order'], 1000)
        self.assertEqual(runtime.physics_callback_id, 0)
        runtime._unregister_physics_logger()
        self.assertEqual(removed, [0])
        self.assertIsNone(runtime.physics_callback_id)

    def test_close_unregisters_before_zero_and_closes_even_on_unregister_error(self):
        for fail_removal in (False, True):
            with self.subTest(fail_removal=fail_removal):
                runtime = self.make_runtime()
                calls = []

                def remove(callback_id):
                    calls.append(('deregister', callback_id))
                    if fail_removal:
                        raise RuntimeError('Diagnostic removal failure')

                runtime.physics_manager.deregister_callback = remove
                runtime.physics_callback_id = 17
                runtime.zero_effort = lambda: calls.append(('zero',))
                runtime.env = SimpleNamespace(close=lambda: calls.append(('env_close',)))
                runtime.app = SimpleNamespace(close=lambda: calls.append(('app_close',)))
                runtime.close()
                self.assertEqual(calls, [('deregister', 17), ('zero',), ('env_close',), ('app_close',)])
                self.assertTrue(runtime.physics_samples.closed)
                self.assertTrue(runtime.events.closed)
                self.assertTrue(runtime.samples.closed)
                self.assertEqual(runtime.physics_log_error is not None, fail_removal)
                runtime.close()
                self.assertEqual(len(calls), 4)


if __name__ == '__main__':
    sys.exit(0 if unittest.main(verbosity=2, exit=False).result.wasSuccessful() else 1)
