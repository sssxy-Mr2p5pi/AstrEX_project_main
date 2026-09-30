"""Official OmniGraph ROS Cartpole profile. No direct-rclpy Sim control path."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import time
import traceback
import uuid

GRAPH_PREFIX = '/World/AstrEXROSGraph_'
TICK_STEP_OUT = '/Tick.outputs:step'
SUBSCRIBE_EXEC_IN = '/Subscribe.inputs:execIn'
SUBSCRIBE_EXEC_OUT = '/Subscribe.outputs:execOut'
CONTROLLER_EXEC_IN = '/Controller.inputs:execIn'


def json_safe(value):
    """Event payloads must stay JSON-serializable even when a state check failed."""
    if isinstance(value, float):
        return value if math.isfinite(value) else repr(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


def validate_trial_limits(trial, cart_limits, scene_max_cart_pos):
    """Check initial and requested cart positions against the actual loaded scene."""
    if trial is None:
        return
    if (len(cart_limits) != 2 or not all(math.isfinite(value) for value in cart_limits)
            or cart_limits[0] > cart_limits[1] or not math.isfinite(scene_max_cart_pos)
            or scene_max_cart_pos <= 0):
        raise RuntimeError('Cart joint/scene limits are unavailable or invalid')
    for key in ('x0', 'hold_position', *(['target'] if 'target' in trial else [])):
        value = trial[key]
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not cart_limits[0] <= value <= cart_limits[1]
                or abs(value) > scene_max_cart_pos):
            raise RuntimeError(
                f'Trial {key}={value} m is outside cart limits {cart_limits} '
                f'or scene boundary +/-{scene_max_cart_pos} m')


def trial_viewer(trial, root_position):
    """Frame the cart/pole and requested +Y travel without changing the robot asset."""
    start = trial['x0'] if trial is not None else 0.0
    target = trial.get('target', trial['hold_position']) if trial is not None else start
    middle = (start + target) / 2
    x, y, z = root_position
    return {'eye': (x + 4.0, y + middle, z + 1.3),
            'lookat': (x, y + middle, z + 0.5)}


class CartpoleROS:
    """Own the demo scene, graph and boundary reset lifecycle."""

    def __init__(self, headless=False, trial=None):
        from isaaclab.app import AppLauncher
        self.run = Path(os.environ['ASTREX_RUN_DIR'])
        self.headless = headless
        self.trial = trial
        self.trial_ready_path = self.run / 'trial_ready.json' if trial is not None else None
        self.trial_initialized = False
        self.trial_first_feedback_pending = False
        experience = Path(os.environ['ASTREX_ISAAC_LAB_ROOT']) / os.environ['ASTREX_ISAAC_EXPERIENCE']
        self.app = AppLauncher(headless=headless, experience=str(experience), kit_args=(
            f'--/app/userConfigPath={self.run / "user.config.json"} '
            f'--/log/file={self.run / "kit.log"} '
            '--/app/settings/persistent=false --/app/settings/loadUserConfig=false')).app
        self.env = None
        self.closed = False
        self.events = (self.run / 'ros_events.jsonl').open('w')
        self.samples = (self.run / 'ros_samples.jsonl').open('w')
        self.resets = 0
        self.steps = 0
        self.keep_running = True
        # The ROS graph is process-lifetime: created exactly once, never rebuilt by reset().
        self.graph_creations = 0
        self.graph_path = None
        self.command_gate = 'closed'
        self.cached_command = None
        self.fault = None
        try:
            self.initialize()
        except BaseException:
            self.close()
            raise

    def initialize(self):
        import gymnasium as gym
        import isaaclab_tasks  # noqa: F401
        import omni.graph.core as og
        import omni.kit.app
        import omni.usd
        import numpy as np
        import torch
        import usdrt
        from isaaclab_tasks.utils import parse_env_cfg
        from isaacsim.core.prims import SingleArticulation
        from isaacsim.core.utils.extensions import enable_extension
        from pxr import UsdPhysics
        self.og, self.np, self.torch, self.usdrt = og, np, torch, usdrt
        enable_extension('omni.graph.action')
        enable_extension('isaacsim.ros2.bridge')
        manager = omni.kit.app.get_app().get_extension_manager()
        required = ['isaacsim.core.nodes.OnPhysicsStep', 'isaacsim.core.nodes.IsaacReadSimulationTime',
                    'isaacsim.ros2.bridge.ROS2PublishClock', 'isaacsim.ros2.bridge.ROS2PublishJointState',
                    'isaacsim.ros2.bridge.ROS2SubscribeJointState', 'isaacsim.ros2.bridge.ROS2Context',
                    'isaacsim.core.nodes.IsaacArticulationController']
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            registered = all(og.GraphRegistry().get_node_type_version(n) > 0 for n in required)
            if manager.is_extension_enabled('isaacsim.ros2.bridge') and registered:
                break
            self.app.update()
        else:
            raise RuntimeError('Official Bridge nodes did not register within 30 seconds')
        cfg = parse_env_cfg('Isaac-Cartpole-Direct-v0', device=os.environ['ASTREX_ROS_DEVICE'], num_envs=1)
        cfg.seed = 42
        cfg.sim.dt = 1 / 120
        cfg.decimation = 2
        cfg.initial_pole_angle_range = [0.0, 0.0]
        cfg.scene.clone_in_fabric = False
        self.viewer = trial_viewer(self.trial, cfg.robot_cfg.init_state.pos)
        cfg.viewer.eye = self.viewer['eye']
        cfg.viewer.lookat = self.viewer['lookat']
        self.env = gym.make('Isaac-Cartpole-Direct-v0', cfg=cfg)
        self.base = self.env.unwrapped
        print('ROS_STARTUP_STAGE env_created', flush=True)
        self.env.reset(seed=42)
        print('ROS_STARTUP_STAGE env_reset', flush=True)
        stage = omni.usd.get_context().get_stage()
        roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        if len(roots) != 1:
            raise RuntimeError('Expected one articulation root: ' + repr(roots))
        self.root = roots[0]
        self.names = list(self.base.cartpole.joint_names)
        self.cart = self.names.index('slider_to_cart')
        self.pole = self.names.index('cart_to_pole')
        cart_limits = self.base.cartpole.data.joint_pos_limits[0, self.cart].tolist()
        validate_trial_limits(self.trial, cart_limits, cfg.max_cart_pos)
        self.articulation = SingleArticulation(self.root, reset_xform_properties=False)
        self.articulation.initialize()
        print('ROS_STARTUP_STAGE articulation_initialized', flush=True)
        self.create_graph()
        if self.trial is not None:
            # Let the bridge create its DDS endpoints on one zero-state physics step,
            # but disconnect Controller execution before Subscribe can run.
            self.og.Controller.edit(self.path, {self.og.Controller.Keys.DISCONNECT: [
                (self.path + SUBSCRIBE_EXEC_OUT, self.path + CONTROLLER_EXEC_IN)]})
        self.zero_effort()
        self.clear_controller_target()
        if self.trial is not None:
            self.base.sim.step(render=not self.headless)
            self.base.scene.update(self.base.cfg.sim.dt)
            self.hold_trial_rest()
            self.close_command_acceptance()
            self.clear_controller_target()
        print('ROS_STARTUP_STAGE effort_zeroed', flush=True)
        ours, others = self.graph_prims()
        if len(ours) != 1 or ours[0] != self.graph_path:
            raise RuntimeError(f'Expected one AstrEX ROS graph prim: {ours} (others: {others})')
        if self.trial is None:
            self.open_command_acceptance()
        self.event('ready', graph=self.path, joints=self.names, articulation=self.root,
                   device=str(self.base.sim.device), dt=cfg.sim.dt, decimation=cfg.decimation,
                   experience=str(Path(os.environ['ASTREX_ISAAC_LAB_ROOT']) / os.environ['ASTREX_ISAAC_EXPERIENCE']),
                   gui=not self.headless, domain=os.environ['ROS_DOMAIN_ID'],
                   num_envs=int(self.base.num_envs), clone_in_fabric=bool(cfg.scene.clone_in_fabric),
                   pipeline_stage=str(self.og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND),
                   trigger_node_type='isaacsim.core.nodes.OnPhysicsStep',
                   rmw_implementation=os.environ.get('RMW_IMPLEMENTATION')
                   or os.environ.get('ASTREX_RMW_IMPLEMENTATION', ''),
                   graph_path=self.graph_path, graph_creations=self.graph_creations,
                   astrex_graph_count=len(ours), command_gate=self.command_gate,
                   trial=self.trial, cart_joint_limits=cart_limits,
                   scene_max_cart_pos=cfg.max_cart_pos, viewer=self.viewer,
                   publish=['/clock', '/joint_states'], subscribe=['/joint_command'])
        (self.run / 'ready.json').write_text(json.dumps(
            {'graph': self.path, 'pid': os.getpid(), 'trial': self.trial,
             'cart_joint_limits': cart_limits, 'scene_max_cart_pos': cfg.max_cart_pos,
             'device': str(self.base.sim.device), 'gui': not self.headless,
             'viewer': self.viewer}))
        print('Waiting for ROS commands', flush=True)

    def create_graph(self):
        # Process-lifetime guard: a second creation would duplicate publishers/subscribers.
        if self.graph_creations:
            raise RuntimeError('AstrEX ROS graph already exists; it must not be rebuilt')
        og, usdrt = self.og, self.usdrt
        self.path = GRAPH_PREFIX + uuid.uuid4().hex
        keys = og.Controller.Keys
        self.graph, _, _, _ = og.Controller.edit(
            {'graph_path': self.path, 'evaluator_name': 'execution',
             'pipeline_stage': og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND}, {
                keys.CREATE_NODES: [('Tick', 'isaacsim.core.nodes.OnPhysicsStep'),
                    ('Time', 'isaacsim.core.nodes.IsaacReadSimulationTime'),
                    ('Clock', 'isaacsim.ros2.bridge.ROS2PublishClock'),
                    ('Joint', 'isaacsim.ros2.bridge.ROS2PublishJointState'),
                    ('Subscribe', 'isaacsim.ros2.bridge.ROS2SubscribeJointState'),
                    ('Controller', 'isaacsim.core.nodes.IsaacArticulationController')],
                keys.CONNECT: [('Tick.outputs:step', 'Clock.inputs:execIn'),
                    ('Time.outputs:simulationTime', 'Clock.inputs:timeStamp'),
                    ('Tick.outputs:step', 'Joint.inputs:execIn'),
                    ('Time.outputs:simulationTime', 'Joint.inputs:timeStamp'),
                    ('Tick.outputs:step', 'Subscribe.inputs:execIn'),
                    ('Subscribe.outputs:execOut', 'Controller.inputs:execIn'),
                    ('Subscribe.outputs:jointNames', 'Controller.inputs:jointNames'),
                    ('Subscribe.outputs:positionCommand', 'Controller.inputs:positionCommand'),
                    ('Subscribe.outputs:velocityCommand', 'Controller.inputs:velocityCommand'),
                    ('Subscribe.outputs:effortCommand', 'Controller.inputs:effortCommand')],
                keys.SET_VALUES: [('Clock.inputs:topicName', '/clock'),
                    ('Joint.inputs:topicName', '/joint_states'),
                    ('Subscribe.inputs:topicName', '/joint_command'),
                    ('Subscribe.inputs:queueSize', 1),
                    ('Time.inputs:resetOnStop', False),
                    ('Joint.inputs:targetPrim', [usdrt.Sdf.Path(self.root)]),
                    ('Controller.inputs:targetPrim', [usdrt.Sdf.Path(self.root)])]})
        self.graph_creations += 1
        self.graph_path = self.path
        print('ROS_STARTUP_STAGE graph_created', flush=True)

    def event(self, kind, **fields):
        row = {'event': kind, 'wall_time': time.time(), 'wall_monotonic': time.monotonic(),
               'reset_count': self.resets, **fields}
        self.events.write(json.dumps(row, allow_nan=False) + '\n')
        self.events.flush()
        print('ROS_EVENT', json.dumps(row, allow_nan=False), flush=True)

    def zero_effort(self):
        # This writes the real PhysX command through the official articulation API.
        # The articulation view uses the torch backend: a numpy array raises
        # "unsqueeze(): argument 'input' must be Tensor" before PhysX is reached.
        device = getattr(self.articulation, '_device', None) or self.base.sim.device
        self.articulation.set_joint_efforts(
            self.torch.zeros(len(self.names), dtype=self.torch.float32, device=device))
        # Clear Lab's buffer too: env.reset() flushes scene data once.
        self.base.cartpole.set_joint_effort_target(self.torch.zeros_like(self.base.cartpole.data.joint_pos))

    def hold_trial_rest(self):
        """Pin the unstarted trial to the exact zero state while its ROS node starts."""
        self.zero_effort()
        rest = self.torch.zeros_like(self.base.cartpole.data.joint_pos)
        self.base.cartpole.write_joint_state_to_sim(rest, rest.clone())

    def maybe_initialize_trial(self):
        """Arm a deterministic initial state only after the external controller is ready."""
        if self.trial is None or self.trial_initialized or not self.trial_ready_path.exists():
            return
        ready = json.loads(self.trial_ready_path.read_text())
        required = {'x0', 'theta0', 'hold_position', 'controller_pid', 'trial_id'}
        if self.trial.get('task', 'hold') != 'hold':
            required.update(('task', 'target', 'reference_speed_mps'))
        if not isinstance(ready, dict) or set(ready) != required:
            raise RuntimeError(f'Invalid trial_ready.json schema; expected {sorted(required)}')
        if ready['trial_id'] != self.run.name:
            raise RuntimeError('trial_ready.json trial_id does not match this run')
        numeric_keys = ('x0', 'theta0', 'hold_position')
        if self.trial.get('task', 'hold') != 'hold':
            if ready['task'] != self.trial['task']:
                raise RuntimeError('trial_ready.json task does not match launched trial')
            numeric_keys += ('target', 'reference_speed_mps')
        for key in numeric_keys:
            value = ready[key]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise RuntimeError(f'Invalid trial_ready.json {key}')
            if value != self.trial[key]:
                raise RuntimeError(f'trial_ready.json {key} does not match launched trial')
        pid = ready['controller_pid']
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 1:
            raise RuntimeError('Invalid trial_ready.json controller_pid')
        try:
            os.kill(pid, 0)
        except OSError as exc:
            raise RuntimeError(f'Trial controller PID {pid} is not live') from exc

        # The gate routine advances one zero-state step to drain any stale command.
        self.hold_trial_rest()
        self.open_command_acceptance()
        self.zero_effort()
        self.clear_controller_target()
        position = self.torch.zeros_like(self.base.cartpole.data.joint_pos)
        velocity = self.torch.zeros_like(position)
        position[0, self.cart] = self.trial['x0']
        position[0, self.pole] = self.trial['theta0']
        self.base.cartpole.write_joint_state_to_sim(position, velocity)
        actual_position = self.articulation.get_joint_positions()
        actual_velocity = self.articulation.get_joint_velocities()
        if actual_position is None or actual_velocity is None:
            raise RuntimeError('Trial PhysX articulation read-back unavailable')
        actual_position = actual_position.tolist()
        actual_velocity = actual_velocity.tolist()
        if (abs(actual_position[self.cart] - self.trial['x0']) > 1e-6
                or abs(actual_position[self.pole] - self.trial['theta0']) > 1e-6
                or any(abs(value) > 1e-6 for value in actual_velocity)):
            raise RuntimeError(f'Trial initial state read-back failed: {actual_position}, {actual_velocity}')
        self.trial_initialized = True
        self.trial_first_feedback_pending = True
        self.event('trial_initialized', trial_id=self.run.name, requested=self.trial,
                   controller_pid=pid, state=self.snapshot(),
                   physx_readback={'position': actual_position, 'velocity': actual_velocity})

    def snapshot(self):
        q = self.base.cartpole.data.joint_pos[0].tolist()
        v = self.base.cartpole.data.joint_vel[0].tolist()
        if not all(math.isfinite(x) for x in q + v):
            raise RuntimeError('Nonfinite Cartpole state')
        get = self.og.Controller.get
        return {'physics_step': self.steps, 'reset_count': self.resets,
                'simulation_time': float(get(self.path + '/Time.outputs:simulationTime')),
                'joint_names': self.names, 'position': q, 'velocity': v,
                'subscriber_effort': list(get(self.path + '/Subscribe.outputs:effortCommand')),
                'controller_effort': list(get(self.path + '/Controller.inputs:effortCommand')),
                'computed_effort': None, 'applied_effort': None,
                'effort_note': 'N/A: Lab buffers do not measure the OmniGraph control path'}

    def boundary_reason(self, row):
        reasons = []
        if abs(row['position'][self.cart]) > self.base.cfg.max_cart_pos:
            reasons.append('cart_position')
        if abs(row['position'][self.pole]) > math.pi / 2:
            reasons.append('pole_angle')
        return reasons

    def graph_prims(self):
        """AstrEX ROS graph roots; other OmniGraph prims are reported but never asserted on."""
        import omni.usd
        stage = omni.usd.get_context().get_stage()
        ours, others = [], []
        for prim in stage.Traverse():
            if prim.GetTypeName() != 'OmniGraph':
                continue
            path = str(prim.GetPath())
            (ours if path.startswith(GRAPH_PREFIX) else others).append(path)
        return sorted(ours), sorted(others)

    def command_acceptance_links(self):
        """Upstream exec links of the controller; the gate is the only thing that changes them."""
        try:
            links = self.og.Controller.attribute(
                self.path + CONTROLLER_EXEC_IN).get_upstream_connections()
            return sorted(link.get_path() for link in links)
        except Exception as exc:
            return ['<query failed: ' + repr(exc) + '>']

    def subscriber_compute_links(self):
        """Upstream exec links of the subscriber node itself (the reset window freezes them)."""
        try:
            links = self.og.Controller.attribute(
                self.path + SUBSCRIBE_EXEC_IN).get_upstream_connections()
            return sorted(link.get_path() for link in links)
        except Exception as exc:
            return ['<query failed: ' + repr(exc) + '>']

    def close_command_acceptance(self):
        """Freeze the subscriber compute and the controller exec link; the DDS subscription stays alive."""
        keys = self.og.Controller.Keys
        pairs = []
        source = self.path + SUBSCRIBE_EXEC_OUT
        if source in self.command_acceptance_links():
            pairs.append((source, self.path + CONTROLLER_EXEC_IN))
        tick = self.path + TICK_STEP_OUT
        if tick in self.subscriber_compute_links():
            pairs.append((tick, self.path + SUBSCRIBE_EXEC_IN))
        if pairs:
            self.og.Controller.edit(self.path, {keys.DISCONNECT: pairs})
        self.command_gate = 'closed'
        self.cached_command = None
        state = {'links': self.command_acceptance_links(),
                 'compute_links': self.subscriber_compute_links()}
        self.event('command_gate_closed', **state)
        return state

    def open_command_acceptance(self):
        """Restore acceptance; commands received inside the window are consumed but not applied."""
        keys = self.og.Controller.Keys
        tick = self.path + TICK_STEP_OUT
        if tick not in self.subscriber_compute_links():
            self.og.Controller.edit(self.path, {keys.CONNECT: [(tick, self.path + SUBSCRIBE_EXEC_IN)]})
            # One step lets the subscriber consume any message received while the gate was closed;
            # its execution output has no controller connection at this point, so nothing is applied.
            self.base.sim.step(render=not self.headless)
            self.base.scene.update(self.base.cfg.sim.dt)
        source = self.path + SUBSCRIBE_EXEC_OUT
        if source not in self.command_acceptance_links():
            self.og.Controller.edit(self.path, {keys.CONNECT: [(source, self.path + CONTROLLER_EXEC_IN)]})
        self.command_gate = 'open'
        state = {'links': self.command_acceptance_links(),
                 'compute_links': self.subscriber_compute_links()}
        self.event('command_gate_opened', **state)
        return state

    def clear_controller_target(self):
        """Zero the graph-side command cache through the official controller inputs."""
        self.og.Controller.edit(self.path, {self.og.Controller.Keys.SET_VALUES: [
            (f'{self.path}/Controller.inputs:positionCommand', []),
            (f'{self.path}/Controller.inputs:velocityCommand', []),
            (f'{self.path}/Controller.inputs:effortCommand', [0.0] * len(self.names)),
            (f'{self.path}/Controller.inputs:jointNames', self.names)]})
        self.cached_command = None

    def verify_reset_state(self, boundary_time):
        """Every check must pass before command acceptance is restored."""
        row = self.snapshot()
        ours, others = self.graph_prims()
        # The controller inputs are connected to the subscriber, so reading them back resolves
        # upstream; the authoritative "target cleared" evidence is the articulation API read-back.
        target = self.articulation.get_applied_joint_efforts().detach().cpu().tolist()
        checks = {
            'astrex_graph_count_is_one': len(ours) == 1 and ours[0] == self.graph_path,
            'graph_path_unchanged': self.graph_path == self.path,
            'graph_creations_is_one': self.graph_creations == 1,
            'state_finite': all(math.isfinite(value) for value in row['position'] + row['velocity']),
            'position_at_rest': all(abs(value) < 1e-3 for value in row['position']),
            'velocity_at_rest': all(abs(value) < 1e-2 for value in row['velocity']),
            'articulation_effort_target_cleared': all(abs(value) < 1e-6 for value in target),
            'command_gate_closed': self.command_gate == 'closed',
            'simulation_time_not_rewound': row['simulation_time'] >= boundary_time - 1e-9,
        }
        return all(checks.values()), checks, row, ours, others

    def latch_fault(self, kind, **fields):
        """Any reset failure keeps acceptance CLOSED; control is never restored automatically."""
        try:
            self.zero_effort()
        except BaseException as exc:
            fields['zero_effort_error'] = repr(exc)
        self.command_gate = 'closed'
        self.cached_command = None
        payload = json_safe(fields)
        payload['command_gate'] = self.command_gate
        payload['links'] = self.command_acceptance_links()
        self.fault = {'kind': kind, 'reset_count': self.resets, 'graph_path': self.graph_path,
                      'fields': payload}
        try:
            self.event('reset_failed', fault_kind=kind, **payload)
        except BaseException as exc:
            print('ROS_FAULT_EVENT_FAILED ' + repr(exc), flush=True)
        (self.run / 'fault.json').write_text(json.dumps(self.fault, indent=2, default=str))
        print('ROS_FAULT_LATCHED ' + kind, flush=True)

    def reset(self, reason, hold=True):
        """Episode lifecycle only: the process-lifetime graph is never rebuilt or deleted."""
        old = self.snapshot()
        boundary_time = old['simulation_time']
        try:
            self.close_command_acceptance()
            self.zero_effort()
            self.clear_controller_target()
            self.event('boundary' if reason != ['test_reset'] else 'test_reset', reason=reason,
                       terminated=reason != ['test_reset'], previous=old,
                       reset_effort_target=[0.0] * len(self.names))
            if hold and not self.headless:
                deadline = time.monotonic() + 0.3
                while time.monotonic() < deadline and self.app.is_running():
                    self.base.sim.render()
                    time.sleep(0.01)
            self.env.reset(seed=42)
            self.zero_effort()
            self.clear_controller_target()
            self.base.scene.update(self.base.cfg.sim.dt)
            self.resets += 1
            verified, checks, row, ours, others = self.verify_reset_state(boundary_time)
            if not verified:
                self.latch_fault('reset_verification_failed', checks=checks, state=row,
                                 other_omnigraph_prims=others)
                return False
            self.open_command_acceptance()
            self.event('reset_ready', reason=reason, state=self.snapshot(),
                       reset_effort_target=[0.0] * len(self.names),
                       graph_path=self.graph_path, graph_creations=self.graph_creations,
                       astrex_graph_count=len(ours), other_omnigraph_prims=others,
                       command_gate=self.command_gate, verified=True, checks=checks)
            print('Waiting for ROS commands', flush=True)
            return True
        except BaseException:
            self.latch_fault('reset_exception', error=traceback.format_exc())
            return False

    def step(self):
        if self.trial is not None and not self.trial_initialized:
            self.hold_trial_rest()
            self.maybe_initialize_trial()
        self.base.sim.step(render=not self.headless)
        self.base.scene.update(self.base.cfg.sim.dt)
        if self.trial is not None and not self.trial_initialized:
            self.hold_trial_rest()
        self.steps += 1
        row = self.snapshot()
        row['boundary'] = self.boundary_reason(row)
        self.samples.write(json.dumps(row, allow_nan=False) + '\n')
        if self.trial_first_feedback_pending:
            self.event('trial_first_feedback', trial_id=self.run.name, state=row)
            self.trial_first_feedback_pending = False
        if self.steps % 120 == 0:
            self.samples.flush()
        if row['boundary'] and self.fault is None:
            self.reset(row['boundary'])
        return row

    def close(self):
        if self.closed:
            return
        self.closed = True
        try:
            if self.env is not None:
                if hasattr(self, 'articulation'):
                    self.zero_effort()
                self.env.close()
        finally:
            self.events.close()
            self.samples.close()
            self.app.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--trial-x0', type=float)
    parser.add_argument('--trial-theta0', type=float)
    parser.add_argument('--trial-hold-position', type=float)
    parser.add_argument('--trial-task', choices=('hold', 'move', 'move_then_hold'), default='hold')
    parser.add_argument('--trial-target', type=float)
    parser.add_argument('--trial-reference-speed', type=float, default=0.05)
    args = parser.parse_args()
    trial_values = (args.trial_x0, args.trial_theta0, args.trial_hold_position)
    trial = None
    if any(value is not None for value in trial_values):
        if any(value is None for value in trial_values) or not all(math.isfinite(value) for value in trial_values):
            parser.error('All trial initial-state options must be present and finite')
        if abs(args.trial_theta0) > math.radians(10):
            parser.error('Trial initial pole angle must be within 10 degrees')
        if abs(args.trial_x0 - args.trial_hold_position) > 0.25:
            parser.error('Trial initial cart offset must be within 0.25 m of hold position')
        trial = {'x0': args.trial_x0, 'theta0': args.trial_theta0,
                 'hold_position': args.trial_hold_position}
    if not math.isfinite(args.trial_reference_speed) or not 0 < args.trial_reference_speed <= 0.05:
        parser.error('Trial reference speed must be finite, positive and at most 0.05 m/s')
    if args.trial_task == 'hold':
        if args.trial_target is not None:
            parser.error('A final trial target is only valid for a moving task')
    else:
        if trial is None:
            parser.error('Moving trials require all three trial initial-state options')
        if args.trial_target is None or not math.isfinite(args.trial_target):
            parser.error('Moving trials require a finite final trial target')
        if args.trial_hold_position != args.trial_x0:
            parser.error('Moving trial hold position must equal x0; pass the final target separately')
        trial.update(task=args.trial_task, target=args.trial_target,
                     reference_speed_mps=args.trial_reference_speed)
    runtime = CartpoleROS(headless=args.headless, trial=trial)
    def stop(signum, frame):
        runtime.keep_running = False
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        import psutil
        process = psutil.Process()
        last_monitor = 0.0
        while runtime.app.is_running() and runtime.keep_running:
            started = time.monotonic()
            runtime.step()
            if started - last_monitor >= 10:
                runtime.event('monitor', rss_bytes=process.memory_info().rss)
                last_monitor = started
            time.sleep(max(0.0, runtime.base.cfg.sim.dt - (time.monotonic() - started)))
    finally:
        runtime.close()


if __name__ == '__main__':
    main()
