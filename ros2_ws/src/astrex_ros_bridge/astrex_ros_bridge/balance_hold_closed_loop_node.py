"""Bounded Cartpole hold, move and sequential GUI trial entry.

The dry-run path never creates a command publisher or releases the initial-state
handshake. This trial entry is not a production ROS Action Server.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.cartpole_reference import ContinuousStability, QuinticReference
from astrex_ros_bridge.state_cache import (
    CART_JOINT, POLE_JOINT, CartpoleStateCache, extract_stamp_ns,
)


FORCE_LIMIT_N = 5.0
ABORT_ANGLE_RAD = math.radians(10.0)
ABORT_POSITION_ERROR_M = 0.25
MAX_SATURATION_SIM_SEC = 0.5
STATE_TIMEOUT_LOCAL_SEC = 0.2
MAX_SIM_SEC = 10.0
MAX_LOCAL_SEC = 60.0
CONTROL_HZ = 60.0
ZERO_COUNT = 10


@dataclass(frozen=True)
class TrialConfig:
    x0: float
    theta0: float
    hold_position: float
    isaac_run_dir: str
    control_hz: float = CONTROL_HZ
    force_limit_n: float = FORCE_LIMIT_N
    max_sim_sec: float = MAX_SIM_SEC
    max_local_sec: float = MAX_LOCAL_SEC
    state_timeout_local_sec: float = STATE_TIMEOUT_LOCAL_SEC
    abort_angle_rad: float = ABORT_ANGLE_RAD
    abort_position_error_m: float = ABORT_POSITION_ERROR_M
    max_saturation_sim_sec: float = MAX_SATURATION_SIM_SEC
    stable_position_m: float = 0.05
    stable_velocity_mps: float = 0.10
    stable_angle_rad: float = math.radians(2.0)
    stable_angular_velocity_radps: float = 0.20
    stable_window_sim_sec: float = 3.0
    max_state_gap_sim_sec: float = 0.05
    task: str = 'hold'
    target: float | None = None
    reference_speed_mps: float = 0.05
    move_stable_sim_sec: float = 1.0
    move_settle_sim_sec: float = 8.0
    s1_timeout_local_sec: float = 0.5
    initialization_timeout_local_sec: float = 30.0
    dry_run: bool = False
    legacy_recovery: bool = True

    def validate(self) -> None:
        scalars = [value for value in asdict(self).values() if isinstance(value, (int, float))]
        if not all(math.isfinite(value) for value in scalars):
            raise ValueError('Trial configuration must be finite')
        if self.force_limit_n != FORCE_LIMIT_N or not 0 < self.max_sim_sec <= MAX_SIM_SEC:
            raise ValueError('The force limit is 5 N and a hold phase lasts at most 10 sim seconds')
        if self.control_hz <= 0 or self.state_timeout_local_sec <= 0 or self.max_local_sec <= 0:
            raise ValueError('Control frequency and local timeouts must be positive')
        ceilings = {
            'state_timeout_local_sec': STATE_TIMEOUT_LOCAL_SEC,
            'abort_angle_rad': ABORT_ANGLE_RAD,
            'abort_position_error_m': ABORT_POSITION_ERROR_M,
            'max_saturation_sim_sec': MAX_SATURATION_SIM_SEC,
            'stable_position_m': 0.05, 'stable_velocity_mps': 0.10,
            'stable_angle_rad': math.radians(2.0),
            'stable_angular_velocity_radps': 0.20,
            'max_state_gap_sim_sec': 0.05,
            'reference_speed_mps': 0.05,
            's1_timeout_local_sec': 0.5,
        }
        for name, ceiling in ceilings.items():
            if not 0 < getattr(self, name) <= ceiling:
                raise ValueError(f'{name} must be positive and cannot relax the fixed limit')
        if self.stable_window_sim_sec < 3.0:
            raise ValueError('Hold requires at least 3 continuous stable simulation seconds')
        if self.task not in ('hold', 'move', 'move_then_hold'):
            raise ValueError('Task must be hold, move or move_then_hold')
        if self.task != 'hold' and self.target is None:
            raise ValueError('A moving task needs a final target')
        if self.move_stable_sim_sec != 1.0 or self.move_settle_sim_sec != 8.0:
            raise ValueError('Move fixes a 1 s stable window and an 8 s settling allowance')
        if self.initialization_timeout_local_sec <= 0:
            raise ValueError('Initialization timeout must be positive')
        if abs(self.theta0) > self.abort_angle_rad:
            raise ValueError('Initial angle exceeds the configured exit boundary')
        if abs(self.x0 - self.hold_position) > self.abort_position_error_m:
            raise ValueError('Initial cart error exceeds the configured exit boundary')


class BalanceHoldClosedLoopNode(Node):
    """One state/control path; dry-run owns no command publisher."""

    def __init__(self, config: TrialConfig, trial_dir: Path):
        super().__init__('cartpole_trial', enable_rosout=False,
                         start_parameter_services=False)
        config.validate()
        self.config = config
        self.trial_dir = trial_dir
        self.isaac_run_dir = Path(config.isaac_run_dir).resolve()
        self.scene_bounds = None
        self._verify_isaac_identity()
        trial_dir.mkdir(parents=True, exist_ok=False)
        self.trace = (trial_dir / 'trace.jsonl').open('x', encoding='utf-8')
        self.events = (self.isaac_run_dir / 'ros_events.jsonl').open(encoding='utf-8')
        self.events.seek(0, os.SEEK_END)
        self.cache = CartpoleStateCache()
        self.controller = BalanceHoldController(max_force=config.force_limit_n)
        self._save_config()
        self.phase = 'WAIT_INITIALIZATION'
        self.task_phase = 'IDLE'
        self.stop_reason = None
        self.start_local_ns = time.monotonic_ns()
        self.initial_stamp_ns = None
        self.initial_state = None
        self.active_start_local_ns = None
        self.stage_start_stamp_ns = None
        self.reference = None
        self.path_corridor = None
        self.stability = ContinuousStability(config.stable_window_sim_sec,
                                             config.max_state_gap_sim_sec)
        self.first_control_ns = None
        self.last_control_ns = None
        self.feedback_count = 0
        self.control_count = 0
        self.max_published_force = 0.0
        self.saturation_start_stamp_ns = None
        self.max_saturation_sim_sec = 0.0
        self.stable_start_stamp_ns = None
        self.last_stability_stamp_ns = None
        self.stable_duration_sim_sec = 0.0
        self.success_state = None
        self.success_stamp_ns = None
        self.move_success_stamp_ns = None
        self.move_success_state = None
        self.move_stable_window = None
        self.s1_state = None
        self.hold_start_stamp_ns = None
        self.hold_success_stamp_ns = None
        self.hold_stable_window = None
        self.transition_zero_local_monotonic_ns = None
        self.last_graph_check_ns = time.monotonic_ns()
        self.publisher = None
        self._min_initial_stamp_ns = None
        self.feedback_subscription = self.create_subscription(
            JointState, '/joint_states', self._on_feedback, 10)
        self._preflight_command_graph()
        if not config.dry_run:
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
            self.publisher = self.create_publisher(JointState, '/joint_command', qos)
            deadline = time.monotonic() + 4.0
            while self.publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
            if self.publisher.get_subscription_count() != 1:
                raise RuntimeError('The only command publisher did not match Isaac subscriber')
        self.timer = self.create_timer(1.0 / config.control_hz, self._tick)
        request = {'x0': config.x0, 'theta0': config.theta0,
                   'hold_position': config.hold_position, 'controller_pid': os.getpid(),
                   'trial_id': self.isaac_run_dir.name}
        if config.task != 'hold':
            request.update(task=config.task, target=config.target,
                           reference_speed_mps=config.reference_speed_mps)
        marker = self.isaac_run_dir / 'trial_ready.json'
        if marker.exists():
            raise RuntimeError('Trial ready marker already exists')
        if config.dry_run:
            self.phase = 'WAIT_INITIAL_FEEDBACK'
        else:
            pending = self.isaac_run_dir / f'trial_ready.{os.getpid()}.tmp'
            with pending.open('x', encoding='utf-8') as stream:
                stream.write(json.dumps(request) + '\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(pending, marker)
        self._record('controller_ready', request=request, gain=self.controller.gain.tolist(),
                     publisher_created=self.publisher is not None, dry_run=config.dry_run)
        self.get_logger().info('Trial ready; waiting for new initial feedback')

    def _save_config(self) -> None:
        data = asdict(self.config)
        data['lqr_gain'] = self.controller.gain.reshape(-1).tolist()
        data['scene_bounds'] = self.scene_bounds
        (self.trial_dir / 'config.json').write_text(json.dumps(data, indent=2))

    def _verify_isaac_identity(self) -> None:
        ready = json.loads((self.isaac_run_dir / 'ready.json').read_text())
        pid = int(ready['pid'])
        cmdline = (Path('/proc') / str(pid) / 'cmdline').read_bytes()
        if b'run_ros_cartpole.py' not in cmdline or not ready['graph'].startswith(
                '/World/AstrEXROSGraph_'):
            raise RuntimeError('Isaac run directory does not identify a live Cartpole ROS profile')
        self.isaac_graph = ready['graph']
        if 'cart_joint_limits' in ready and 'scene_max_cart_pos' in ready:
            lower, upper = ready['cart_joint_limits']
            scene_limit = ready['scene_max_cart_pos']
            if (not all(math.isfinite(v) for v in (lower, upper, scene_limit))
                    or lower >= upper or scene_limit <= 0):
                raise RuntimeError('Isaac ready scene limits are invalid')
            self.scene_bounds = (max(lower, -scene_limit), min(upper, scene_limit))
            target = self.config.hold_position if self.config.task == 'hold' else self.config.target
            if not all(self.scene_bounds[0] <= value <= self.scene_bounds[1]
                       for value in (self.config.x0, self.config.hold_position, target)):
                raise RuntimeError('Initial state or final target is outside Isaac scene limits')
        elif self.config.task != 'hold':
            raise RuntimeError('Moving trials require actual Isaac cart and scene limits')

    def _preflight_command_graph(self) -> None:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            publishers = self.get_publishers_info_by_topic('/joint_command')
            subscribers = self.get_subscriptions_info_by_topic('/joint_command')
            if publishers or len(subscribers) == 1:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if (publishers or len(subscribers) != 1 or
                subscribers[0].topic_type != 'sensor_msgs/msg/JointState'):
            raise RuntimeError(
                f'Expected zero command publishers and one Isaac subscriber; '
                f'found {len(publishers)} publishers and {len(subscribers)} subscribers')

    def _record(self, event: str, **fields) -> None:
        row = {'event': event, 'wall_time_ns': time.time_ns(),
               'local_monotonic_ns': time.monotonic_ns(), **fields}
        self.trace.write(json.dumps(row, allow_nan=False) + '\n')
        self.trace.flush()

    @staticmethod
    def _fields(snapshot) -> dict:
        return {'stamp_ns': snapshot.stamp_ns,
                'received_monotonic_ns': snapshot.received_monotonic_ns,
                'x': snapshot.x, 'x_dot': snapshot.x_dot,
                'theta': snapshot.theta, 'theta_dot': snapshot.theta_dot}

    def _target(self) -> float:
        return (self.config.hold_position if self.config.task == 'hold'
                else self.config.target)

    def _reference_fields(self, snapshot) -> dict:
        reference = (self.reference.position(snapshot.stamp_ns)
                     if self.task_phase == 'MOVE' and self.reference is not None else self._target())
        velocity = (self.reference.velocity(snapshot.stamp_ns)
                    if self.task_phase == 'MOVE' and self.reference is not None else 0.0)
        return {'task_phase': self.task_phase, 'target': self._target(),
                'x_ref': reference, 'reference_velocity_mps': velocity}

    def _begin_control(self, snapshot) -> None:
        self.initial_state = snapshot
        self.initial_stamp_ns = snapshot.stamp_ns
        self.active_start_local_ns = time.monotonic_ns()
        self.stage_start_stamp_ns = snapshot.stamp_ns
        self.task_phase = 'HOLD' if self.config.task == 'hold' else 'MOVE'
        if self.task_phase == 'MOVE':
            self.reference = QuinticReference(
                snapshot.x, self._target(), snapshot.stamp_ns, self.config.reference_speed_mps)
            lower = min(snapshot.x, self._target()) - self.config.abort_position_error_m
            upper = max(snapshot.x, self._target()) + self.config.abort_position_error_m
            if self.scene_bounds is not None:
                lower, upper = max(lower, self.scene_bounds[0]), min(upper, self.scene_bounds[1])
            self.path_corridor = (lower, upper)
            self._record('reference_started', reference=asdict(self.reference),
                         path_corridor=self.path_corridor)
            self.stability = ContinuousStability(self.config.move_stable_sim_sec,
                                                 self.config.max_state_gap_sim_sec)
        else:
            self.hold_start_stamp_ns = snapshot.stamp_ns
        self._record('initial_feedback', **self._fields(snapshot),
                     **self._reference_fields(snapshot))
        if not self.config.dry_run and (
                abs(snapshot.x - self.config.x0) > 0.005
                or abs(snapshot.theta - self.config.theta0) > math.radians(0.2)
                or abs(snapshot.x_dot) > 0.05 or abs(snapshot.theta_dot) > 0.1):
            self.request_stop('INITIAL_STATE_MISMATCH')
            return
        self.phase = 'CONTROL'
        self._control_once(snapshot)
        if self.stop_reason is None:
            self._update_stability(snapshot)

    def _start_hold(self, snapshot) -> None:
        self.s1_state = snapshot
        self.hold_start_stamp_ns = snapshot.stamp_ns
        self.stage_start_stamp_ns = snapshot.stamp_ns
        self.task_phase = 'HOLD'
        self.phase = 'CONTROL'
        self.saturation_start_stamp_ns = None
        self.stability = ContinuousStability(self.config.stable_window_sim_sec,
                                             self.config.max_state_gap_sim_sec)
        self._sync_stability()
        self._record('hold_started', state=self._fields(snapshot),
                     **self._fields(snapshot), **self._reference_fields(snapshot))
        self._control_once(snapshot)
        if self.stop_reason is None:
            self._update_stability(snapshot)

    def _poll_isaac_events(self) -> None:
        while True:
            offset = self.events.tell()
            line = self.events.readline()
            if not line:
                break
            if not line.endswith('\n'):
                self.events.seek(offset)
                break
            event = json.loads(line)
            kind = event.get('event')
            if kind in ('boundary', 'reset_ready', 'reset_failed', 'test_reset'):
                self.request_stop('ISAAC_' + kind.upper())
                return
            if kind == 'trial_initialized':
                if self.phase != 'WAIT_INITIALIZATION':
                    self.request_stop('DUPLICATE_INITIALIZATION')
                    return
                state = event['state']
                self._min_initial_stamp_ns = int(round(state['simulation_time'] * 1e9))
                self.phase = 'WAIT_INITIAL_FEEDBACK'
                self._record('trial_initialized', isaac_state=state)

    def _on_feedback(self, msg: JointState) -> None:
        if self.stop_reason is not None:
            return
        # A queued boundary/reset must win over a potential success frame.
        self._poll_isaac_events()
        if self.stop_reason is not None:
            return
        stamp, stamp_error = extract_stamp_ns(msg)
        if (stamp_error is None and self.phase in ('CONTROL', 'WAIT_S1')
                and self.cache.snapshot is not None
                and stamp < self.cache.snapshot.stamp_ns):
            self.request_stop('STATE_TIME_REGRESSION')
            return
        accepted, reason = self.cache.update(msg)
        if not accepted:
            if reason != 'JointState header.stamp did not advance.':
                self._record('feedback_rejected', reason=reason)
                if self.phase in ('CONTROL', 'WAIT_S1', 'WAIT_INITIAL_FEEDBACK'):
                    self.request_stop('INVALID_FEEDBACK')
            return
        snapshot = self.cache.snapshot
        if self.phase == 'WAIT_INITIALIZATION':
            return
        if self.phase == 'WAIT_INITIAL_FEEDBACK':
            if (self._min_initial_stamp_ns is not None
                    and snapshot.stamp_ns <= self._min_initial_stamp_ns):
                return
            self._begin_control(snapshot)
            return
        if self.phase == 'WAIT_S1':
            self.feedback_count += 1
            self._record('feedback', **self._fields(snapshot),
                         **self._reference_fields(snapshot))
            if not self._state_safe(snapshot):
                return
            if (snapshot.stamp_ns > self.move_success_stamp_ns
                    and snapshot.received_monotonic_ns > self.transition_zero_local_monotonic_ns):
                self._start_hold(snapshot)
            return
        if self.phase == 'CONTROL':
            self.feedback_count += 1
            self._record('feedback', **self._fields(snapshot),
                         **self._reference_fields(snapshot))
            if self._safe_command(snapshot) is not None:
                self._update_stability(snapshot)

    def _publish(self, force: float, phase: str, snapshot=None, raw=None) -> None:
        if self.config.dry_run or self.publisher is None:
            raise RuntimeError('Dry-run must never publish a joint command')
        if not math.isfinite(force):
            raise ValueError('Refuse a nonfinite command')
        limit = self.config.force_limit_n
        command = max(-limit, min(limit, force))
        if abs(command) > limit:
            raise AssertionError('Published force exceeds 5 N')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [CART_JOINT, POLE_JOINT]
        msg.effort = [float(command), 0.0]
        self.publisher.publish(msg)
        self.max_published_force = max(self.max_published_force, abs(command))
        row = {'phase': phase, 'f_raw': raw, 'f_cmd': command,
               'hold_position': self.config.hold_position,
               'task_phase': self.task_phase, 'target': self._target(), 'pole_torque': 0.0,
               'clamp_ratio': (abs(command / raw) if raw else 1.0)}
        if snapshot is not None:
            row.update(self._fields(snapshot))
            row.update(self._reference_fields(snapshot))
        self._record('command', **row)

    def request_stop(self, reason: str) -> None:
        if self.stop_reason is not None:
            return
        self.stop_reason = reason
        self.phase = 'STOPPING'
        if self.publisher is not None:
            try:
                self._publish(0.0, 'stop')
            except Exception as exc:
                self._record('zero_publish_error', error=repr(exc))
                self.stop_reason = 'ZERO_PUBLISH_FAILED'
                self.success_stamp_ns = None
                self.success_state = None
        self.get_logger().info(
            f'Trial finished: {self.stop_reason}; success_stamp_ns={self.success_stamp_ns}')

    def _safe_command(self, snapshot):
        """Run every fault/timeout check before success or nonzero output."""
        if not self._state_safe(snapshot):
            return
        config = self.config
        x_ref = self._reference_fields(snapshot)['x_ref']
        elapsed_sim = (snapshot.stamp_ns - self.stage_start_stamp_ns) / 1e9
        limit = (self.reference.duration_sec + config.move_settle_sim_sec
                 if self.task_phase == 'MOVE' else config.max_sim_sec)
        if (not config.dry_run
                and elapsed_sim >= limit - 1.0 / config.control_hz):
            self.request_stop('SIM_TIMEOUT')
            return
        try:
            raw = self.controller.compute_raw_force_from_state(
                snapshot.values, x_ref)
            if not math.isfinite(raw):
                raise ValueError('Nonfinite LQR output')
            command = self.controller.compute_force_from_state(
                snapshot.values, x_ref)
            if abs(command) > config.force_limit_n:
                raise ValueError('Controller returned force above 5 N')
        except (ValueError, OverflowError, FloatingPointError) as exc:
            self._record('control_error', detail=repr(exc))
            self.request_stop('NONFINITE_OR_INVALID_FORCE')
            return
        if abs(raw) > config.force_limit_n:
            if self.saturation_start_stamp_ns is None:
                self.saturation_start_stamp_ns = snapshot.stamp_ns
            saturation_sec = (snapshot.stamp_ns - self.saturation_start_stamp_ns) / 1e9
            self.max_saturation_sim_sec = max(self.max_saturation_sim_sec, saturation_sec)
            if not config.dry_run and saturation_sec > config.max_saturation_sim_sec:
                self.request_stop('SATURATION_TIMEOUT')
                return
        else:
            self.saturation_start_stamp_ns = None
        return raw, command

    def _active_local_limit(self) -> float:
        if self.reference is None:
            return self.config.max_local_sec
        planned = self.reference.duration_sec + self.config.move_settle_sim_sec
        if self.config.task == 'move_then_hold':
            planned += self.config.max_sim_sec
        return max(self.config.max_local_sec, 3.0 * planned)

    def _state_safe(self, snapshot) -> bool:
        if self.stop_reason is not None:
            return False
        now_ns = time.monotonic_ns()
        config = self.config
        if (self.active_start_local_ns is not None
                and (now_ns - self.active_start_local_ns) / 1e9 > self._active_local_limit()):
            self.request_stop('LOCAL_TIMEOUT')
            return False
        if self.phase == 'WAIT_S1' and (
                (now_ns - self.transition_zero_local_monotonic_ns) / 1e9
                > config.s1_timeout_local_sec):
            self.request_stop('S1_TIMEOUT')
            return False
        if not self.cache.is_fresh(config.state_timeout_local_sec, now_ns):
            self.request_stop('STALE_FEEDBACK')
            return False
        x_ref = self._reference_fields(snapshot)['x_ref']
        if abs(snapshot.theta) > config.abort_angle_rad:
            self.request_stop('TRIAL_BOUNDARY')
            return False
        # A read-only cart cannot follow a moving reference. Keep diagnostics,
        # but evaluate tracking/saturation capability only after real actuation.
        if not config.dry_run and (
                abs(snapshot.x - x_ref) > config.abort_position_error_m
                or (self.task_phase == 'MOVE' and self.path_corridor is not None
                    and not self.path_corridor[0] <= snapshot.x <= self.path_corridor[1])
                or (self.scene_bounds is not None
                    and not self.scene_bounds[0] <= snapshot.x <= self.scene_bounds[1])):
            self.request_stop('TRIAL_BOUNDARY')
            return False
        if (now_ns - self.last_graph_check_ns) / 1e9 >= 1.0:
            if not self._command_graph_safe():
                return False
            self.last_graph_check_ns = now_ns
        return True

    def _command_graph_safe(self) -> bool:
        expected = 0 if self.config.dry_run else 1
        if len(self.get_publishers_info_by_topic('/joint_command')) != expected:
            self.request_stop('COMPETING_COMMAND_PUBLISHER')
            return False
        return True

    def _stable(self, snapshot) -> bool:
        config = self.config
        if not (abs(snapshot.x - self._target()) <= config.stable_position_m
                and abs(snapshot.x_dot) <= config.stable_velocity_mps
                and abs(snapshot.theta) <= config.stable_angle_rad
                and abs(snapshot.theta_dot) <= config.stable_angular_velocity_radps):
            return False
        # These extra ratios belong only to the historical hold recovery trial.
        if config.task == 'hold' and config.legacy_recovery:
            if abs(config.theta0) > 1e-9 and abs(snapshot.theta) >= 0.5 * abs(config.theta0):
                return False
            if (abs(config.x0 - config.hold_position) >= 0.05 - 1e-9
                    and abs(snapshot.x - config.hold_position) >= 0.025):
                return False
        return True

    def _sync_stability(self) -> None:
        self.stable_start_stamp_ns = self.stability.start_stamp_ns
        self.last_stability_stamp_ns = self.stability.last_stamp_ns
        self.stable_duration_sim_sec = self.stability.duration_sim_sec

    def _update_stability(self, snapshot) -> None:
        """Count simulation time only when a new accepted state arrives."""
        self.stability.max_gap_sec = self.config.max_state_gap_sim_sec
        qualified = self._stable(snapshot)
        if self.task_phase == 'MOVE':
            qualified = qualified and self.reference.finished(snapshot.stamp_ns)
        complete = self.stability.update(snapshot.stamp_ns, qualified)
        self._sync_stability()
        if self.config.dry_run:
            # This accepts wiring/calculation coverage, never physical recovery.
            covered = (complete if self.reference is None else
                       snapshot.stamp_ns >= self.reference.end_stamp_ns
                       + round(self.config.move_stable_sim_sec * 1e9))
            if covered and self._command_graph_safe():
                self.request_stop('DRY_RUN_PASS')
            return
        if not complete or not self._command_graph_safe():
            return
        if self.task_phase == 'MOVE':
            self.move_success_stamp_ns = snapshot.stamp_ns
            self.move_success_state = snapshot
            self.move_stable_window = self.stability.window()
            self._record('move_success', state=self._fields(snapshot),
                         stable_window=self.move_stable_window,
                         **self._fields(snapshot), **self._reference_fields(snapshot))
            if self.config.task == 'move_then_hold':
                self.phase = 'WAIT_S1'
                self.task_phase = 'WAIT_S1'
                try:
                    self._publish(0.0, 'transition_zero', snapshot, raw=0.0)
                except Exception as exc:
                    self._record('zero_publish_error', error=repr(exc))
                    self.request_stop('ZERO_PUBLISH_FAILED')
                    return
                self.transition_zero_local_monotonic_ns = time.monotonic_ns()
                self._record('phase_zero', **self._fields(snapshot), task_phase='WAIT_S1',
                             zero_published_local_monotonic_ns=self.transition_zero_local_monotonic_ns)
                return
            self.success_stamp_ns = snapshot.stamp_ns
            self.success_state = snapshot
            self.request_stop('MOVE_SUCCESS')
        else:
            self.hold_success_stamp_ns = snapshot.stamp_ns
            self.hold_stable_window = self.stability.window()
            self.success_stamp_ns = snapshot.stamp_ns
            self.success_state = snapshot
            self.request_stop('DEMO_SUCCESS' if self.config.task == 'move_then_hold' else 'SUCCESS')

    def _control_once(self, snapshot) -> None:
        pair = self._safe_command(snapshot)
        if pair is None:
            return
        raw, command = pair
        if self.config.dry_run:
            self._record('calculation', phase='control', f_raw=raw, f_cmd=command,
                         pole_torque=0.0, **self._fields(snapshot),
                         **self._reference_fields(snapshot))
        else:
            self._publish(command, 'control', snapshot, raw)
        now_ns = time.monotonic_ns()
        if self.first_control_ns is None:
            self.first_control_ns = now_ns
        self.last_control_ns = now_ns
        self.control_count += 1
        if self.control_count % 60 == 0:
            output_label = 'suggested_cmd' if self.config.dry_run else 'published'
            self.get_logger().info(
                f'control={self.control_count} x={snapshot.x:+.4f} '
                f'theta={math.degrees(snapshot.theta):+.3f}deg '
                f'raw={raw:+.3f}N {output_label}={command:+.3f}N')

    def _tick(self) -> None:
        if self.stop_reason is not None:
            return
        self._poll_isaac_events()
        if self.stop_reason is not None:
            return
        now_ns = time.monotonic_ns()
        if self.phase in ('WAIT_INITIALIZATION', 'WAIT_INITIAL_FEEDBACK'):
            if (now_ns - self.start_local_ns) / 1e9 > self.config.initialization_timeout_local_sec:
                self.request_stop('INITIALIZATION_TIMEOUT')
            return
        if self.phase == 'WAIT_S1':
            self._state_safe(self.cache.snapshot)
            return
        if self.phase != 'CONTROL':
            return
        self._control_once(self.cache.snapshot)

    def finish(self) -> dict:
        if self.stop_reason is None:
            self.request_stop('UNEXPECTED_EXIT')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if self.publisher is not None:
            for _ in range(ZERO_COUNT - 1):
                try:
                    self._publish(0.0, 'final_zero')
                    time.sleep(1.0 / self.config.control_hz)
                except Exception as exc:
                    self._record('zero_publish_error', error=repr(exc))
                    self.stop_reason = 'ZERO_PUBLISH_FAILED'
                    self.success_stamp_ns = None
                    self.success_state = None
        snapshot = self.cache.snapshot
        final = self._fields(snapshot) if snapshot else None
        duration_ns = (self.last_control_ns - self.first_control_ns
                       if self.last_control_ns and self.first_control_ns else 0)
        result = {'reason': self.stop_reason, 'initial_state': (
                  self._fields(self.initial_state) if self.initial_state else None),
                  'final_state': final,
                  'success_stamp_ns': self.success_stamp_ns,
                  'success_state': (self._fields(self.success_state)
                                    if self.success_state is not None else None),
                  'stable_window': ({
                      'start_stamp_ns': self.stable_start_stamp_ns,
                      'end_stamp_ns': self.success_stamp_ns,
                      'duration_sim_sec': self.stable_duration_sim_sec,
                  } if self.success_state is not None else None),
                  'task': self.config.task,
                  'dry_run': self.config.dry_run,
                  'publisher_created': self.publisher is not None,
                  'reference': asdict(self.reference) if self.reference is not None else None,
                  'path_corridor': self.path_corridor,
                  'active_local_limit_sec': self._active_local_limit(),
                  'move_success_stamp_ns': self.move_success_stamp_ns,
                  'move_success_state': (self._fields(self.move_success_state)
                                         if self.move_success_state is not None else None),
                  'move_stable_window': self.move_stable_window,
                  's1_state': self._fields(self.s1_state) if self.s1_state is not None else None,
                  'hold_start_stamp_ns': self.hold_start_stamp_ns,
                  'hold_success_stamp_ns': self.hold_success_stamp_ns,
                  'hold_stable_window': self.hold_stable_window,
                  'transition_zero_local_monotonic_ns': self.transition_zero_local_monotonic_ns,
                  'max_published_force_n': self.max_published_force,
                  'max_continuous_saturation_sim_sec': self.max_saturation_sim_sec,
                  'control_count': self.control_count,
                  'feedback_count': self.feedback_count,
                  'control_rate_hz': (self.control_count * 1e9 / duration_ns
                                      if duration_ns else 0.0),
                  'feedback_rate_hz': (self.feedback_count * 1e9 / duration_ns
                                       if duration_ns else 0.0)}
        self._record('exit', **result)
        (self.trial_dir / 'result.json').write_text(json.dumps(result, indent=2))
        self.events.close()
        self.trace.close()
        return result


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-run-dir', type=Path, required=True)
    parser.add_argument('--trial-dir', type=Path, required=True)
    parser.add_argument('--x0', type=float, required=True)
    parser.add_argument('--theta0', type=float, required=True, help='Initial pole angle in radians')
    parser.add_argument('--hold-position', type=float, required=True)
    parser.add_argument('--task', choices=('hold', 'move', 'move_then_hold'), default='hold')
    parser.add_argument('--target', type=float)
    parser.add_argument('--reference-speed', type=float, default=0.05,
                        help='Maximum reference speed in m/s; at most 0.05')
    parser.add_argument('--dry-run', action='store_true',
                        help='Read only; no publisher or initial-state release')
    parser.add_argument('--fixed-tolerances', action='store_true',
                        help='Use fixed four-state tolerances instead of historical hold reduction ratios')
    args = parser.parse_args()
    config = TrialConfig(args.x0, args.theta0, args.hold_position,
                         str(args.isaac_run_dir.resolve()), task=args.task,
                         target=args.target, reference_speed_mps=args.reference_speed,
                         dry_run=args.dry_run,
                         legacy_recovery=args.task == 'hold' and not args.fixed_tolerances)
    config.validate()
    if args.trial_dir.exists():
        parser.error('Trial output directory already exists')
    return config, args.trial_dir


def main():
    config, trial_dir = arguments()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = BalanceHoldClosedLoopNode(config, trial_dir)
        while node.stop_reason is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        if node is not None:
            node.request_stop('INTERRUPTED')
    except BaseException as exc:
        if node is not None:
            node._record('exception', detail=repr(exc))
            node.request_stop('EXCEPTION')
        else:
            raise
    finally:
        if node is not None:
            result = node.finish()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0 if result['reason'] in ('SUCCESS', 'MOVE_SUCCESS', 'DEMO_SUCCESS', 'DRY_RUN_PASS') else 1


if __name__ == '__main__':
    raise SystemExit(main())
