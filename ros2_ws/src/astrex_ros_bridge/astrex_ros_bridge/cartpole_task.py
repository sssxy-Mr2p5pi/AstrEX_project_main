"""Bounded Cartpole task logic without ROS communication or simulator ownership.

The adapter publishes returned forces. It acknowledges the Move transition zero,
and confirms the final zero through Isaac evidence before reporting success.
This module has no reset, publisher, service callback or filesystem side effects.
"""

from dataclasses import asdict, dataclass
import math
from typing import Protocol

from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.cartpole_reference import ContinuousStability, QuinticReference


class Snapshot(Protocol):
    x: float
    x_dot: float
    theta: float
    theta_dot: float
    stamp_ns: int
    received_monotonic_ns: int

    @property
    def values(self) -> tuple[float, float, float, float]: ...


@dataclass(frozen=True)
class TaskLimits:
    """The existing Step4 limits, fixed for the reusable service profile."""

    force_limit_n: float = 5.0
    reference_speed_mps: float = 0.05
    move_stable_sim_sec: float = 1.0
    hold_stable_sim_sec: float = 3.0
    move_settle_sim_sec: float = 8.0
    hold_timeout_sim_sec: float = 10.0
    state_timeout_local_sec: float = 0.2
    s1_timeout_local_sec: float = 0.5
    max_state_gap_sim_sec: float = 0.05
    max_saturation_sim_sec: float = 0.5
    abort_angle_rad: float = math.radians(10.0)
    abort_position_error_m: float = 0.25
    stable_position_m: float = 0.05
    stable_velocity_mps: float = 0.10
    stable_angle_rad: float = math.radians(2.0)
    stable_angular_velocity_radps: float = 0.20


def snapshot_fields(snapshot: Snapshot | None) -> dict | None:
    if snapshot is None:
        return None
    return {name: getattr(snapshot, name) for name in (
        'x', 'x_dot', 'theta', 'theta_dot', 'stamp_ns', 'received_monotonic_ns')}


def make_move_plan(snapshot: Snapshot, target: float, speed: float = 0.05,
                   margin: float = 0.25, scene_bounds=None):
    """Use the real current position, never the previous requested target."""
    reference = QuinticReference(snapshot.x, target, snapshot.stamp_ns, speed)
    lower, upper = min(snapshot.x, target) - margin, max(snapshot.x, target) + margin
    if scene_bounds is not None:
        lower, upper = max(lower, scene_bounds[0]), min(upper, scene_bounds[1])
    return reference, (lower, upper)


def four_state_stable(snapshot: Snapshot, target: float, limits) -> bool:
    return (abs(snapshot.x - target) <= limits.stable_position_m
            and abs(snapshot.x_dot) <= limits.stable_velocity_mps
            and abs(snapshot.theta) <= limits.stable_angle_rad
            and abs(snapshot.theta_dot) <= limits.stable_angular_velocity_radps)


def advance_phase_stability(stability: ContinuousStability, snapshot: Snapshot,
                            qualified: bool, phase: str,
                            reference: QuinticReference | None) -> bool:
    """A Move window starts only after the smooth reference reaches its end."""
    if phase == 'MOVE':
        qualified = qualified and reference.finished(snapshot.stamp_ns)
    return stability.update(snapshot.stamp_ns, qualified)


def s1_is_new(snapshot: Snapshot, move_success_stamp_ns: int,
              transition_zero_local_ns: int | None) -> bool:
    return (transition_zero_local_ns is not None
            and snapshot.stamp_ns > move_success_stamp_ns
            and snapshot.received_monotonic_ns > transition_zero_local_ns)


def task_local_limit(reference: QuinticReference | None, minimum_sec: float,
                     move_settle_sec: float, hold_timeout_sec: float = 0.0) -> float:
    if reference is None:
        return minimum_sec
    return max(minimum_sec, 3.0 * (
        reference.duration_sec + move_settle_sec + hold_timeout_sec))


@dataclass(frozen=True)
class TaskStep:
    """A calculation, not proof that Isaac applied its returned command."""

    phase: str
    event: str | None
    f_raw: float | None
    f_cmd: float
    x_ref: float
    reference_velocity_mps: float
    done: bool
    success: bool
    reason: str | None


class CartpoleTask:
    """One sequential Move → transition zero → fresh S1 → Hold request."""

    def __init__(self, controller=None, scene_bounds=None):
        self.limits = TaskLimits()
        self.controller = (BalanceHoldController(max_force=self.limits.force_limit_n)
                           if controller is None else controller)
        if self.controller.max_force != self.limits.force_limit_n:
            raise ValueError('The service controller must retain the 5 N limit')
        if scene_bounds is not None:
            if (len(scene_bounds) != 2 or not all(math.isfinite(v) for v in scene_bounds)
                    or scene_bounds[0] >= scene_bounds[1]):
                raise ValueError('Scene bounds must be finite and ordered')
            scene_bounds = tuple(scene_bounds)
        self.scene_bounds = scene_bounds
        self.phase = 'IDLE'
        self.reason = None

    @property
    def done(self) -> bool:
        return self.phase == 'DONE'

    @property
    def success(self) -> bool:
        return self.done and self.reason == 'SUCCESS'

    @staticmethod
    def admission_error(snapshot: Snapshot | None, target_position_m: float,
                        now_monotonic_ns: int) -> str | None:
        if (not isinstance(target_position_m, (int, float))
                or isinstance(target_position_m, bool)
                or not math.isfinite(target_position_m)
                or not -0.5 <= target_position_m <= 0.5):
            return 'INVALID_TARGET: require a finite absolute coordinate in [-0.5, 0.5] m'
        if snapshot is None:
            return 'NO_STATE: no valid joint feedback'
        if not all(math.isfinite(v) for v in snapshot.values):
            return 'INVALID_STATE: four-state feedback must be finite'
        if (not isinstance(snapshot.stamp_ns, int) or isinstance(snapshot.stamp_ns, bool)
                or snapshot.stamp_ns < 0
                or not isinstance(snapshot.received_monotonic_ns, int)
                or isinstance(snapshot.received_monotonic_ns, bool)
                or snapshot.received_monotonic_ns < 0
                or not isinstance(now_monotonic_ns, int)
                or isinstance(now_monotonic_ns, bool) or now_monotonic_ns < 0):
            return 'INVALID_STATE_TIME: invalid simulation or local timestamp'
        age_ns = now_monotonic_ns - snapshot.received_monotonic_ns
        if not 0 <= age_ns <= 200_000_000:
            return 'STALE_STATE: feedback must be at most 0.2 local seconds old'
        if not (abs(snapshot.x) <= 0.55 and abs(snapshot.x_dot) <= 0.10
                and abs(snapshot.theta) <= math.radians(2.0)
                and abs(snapshot.theta_dot) <= 0.20):
            return 'INITIAL_STATE_REJECTED: state is outside small-perturbation admission limits'
        return None

    def begin(self, snapshot: Snapshot, target_position_m: float,
              now_monotonic_ns: int) -> None:
        if self.phase not in ('IDLE', 'DONE'):
            raise ValueError('BUSY: a task is already active')
        error = self.admission_error(snapshot, target_position_m, now_monotonic_ns)
        if error:
            raise ValueError(error)
        if self.scene_bounds is not None and not all(
                self.scene_bounds[0] <= value <= self.scene_bounds[1]
                for value in (snapshot.x, target_position_m)):
            raise ValueError('INITIAL_STATE_REJECTED: current position or target is outside scene bounds')
        self.initial_state = snapshot
        self.target = float(target_position_m)
        self.start_local_ns = now_monotonic_ns
        self.stage_start_stamp_ns = snapshot.stamp_ns
        self.reference, self.path_corridor = make_move_plan(
            snapshot, self.target, self.limits.reference_speed_mps,
            self.limits.abort_position_error_m, self.scene_bounds)
        self.local_limit_sec = task_local_limit(
            self.reference, 60.0, self.limits.move_settle_sim_sec,
            self.limits.hold_timeout_sim_sec)
        self.stability = ContinuousStability(
            self.limits.move_stable_sim_sec, self.limits.max_state_gap_sim_sec)
        self.move_success_state = self.s1_state = self.success_state = None
        self.move_success_stamp_ns = self.hold_start_stamp_ns = None
        self.hold_success_stamp_ns = self.success_stamp_ns = None
        self.move_stable_window = self.hold_stable_window = None
        self.transition_zero_local_monotonic_ns = None
        self.wait_s1_local_ns = None
        self.saturation_start_stamp_ns = None
        self.max_saturation_sim_sec = 0.0
        self.last_stamp_ns = snapshot.stamp_ns
        self.last_tick_local_ns = now_monotonic_ns
        self.phase, self.reason = 'MOVE', None

    def note_transition_zero(self, publish_completed_monotonic_ns: int) -> None:
        """Call only after the adapter successfully publishes the Move zero."""
        if self.phase != 'WAIT_S1':
            raise ValueError('A transition zero is acknowledged only while waiting for S1')
        if (not isinstance(publish_completed_monotonic_ns, int)
                or publish_completed_monotonic_ns < self.wait_s1_local_ns):
            raise ValueError('Transition zero completion cannot precede Move success')
        if self.transition_zero_local_monotonic_ns is not None:
            raise ValueError('Transition zero has already been acknowledged')
        self.transition_zero_local_monotonic_ns = publish_completed_monotonic_ns

    def fail(self, reason: str) -> None:
        """External graph, reset and shutdown faults also terminate this task."""
        if not isinstance(reason, str) or not reason:
            raise ValueError('A failure reason is required')
        if self.done and not self.success:
            return
        self.phase, self.reason = 'DONE', reason
        self.success_state = None
        self.success_stamp_ns = None

    def _reference(self, snapshot: Snapshot | None) -> tuple[float, float]:
        if self.phase == 'MOVE' and snapshot is not None:
            return (self.reference.position(snapshot.stamp_ns),
                    self.reference.velocity(snapshot.stamp_ns))
        return self.target, 0.0

    def _step(self, snapshot, event=None, raw=None, command=0.0) -> TaskStep:
        reference, velocity = self._reference(snapshot)
        return TaskStep(self.phase, event, raw, command, reference, velocity,
                        self.done, self.success, self.reason)

    def _fault(self, snapshot: Snapshot | None, now_ns: int) -> str | None:
        if not isinstance(now_ns, int) or now_ns < self.last_tick_local_ns:
            return 'LOCAL_TIME_REGRESSION'
        if (now_ns - self.start_local_ns) / 1e9 > self.local_limit_sec:
            return 'LOCAL_TIMEOUT'
        if snapshot is None:
            return 'INVALID_FEEDBACK'
        if not all(math.isfinite(v) for v in snapshot.values):
            return 'INVALID_FEEDBACK'
        if (not isinstance(snapshot.stamp_ns, int) or snapshot.stamp_ns < 0
                or not isinstance(snapshot.received_monotonic_ns, int)):
            return 'INVALID_FEEDBACK_TIME'
        if snapshot.stamp_ns < self.last_stamp_ns:
            return 'STATE_TIME_REGRESSION'
        age = now_ns - snapshot.received_monotonic_ns
        if not 0 <= age <= self.limits.state_timeout_local_sec * 1e9:
            return 'STALE_FEEDBACK'
        reference, _ = self._reference(snapshot)
        if (abs(snapshot.theta) > self.limits.abort_angle_rad
                or abs(snapshot.x - reference) > self.limits.abort_position_error_m
                or (self.phase == 'MOVE'
                    and not self.path_corridor[0] <= snapshot.x <= self.path_corridor[1])
                or (self.scene_bounds is not None
                    and not self.scene_bounds[0] <= snapshot.x <= self.scene_bounds[1])):
            return 'TASK_BOUNDARY'
        if self.phase == 'WAIT_S1':
            start = (self.transition_zero_local_monotonic_ns
                     if self.transition_zero_local_monotonic_ns is not None
                     else self.wait_s1_local_ns)
            if (now_ns - start) / 1e9 > self.limits.s1_timeout_local_sec:
                return 'S1_TIMEOUT'
        else:
            deadline = (self.reference.duration_sec + self.limits.move_settle_sim_sec
                        if self.phase == 'MOVE' else self.limits.hold_timeout_sim_sec)
            if (snapshot.stamp_ns - self.stage_start_stamp_ns) / 1e9 >= deadline:
                return 'SIM_TIMEOUT'
        return None

    def tick(self, snapshot: Snapshot | None, now_monotonic_ns: int) -> TaskStep:
        """Read one snapshot. Repeated snapshots never accumulate stable time."""
        if self.phase == 'IDLE':
            raise ValueError('Begin a task before ticking it')
        if self.done:
            return self._step(snapshot)
        error = self._fault(snapshot, now_monotonic_ns)
        if error:
            self.fail(error)
            return self._step(snapshot, event='fault')
        self.last_tick_local_ns = now_monotonic_ns
        self.last_stamp_ns = snapshot.stamp_ns
        event = None
        if self.phase == 'WAIT_S1':
            if not s1_is_new(snapshot, self.move_success_stamp_ns,
                             self.transition_zero_local_monotonic_ns):
                return self._step(snapshot)
            self.s1_state = snapshot
            self.hold_start_stamp_ns = self.stage_start_stamp_ns = snapshot.stamp_ns
            self.phase = 'HOLD'
            self.saturation_start_stamp_ns = None
            self.stability = ContinuousStability(
                self.limits.hold_stable_sim_sec, self.limits.max_state_gap_sim_sec)
            event = 'hold_started'
        reference, _ = self._reference(snapshot)
        try:
            raw = self.controller.compute_raw_force_from_state(snapshot.values, reference)
            command = self.controller.compute_force_from_state(snapshot.values, reference)
            if (not math.isfinite(raw) or not math.isfinite(command)
                    or abs(command) > self.limits.force_limit_n):
                raise ValueError('Invalid force or controller limit')
        except (ValueError, OverflowError, FloatingPointError):
            self.fail('NONFINITE_OR_INVALID_FORCE')
            return self._step(snapshot, event='fault')
        if abs(raw) > self.limits.force_limit_n:
            if self.saturation_start_stamp_ns is None:
                self.saturation_start_stamp_ns = snapshot.stamp_ns
            duration = (snapshot.stamp_ns - self.saturation_start_stamp_ns) / 1e9
            self.max_saturation_sim_sec = max(self.max_saturation_sim_sec, duration)
            if duration > self.limits.max_saturation_sim_sec:
                self.fail('SATURATION_TIMEOUT')
                return self._step(snapshot, event='fault', raw=raw)
        else:
            self.saturation_start_stamp_ns = None
        complete = advance_phase_stability(
            self.stability, snapshot, four_state_stable(snapshot, self.target, self.limits),
            self.phase, self.reference)
        if complete and self.phase == 'MOVE':
            self.move_success_state = snapshot
            self.move_success_stamp_ns = snapshot.stamp_ns
            self.move_stable_window = self.stability.window()
            self.phase = 'WAIT_S1'
            self.wait_s1_local_ns = now_monotonic_ns
            return self._step(snapshot, event='move_success', raw=0.0)
        if complete:
            self.hold_success_stamp_ns = self.success_stamp_ns = snapshot.stamp_ns
            self.success_state = snapshot
            self.hold_stable_window = self.stability.window()
            self.phase, self.reason = 'DONE', 'SUCCESS'
            return self._step(snapshot, event='hold_success', raw=0.0)
        return self._step(snapshot, event=event, raw=raw, command=command)

    def fields(self) -> dict:
        """Evidence for the adapter; force buffers remain adapter-owned."""
        if self.phase == 'IDLE':
            return {'phase': 'IDLE'}
        return {
            'phase': self.phase, 'reason': self.reason, 'target': self.target,
            'initial_state': snapshot_fields(self.initial_state),
            'reference': asdict(self.reference), 'path_corridor': self.path_corridor,
            'active_local_limit_sec': self.local_limit_sec,
            'move_success_stamp_ns': self.move_success_stamp_ns,
            'move_success_state': snapshot_fields(self.move_success_state),
            'move_stable_window': self.move_stable_window,
            'transition_zero_local_monotonic_ns': self.transition_zero_local_monotonic_ns,
            's1_state': snapshot_fields(self.s1_state),
            'hold_start_stamp_ns': self.hold_start_stamp_ns,
            'hold_success_stamp_ns': self.hold_success_stamp_ns,
            'hold_stable_window': self.hold_stable_window,
            'success_stamp_ns': self.success_stamp_ns,
            'success_state': snapshot_fields(self.success_state),
            'max_continuous_saturation_sim_sec': self.max_saturation_sim_sec,
        }
