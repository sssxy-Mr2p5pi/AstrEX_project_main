"""Pure task tests. Synthetic samples do not count as physical acceptance."""

from dataclasses import dataclass, replace
import math

import pytest

from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.cartpole_task import CartpoleTask, TaskLimits, s1_is_new


LOCAL_BASE = 1_000_000_000_000
STEP_NS = 25_000_000


@dataclass(frozen=True)
class Sample:
    x: float = 0.0
    x_dot: float = 0.0
    theta: float = 0.0
    theta_dot: float = 0.0
    stamp_ns: int = 0
    received_monotonic_ns: int = LOCAL_BASE

    @property
    def values(self):
        return self.x, self.x_dot, self.theta, self.theta_dot


def sample(stamp_ns=0, **fields):
    return Sample(stamp_ns=stamp_ns, received_monotonic_ns=LOCAL_BASE + stamp_ns, **fields)


def begun(target=0.0, initial=None, controller=None, scene_bounds=None):
    task = CartpoleTask(controller=controller, scene_bounds=scene_bounds)
    initial = initial or sample()
    task.begin(initial, target, initial.received_monotonic_ns)
    return task


def feed(task, start_ns, end_ns, **fields):
    result = None
    for stamp in range(start_ns, end_ns + 1, STEP_NS):
        current = sample(stamp, **fields)
        result = task.tick(current, current.received_monotonic_ns)
    return result


def move_zero_to_wait(task):
    task.tick(sample(), LOCAL_BASE)
    result = feed(task, STEP_NS, 1_000_000_000)
    assert result.event == 'move_success'
    assert result.f_cmd == 0.0 and not result.done
    assert task.phase == 'WAIT_S1'
    return LOCAL_BASE + 1_000_000_001


@pytest.mark.parametrize('target', [-0.5, -0.3, 0.0, 0.3, 0.5])
def test_target_range_includes_both_endpoints(target):
    task = begun(target)
    assert task.target == target
    assert task.phase == 'MOVE'


@pytest.mark.parametrize('target', [-0.500001, 0.500001, math.nan, math.inf, -math.inf, True])
def test_invalid_targets_are_rejected_without_starting(target):
    task = CartpoleTask()
    with pytest.raises(ValueError, match='INVALID_TARGET'):
        task.begin(sample(), target, LOCAL_BASE)
    assert task.phase == 'IDLE'


@pytest.mark.parametrize('current,now,reason', [
    (None, LOCAL_BASE, 'NO_STATE'),
    (sample(theta=math.nan), LOCAL_BASE, 'INVALID_STATE'),
    (sample(), LOCAL_BASE + 200_000_001, 'STALE_STATE'),
    (sample(), LOCAL_BASE - 1, 'STALE_STATE'),
    (sample(x=0.550001), LOCAL_BASE, 'INITIAL_STATE_REJECTED'),
    (sample(x_dot=0.100001), LOCAL_BASE, 'INITIAL_STATE_REJECTED'),
    (sample(theta=math.radians(2.0001)), LOCAL_BASE, 'INITIAL_STATE_REJECTED'),
    (sample(theta_dot=0.200001), LOCAL_BASE, 'INITIAL_STATE_REJECTED'),
    (replace(sample(), stamp_ns=-1), LOCAL_BASE, 'INVALID_STATE_TIME'),
])
def test_state_admission(current, now, reason):
    assert CartpoleTask.admission_error(current, 0.0, now).startswith(reason)


def test_current_real_position_starts_reference_and_scene_clips_corridor():
    initial = sample(x=0.271, theta=0.001)
    task = begun(-0.3, initial=initial, scene_bounds=(-0.6, 0.6))
    assert task.reference.x_start == 0.271
    assert task.reference.position(0) == 0.271
    assert task.path_corridor == (-0.55, 0.521)
    assert task.local_limit_sec == pytest.approx(3.0 * (task.reference.duration_sec + 18.0))
    with pytest.raises(ValueError, match='scene bounds'):
        begun(0.5, scene_bounds=(-0.3, 0.3))


def test_busy_cannot_replace_reference_or_current_state():
    task = begun(0.3)
    initial = task.initial_state
    with pytest.raises(ValueError, match='BUSY'):
        task.begin(sample(x=-0.2), -0.3, LOCAL_BASE)
    assert task.initial_state is initial and task.target == 0.3


def test_reference_must_finish_before_move_stability_window():
    task = begun(0.01)
    assert task.reference.duration_sec == 2.0
    task.tick(sample(), LOCAL_BASE)
    feed(task, STEP_NS, 1_975_000_000)
    assert task.stability.start_stamp_ns is None
    current = sample(2_000_000_000, x=0.01)
    task.tick(current, current.received_monotonic_ns)
    assert task.stability.start_stamp_ns == 2_000_000_000
    result = feed(task, 2_025_000_000, 3_000_000_000, x=0.01)
    assert result.event == 'move_success'
    assert task.move_stable_window['duration_sim_sec'] == 1.0


def test_repeated_snapshot_and_timer_ticks_do_not_accumulate_time_or_repeat_events():
    task = begun()
    current = sample()
    task.tick(current, LOCAL_BASE)
    for index in range(20):
        result = task.tick(current, LOCAL_BASE + index * 1_000_000)
        assert result.event is None and not result.done
    assert task.stability.duration_sim_sec == 0.0


def test_sequential_transition_requires_zero_ack_and_two_new_state_times():
    task = begun()
    ack = move_zero_to_wait(task)
    newer = sample(1_025_000_000)
    result = task.tick(newer, newer.received_monotonic_ns)
    assert result.f_cmd == 0.0 and task.s1_state is None
    task.note_transition_zero(newer.received_monotonic_ns + 1)
    result = task.tick(newer, newer.received_monotonic_ns + 2)
    assert result.f_cmd == 0.0 and task.s1_state is None
    assert not s1_is_new(sample(1_000_000_000), 1_000_000_000, ack)
    current = sample(1_050_000_000)
    result = task.tick(current, current.received_monotonic_ns)
    assert result.event == 'hold_started' and task.phase == 'HOLD'
    assert task.s1_state is current
    assert task.stability.duration_sim_sec == 0.0
    result = feed(task, 1_075_000_000, 4_025_000_000)
    assert not result.done
    result = feed(task, 4_050_000_000, 4_050_000_000)
    assert result.done and result.success and result.f_cmd == 0.0
    assert task.hold_stable_window['duration_sim_sec'] == 3.0
    assert task.move_stable_window['duration_sim_sec'] == 1.0
    assert task.fields()['hold_start_stamp_ns'] == current.stamp_ns


def test_transition_zero_ack_cannot_be_missing_repeated_or_early():
    task = begun()
    with pytest.raises(ValueError, match='waiting for S1'):
        task.note_transition_zero(LOCAL_BASE)
    ack = move_zero_to_wait(task)
    with pytest.raises(ValueError, match='precede'):
        task.note_transition_zero(ack - 2)
    task.note_transition_zero(ack)
    with pytest.raises(ValueError, match='already'):
        task.note_transition_zero(ack + 1)


@pytest.mark.parametrize('fields', [
    {'x': 0.06}, {'x_dot': 0.11}, {'theta': math.radians(2.1)}, {'theta_dot': 0.21},
])
def test_stability_condition_break_restarts_full_window(fields):
    task = begun()
    task.tick(sample(), LOCAL_BASE)
    feed(task, STEP_NS, 975_000_000)
    current = sample(1_000_000_000, **fields)
    task.tick(current, current.received_monotonic_ns)
    assert task.stability.start_stamp_ns is None
    result = feed(task, 1_025_000_000, 2_000_000_000)
    assert result.event is None
    result = feed(task, 2_025_000_000, 2_025_000_000)
    assert result.event == 'move_success'


def test_feedback_gap_restarts_stability_but_time_reversal_fails():
    task = begun()
    task.tick(sample(), LOCAL_BASE)
    feed(task, STEP_NS, 975_000_000)
    current = sample(1_075_000_000)
    task.tick(current, current.received_monotonic_ns)
    assert task.stability.start_stamp_ns == current.stamp_ns
    backward = sample(1_050_000_000)
    result = task.tick(backward, current.received_monotonic_ns + 1)
    assert result.done and not result.success and result.f_cmd == 0.0
    assert result.reason == 'STATE_TIME_REGRESSION'


@pytest.mark.parametrize('current,now,reason', [
    (sample(theta=math.radians(10.1)), LOCAL_BASE, 'TASK_BOUNDARY'),
    (sample(x=-0.251), LOCAL_BASE, 'TASK_BOUNDARY'),
    (sample(), LOCAL_BASE + 200_000_001, 'STALE_FEEDBACK'),
    (sample(), LOCAL_BASE - 1, 'LOCAL_TIME_REGRESSION'),
    (sample(theta=math.inf), LOCAL_BASE, 'INVALID_FEEDBACK'),
    (sample(8_000_000_000), LOCAL_BASE + 8_000_000_000, 'SIM_TIMEOUT'),
    (sample(10_000_000), LOCAL_BASE + 60_000_000_001, 'LOCAL_TIMEOUT'),
])
def test_faults_win_over_success_and_return_zero(current, now, reason):
    task = begun()
    result = task.tick(current, now)
    assert result.done and not result.success and result.f_cmd == 0.0
    assert task.reason == reason and task.success_state is None


def test_s1_timeout_and_external_reset_are_failures_not_recovery():
    task = begun()
    ack = move_zero_to_wait(task)
    task.note_transition_zero(ack)
    current = sample(1_025_000_000)
    current = replace(current, received_monotonic_ns=ack + 500_000_001)
    result = task.tick(current, current.received_monotonic_ns)
    assert result.reason == 'S1_TIMEOUT' and result.f_cmd == 0.0
    other = begun()
    other.fail('ISAAC_BOUNDARY')
    assert other.tick(sample(), LOCAL_BASE).reason == 'ISAAC_BOUNDARY'


class ConstantController:
    max_force = 5.0

    def __init__(self, force):
        self.force = force

    def compute_raw_force_from_state(self, state, target):
        return self.force

    def compute_force_from_state(self, state, target):
        return max(-self.max_force, min(self.max_force, self.force))


def test_saturation_stays_clipped_then_fails_after_half_sim_second():
    task = begun(controller=ConstantController(6.0))
    assert task.tick(sample(), LOCAL_BASE).f_cmd == 5.0
    result = feed(task, STEP_NS, 500_000_000)
    assert not result.done and result.f_cmd == 5.0
    result = feed(task, 525_000_000, 525_000_000)
    assert result.reason == 'SATURATION_TIMEOUT' and result.f_cmd == 0.0
    assert task.max_saturation_sim_sec == 0.525


def test_nonfinite_raw_and_invalid_controller_limit_stop_output():
    task = begun(controller=ConstantController(math.nan))
    result = task.tick(sample(), LOCAL_BASE)
    assert result.reason == 'NONFINITE_OR_INVALID_FORCE' and result.f_cmd == 0.0
    with pytest.raises(ValueError, match='5 N'):
        CartpoleTask(controller=BalanceHoldController(max_force=6.0))


def test_same_instance_can_begin_again_without_old_windows_or_previous_target():
    task = begun(0.3)
    task.fail('UNIT_FAILURE')
    current = sample(10_000_000_000, x=0.2992)
    task.begin(current, -0.3, current.received_monotonic_ns)
    assert task.reference.x_start == 0.2992
    assert task.reason is None and task.stability.duration_sim_sec == 0.0
    assert task.move_success_state is None and task.s1_state is None
    assert task.transition_zero_local_monotonic_ns is None
    assert task.max_saturation_sim_sec == 0.0


def test_external_fault_can_override_algorithm_success_before_final_zero_confirm():
    task = begun()
    ack = move_zero_to_wait(task)
    task.note_transition_zero(ack)
    feed(task, 1_025_000_000, 4_025_000_000)
    assert task.success
    task.fail('ZERO_NOT_CONFIRMED')
    assert task.done and not task.success and task.success_stamp_ns is None
    assert task.reason == 'ZERO_NOT_CONFIRMED'


def test_limits_match_the_approved_fixed_profile():
    limits = TaskLimits()
    assert limits.force_limit_n == 5.0 and limits.reference_speed_mps == 0.05
    assert limits.move_stable_sim_sec == 1.0 and limits.hold_stable_sim_sec == 3.0
    assert limits.stable_position_m == 0.05 and limits.stable_velocity_mps == 0.10
    assert limits.stable_angle_rad == math.radians(2.0)
    assert limits.stable_angular_velocity_radps == 0.20
