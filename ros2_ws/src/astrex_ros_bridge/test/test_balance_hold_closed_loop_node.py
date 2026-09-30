"""Focused checks for the bounded ROS command source and stop behavior."""

import json
import math
import time
from dataclasses import replace

import pytest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from astrex_ros_bridge.balance_hold_closed_loop_node import (
    FORCE_LIMIT_N, BalanceHoldClosedLoopNode, TrialConfig,
)
from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.state_cache import StateSnapshot


def feedback(stamp_ns, x=0.0, theta=0.0, x_dot=0.0, theta_dot=0.0):
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.name = ['cart_to_pole', 'slider_to_cart']
    msg.position = [theta, x]
    msg.velocity = [theta_dot, x_dot]
    return msg


@pytest.fixture
def trial(tmp_path, monkeypatch, request):
    rclpy.init()
    isaac = tmp_path / 'isaac'
    isaac.mkdir()
    (isaac / 'ros_events.jsonl').write_text('')
    listener = Node('fake_isaac_subscriber', enable_rosout=False)
    received = []
    listener.create_subscription(
        JointState, '/joint_command', lambda msg: received.append(msg), 10)
    monkeypatch.setattr(
        BalanceHoldClosedLoopNode, '_verify_isaac_identity',
        lambda self: setattr(self, 'isaac_graph', '/World/AstrEXROSGraph_test'))
    node = BalanceHoldClosedLoopNode(
        TrialConfig(0.0, 0.0, 0.0, str(isaac), **getattr(request, 'param', {})),
        tmp_path / 'trial')
    yield node, listener, received, isaac
    if not node.trace.closed:
        node.request_stop('TEST_END')
        node.finish()
    node.destroy_node()
    listener.destroy_node()
    rclpy.try_shutdown()


def initialize_trial(node, isaac, theta=0.0):
    with (isaac / 'ros_events.jsonl').open('a') as stream:
        stream.write(json.dumps({
            'event': 'trial_initialized',
            'state': {'simulation_time': 0.0, 'reset_count': 0},
        }) + '\n')
    node._poll_isaac_events()
    node._on_feedback(feedback(1_000_000_000, theta=theta))
    assert node.phase == 'CONTROL'


def test_fixed_cap_and_nonfinite_raw_guard():
    config = TrialConfig(0.0, 0.0, 0.0, '/tmp/example')
    config.validate()
    with pytest.raises(ValueError):
        TrialConfig(0.0, 0.0, 0.0, '/tmp/example', force_limit_n=400).validate()
    controller = BalanceHoldController(max_force=FORCE_LIMIT_N)
    assert math.isfinite(controller.compute_raw_force_from_state(
        (0.0, 0.0, math.radians(1), 0.0), 0.0))
    with pytest.raises(ValueError, match='finite'):
        controller.compute_raw_force_from_state((1e308, 1e308, 1e308, 1e308), 0.0)


def test_initial_feedback_and_clipped_publication(trial):
    node, listener, received, isaac = trial
    marker = json.loads((isaac / 'trial_ready.json').read_text())
    assert marker['x0'] == marker['theta0'] == marker['hold_position'] == 0.0
    node._on_feedback(feedback(5))
    assert node.phase == 'WAIT_INITIALIZATION'
    initialize_trial(node, isaac)
    node._publish(9.0, 'unit', node.cache.snapshot, raw=9.0)
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert received
    assert received[-1].name == ['slider_to_cart', 'cart_to_pole']
    assert received[-1].effort[1] == 0.0
    assert max(abs(msg.effort[0]) for msg in received) <= 5.0
    node.request_stop('UNIT_DONE')
    node.finish()
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert received[-1].effort[0] == 0.0
    assert json.loads((node.trial_dir / 'result.json').read_text())['reason'] == 'UNIT_DONE'


def test_invalid_feedback_and_boundary_stop_with_zero(trial):
    node, listener, received, isaac = trial
    initialize_trial(node, isaac)
    bad = feedback(1_010_000_000)
    bad.velocity = [float('nan'), 0.0]
    node._on_feedback(bad)
    assert node.stop_reason == 'INVALID_FEEDBACK'
    node.finish()
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert received[-1].effort[0] == 0.0


def test_saturation_and_stale_feedback_have_distinct_reasons(trial):
    node, listener, received, isaac = trial
    initialize_trial(node, isaac)
    for stamp in (2_000_000_000, 2_600_000_001):
        snapshot = StateSnapshot(0.0, 0.0, 0.03, 0.1, stamp, time.monotonic_ns())
        node.cache.snapshot = snapshot
        node._control_once(snapshot)
    assert node.stop_reason == 'SATURATION_TIMEOUT'
    assert node.max_saturation_sim_sec > 0.5
    node.finish()
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert received[-1].effort[0] == 0.0


def test_isaac_boundary_event_has_priority(trial):
    node, listener, received, isaac = trial
    initialize_trial(node, isaac)
    with (isaac / 'ros_events.jsonl').open('a') as stream:
        stream.write(json.dumps({'event': 'boundary', 'reset_count': 0}) + '\n')
    node._tick()
    assert node.stop_reason == 'ISAAC_BOUNDARY'
    node.finish()
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert received[-1].effort[0] == 0.0


def feed_stable(node, start_ns, end_ns):
    for stamp in range(start_ns, end_ns + 1, 25_000_000):
        node._on_feedback(feedback(stamp))


def test_online_success_needs_three_new_sim_seconds_and_stops_output(trial):
    node, listener, received, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 3_975_000_000)
    assert node.stop_reason is None
    assert node.stable_duration_sim_sec == pytest.approx(2.975)
    node._on_feedback(feedback(4_000_000_000))
    assert node.stop_reason == 'SUCCESS'
    assert node.success_stamp_ns == 4_000_000_000
    commands_before = node.control_count
    node._on_feedback(feedback(4_025_000_000, theta=0.1))
    node._tick()
    assert node.control_count == commands_before
    result = node.finish()
    assert result['success_state']['stamp_ns'] == result['success_stamp_ns']
    assert result['stable_window'] == {
        'start_stamp_ns': 1_000_000_000, 'end_stamp_ns': 4_000_000_000,
        'duration_sim_sec': 3.0,
    }
    rclpy.spin_once(listener, timeout_sec=0.2)
    assert list(received[-1].effort) == [0.0, 0.0]
    rows = [json.loads(line) for line in (node.trial_dir / 'trace.jsonl').read_text().splitlines()]
    stop_index = next(i for i, row in enumerate(rows) if row.get('phase') == 'stop')
    assert all(row['f_cmd'] == 0.0 for row in rows[stop_index:] if row['event'] == 'command')


def test_duplicate_feedback_and_timer_cannot_accumulate_stability(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    for _ in range(10):
        node._on_feedback(feedback(1_000_000_000))
        node._tick()
    assert node.stable_duration_sim_sec == 0.0
    assert node.stop_reason is None


@pytest.mark.parametrize('fields', [
    {'x': 0.06}, {'x_dot': 0.11}, {'theta': math.radians(2.1)}, {'theta_dot': 0.21},
])
def test_any_stability_condition_break_restarts_full_window(trial, fields):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 1_975_000_000)
    node._on_feedback(feedback(2_000_000_000, **fields))
    assert node.stable_start_stamp_ns is None
    feed_stable(node, 2_025_000_000, 5_000_000_000)
    assert node.stop_reason is None
    node._on_feedback(feedback(5_025_000_000))
    assert node.stop_reason == 'SUCCESS'
    assert node.stable_start_stamp_ns == 2_025_000_000


def test_gap_restarts_window_and_time_regression_faults(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 3_975_000_000)
    node._on_feedback(feedback(4_075_000_000))
    assert node.stop_reason is None
    assert node.stable_start_stamp_ns == 4_075_000_000
    assert node.stable_duration_sim_sec == 0.0
    node._on_feedback(feedback(4_050_000_000))
    assert node.stop_reason == 'STATE_TIME_REGRESSION'


def test_pending_boundary_wins_over_last_success_feedback(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 3_975_000_000)
    with (isaac / 'ros_events.jsonl').open('a') as stream:
        stream.write(json.dumps({'event': 'boundary', 'reset_count': 0}) + '\n')
    node._on_feedback(feedback(4_000_000_000))
    assert node.stop_reason == 'ISAAC_BOUNDARY'
    assert node.success_stamp_ns is None


@pytest.mark.parametrize('config_change,state,reason', [
    ({'abort_angle_rad': 0.02}, {'theta': 0.03}, 'TRIAL_BOUNDARY'),
    ({'abort_position_error_m': 0.01}, {'x': 0.02}, 'TRIAL_BOUNDARY'),
    ({'max_sim_sec': 0.5}, {}, 'SIM_TIMEOUT'),
    ({'state_timeout_local_sec': 0.01}, {}, 'STALE_FEEDBACK'),
    ({'max_local_sec': 0.01}, {}, 'LOCAL_TIMEOUT'),
])
def test_runtime_uses_configured_exit_limits(trial, config_change, state, reason):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    node.config = replace(node.config, **config_change)
    node.config.validate()
    stamp = 1_500_000_000 if reason == 'SIM_TIMEOUT' else 1_025_000_000
    received_ns = time.monotonic_ns()
    if reason == 'STALE_FEEDBACK':
        received_ns -= 20_000_000
    if reason == 'LOCAL_TIMEOUT':
        node.active_start_local_ns = time.monotonic_ns() - 20_000_000
    snapshot = StateSnapshot(state.get('x', 0.0), 0.0, state.get('theta', 0.0), 0.0,
                             stamp, received_ns)
    node.cache.snapshot = snapshot
    node._control_once(snapshot)
    assert node.stop_reason == reason
    assert node.success_stamp_ns is None


def test_configured_saturation_and_gap_limits_are_used(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    node.config = replace(node.config, max_state_gap_sim_sec=0.01, max_saturation_sim_sec=0.01)
    node._on_feedback(feedback(1_025_000_000))
    assert node.stable_start_stamp_ns == 1_025_000_000
    for stamp in (1_050_000_000, 1_075_000_000):
        snapshot = StateSnapshot(0.0, 0.0, 0.03, 0.1, stamp, time.monotonic_ns())
        node.cache.snapshot = snapshot
        node._control_once(snapshot)
    assert node.stop_reason == 'SATURATION_TIMEOUT'


def test_extended_initial_angles_keep_fixed_protection():
    for angle in (-8, -5, -3, 3, 5, 8):
        TrialConfig(0.3, math.radians(angle), 0.3, '/tmp/example').validate()
    for changes in ({'theta0': math.radians(10.1)}, {'x0': 0.251},
                    {'max_sim_sec': 0.0}, {'max_state_gap_sim_sec': 0.051},
                    {'state_timeout_local_sec': 0.201}, {'stable_window_sim_sec': 2.999},
                    {'abort_angle_rad': math.radians(11)}, {'max_saturation_sim_sec': 0.501}):
        config = TrialConfig(0.0, 0.0, 0.0, '/tmp/example')
        with pytest.raises(ValueError):
            replace(config, **changes).validate()


def test_b_recovery_requirement_is_kept(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    node.config = replace(node.config, theta0=math.radians(2))
    node._on_feedback(feedback(1_025_000_000, theta=math.radians(1.1)))
    assert node.stable_start_stamp_ns is None
    node._on_feedback(feedback(1_050_000_000, theta=math.radians(0.9)))
    assert node.stable_start_stamp_ns == 1_050_000_000


def test_zero_publish_failure_cannot_return_success(trial, monkeypatch):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 3_975_000_000)

    def failed_publish(msg):
        raise RuntimeError('injected publication failure')

    monkeypatch.setattr(node.publisher, 'publish', failed_publish)
    node._on_feedback(feedback(4_000_000_000))
    assert node.stop_reason == 'ZERO_PUBLISH_FAILED'
    assert node.success_stamp_ns is None
    assert node.finish()['reason'] == 'ZERO_PUBLISH_FAILED'


@pytest.mark.parametrize('trial', [
    {'task': 'move', 'target': 0.3}, {'task': 'move', 'target': -0.3},
], indirect=True)
def test_move_reference_uses_sim_time_and_cannot_finish_before_endpoint(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    assert node.reference.duration_sec == pytest.approx(11.25)
    assert node.reference.position(node.initial_stamp_ns) == node.initial_state.x
    assert node._active_local_limit() == max(60.0, 3 * (11.25 + 8.0))
    # Even an already-nearby final target cannot bypass the reference duration.
    node.config = replace(node.config, stable_position_m=0.05)
    sign = 1 if node.config.target > 0 else -1
    for stamp in range(1_025_000_000, node.reference.end_stamp_ns + 1, 25_000_000):
        position = node.reference.position(stamp)
        node._on_feedback(feedback(stamp, x=position))
    assert node.stop_reason is None
    assert node.stable_duration_sim_sec == 0.0
    for stamp in range(node.reference.end_stamp_ns + 25_000_000,
                       node.reference.end_stamp_ns + 1_000_000_001, 25_000_000):
        node._on_feedback(feedback(stamp, x=sign * 0.3))
    assert node.stop_reason == 'MOVE_SUCCESS'
    assert node.move_stable_window['duration_sim_sec'] == 1.0
    assert node.success_stamp_ns == node.reference.end_stamp_ns + 1_000_000_000


@pytest.mark.parametrize('trial', [{'task': 'move', 'target': 0.5}], indirect=True)
def test_move_boundaries_follow_reference_not_final_target(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    # The initial final-target error is 0.5 m and is valid for a moving task.
    assert node.stop_reason is None
    assert node.path_corridor == (-0.25, 0.75)
    assert node._stable(node.cache.snapshot) is False
    node._on_feedback(feedback(1_025_000_000, x=-0.251))
    assert node.stop_reason == 'TRIAL_BOUNDARY'


@pytest.mark.parametrize('trial', [{'task': 'move', 'target': 0.01}], indirect=True)
def test_already_inside_target_tolerance_still_waits_for_reference(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 2_975_000_000)
    assert node.stop_reason is None
    assert node.stable_start_stamp_ns is None
    node._on_feedback(feedback(3_000_000_000))
    assert node.stable_start_stamp_ns == node.reference.end_stamp_ns
    assert node.stop_reason is None


@pytest.mark.parametrize('trial', [{'task': 'move_then_hold', 'target': 0.0}], indirect=True)
def test_sequential_zero_new_s1_and_separate_hold_window(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 2_000_000_000)
    assert node.stop_reason is None and node.phase == 'WAIT_S1'
    assert node.move_success_stamp_ns == 2_000_000_000
    controls_before = node.control_count
    node._tick()
    node._on_feedback(feedback(2_000_000_000))
    assert node.control_count == controls_before
    assert node.s1_state is None
    node._on_feedback(feedback(2_025_000_000))
    assert node.phase == 'CONTROL' and node.task_phase == 'HOLD'
    assert node.s1_state.stamp_ns > node.move_success_stamp_ns
    assert node.s1_state.received_monotonic_ns > node.transition_zero_local_monotonic_ns
    assert node.stable_duration_sim_sec == 0.0
    feed_stable(node, 2_050_000_000, 5_000_000_000)
    assert node.stop_reason is None
    node._on_feedback(feedback(5_025_000_000))
    assert node.stop_reason == 'DEMO_SUCCESS'
    result = node.finish()
    assert result['move_stable_window']['duration_sim_sec'] == 1.0
    assert result['hold_stable_window']['duration_sim_sec'] == 3.0
    assert result['hold_start_stamp_ns'] == result['s1_state']['stamp_ns']
    rows = [json.loads(line) for line in (node.trial_dir / 'trace.jsonl').read_text().splitlines()]
    transition = next(i for i, row in enumerate(rows) if row.get('phase') == 'transition_zero')
    hold_start = next(i for i, row in enumerate(rows) if row['event'] == 'hold_started')
    assert all(row['event'] != 'command' or row['f_cmd'] == 0.0
               for row in rows[transition:hold_start])


@pytest.mark.parametrize('trial', [{'task': 'move_then_hold', 'target': 0.0}], indirect=True)
def test_s1_timeout_cannot_start_hold(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 2_000_000_000)
    node.transition_zero_local_monotonic_ns = time.monotonic_ns() - 501_000_000
    node._tick()
    assert node.stop_reason == 'S1_TIMEOUT'
    assert node.s1_state is None


@pytest.mark.parametrize('trial', [{'task': 'move_then_hold', 'target': 0.0}], indirect=True)
def test_pending_fault_wins_over_move_success(trial):
    node, _, _, isaac = trial
    initialize_trial(node, isaac)
    feed_stable(node, 1_025_000_000, 1_975_000_000)
    with (isaac / 'ros_events.jsonl').open('a') as stream:
        stream.write(json.dumps({'event': 'boundary'}) + '\n')
    node._on_feedback(feedback(2_000_000_000))
    assert node.stop_reason == 'ISAAC_BOUNDARY'
    assert node.move_success_stamp_ns is None and node.s1_state is None


@pytest.mark.parametrize('trial', [
    {'task': 'move', 'target': 0.3, 'dry_run': True},
    {'task': 'move', 'target': -0.3, 'dry_run': True},
    {'task': 'move', 'target': 0.5, 'dry_run': True},
], indirect=True)
def test_strict_dry_run_never_creates_publisher_or_releases_marker(trial):
    node, listener, received, isaac = trial
    assert node.publisher is None
    assert not (isaac / 'trial_ready.json').exists()
    node._on_feedback(feedback(1_000_000_000))
    assert node.phase == 'CONTROL'
    for stamp in range(1_025_000_000,
                       node.reference.end_stamp_ns + 1_000_000_001, 25_000_000):
        node._on_feedback(feedback(stamp))
        node._tick()
    assert node.stop_reason == 'DRY_RUN_PASS'
    assert node.success_stamp_ns is None
    result = node.finish()
    rclpy.spin_once(listener, timeout_sec=0.01)
    assert received == []
    assert result['publisher_created'] is False
    assert result['max_published_force_n'] == 0.0
    assert result['success_state'] is None
    rows = [json.loads(line) for line in (node.trial_dir / 'trace.jsonl').read_text().splitlines()]
    assert not any(row['event'] == 'command' for row in rows)
    calculations = [row for row in rows if row['event'] == 'calculation']
    assert calculations and all(abs(row['f_cmd']) <= 5.0 for row in calculations)
    assert all(row['pole_torque'] == 0.0 for row in calculations)
    for row in calculations:
        state_error = [row['x'] - row['x_ref'], row['x_dot'], row['theta'], row['theta_dot']]
        independent_raw = -sum(k * e for k, e in zip(node.controller.gain[0], state_error))
        assert row['f_raw'] == pytest.approx(independent_raw)
        assert row['f_cmd'] == pytest.approx(max(-5.0, min(5.0, independent_raw)))


@pytest.mark.parametrize('trial', [{'task': 'move', 'target': 0.3, 'dry_run': True}], indirect=True)
def test_dry_run_still_faults_on_time_regression_and_invalid_state(trial):
    node, _, _, _ = trial
    node._on_feedback(feedback(1_000_000_000))
    node._on_feedback(feedback(1_025_000_000))
    node._on_feedback(feedback(1_010_000_000))
    assert node.stop_reason == 'STATE_TIME_REGRESSION'
    assert node.publisher is None


def test_move_requires_finite_target_and_keeps_all_force_protection():
    for changes in ({'task': 'move'}, {'task': 'move', 'target': math.inf},
                    {'task': 'move', 'target': 0.5, 'reference_speed_mps': 0.051},
                    {'task': 'move', 'target': 0.5, 'force_limit_n': 5.01}):
        with pytest.raises(ValueError):
            replace(TrialConfig(0, 0, 0, '/tmp/example'), **changes).validate()
    config = TrialConfig(0, 0, 0, '/tmp/example', task='move', target=0.5)
    config.validate()
