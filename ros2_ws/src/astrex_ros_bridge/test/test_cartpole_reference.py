"""Focused reference and fresh-feedback timing tests; no ROS or simulation."""

import math

import pytest

from astrex_ros_bridge.cartpole_reference import ContinuousStability, QuinticReference


@pytest.mark.parametrize('target,expected_duration', [
    (0.3, 11.25), (-0.3, 11.25), (0.5, 18.75), (-0.5, 18.75),
])
def test_quintic_duration_direction_speed_and_fixed_endpoint(target, expected_duration):
    start = 2_000_000_000
    reference = QuinticReference(0.0, target, start)
    assert reference.duration_sec == pytest.approx(expected_duration, abs=1e-9)
    assert reference.end_stamp_ns == start + round(expected_duration * 1e9)
    stamps = [start + round(reference.duration_sec * 1e9 * i / 1000) for i in range(1001)]
    positions = [reference.position(stamp) for stamp in stamps]
    velocities = [reference.velocity(stamp) for stamp in stamps]
    assert positions[0] == 0.0 and positions[-1] == target
    assert all(min(0.0, target) <= position <= max(0.0, target) for position in positions)
    assert all((b - a) * target >= 0 for a, b in zip(positions, positions[1:]))
    assert max(abs(velocity) for velocity in velocities) <= 0.05 + 1e-12
    # Use sampled differences as an independent check of the speed bound.
    assert max(abs((b - a) / ((tb - ta) / 1e9))
               for a, b, ta, tb in zip(positions, positions[1:], stamps, stamps[1:])) <= 0.05 + 1e-8
    assert velocities[0] == velocities[-1] == 0.0
    assert not reference.finished(reference.end_stamp_ns - 1)
    assert reference.finished(reference.end_stamp_ns)
    assert reference.position(reference.end_stamp_ns + 20_000_000_000) == target
    assert reference.velocity(reference.end_stamp_ns + 20_000_000_000) == 0.0


def test_reference_uses_real_origin_and_simulation_time_only():
    reference = QuinticReference(-0.2, 0.3, 8_000_000_000)
    assert reference.position(0) == reference.position(reference.start_stamp_ns) == -0.2
    assert reference.velocity(reference.start_stamp_ns) == 0.0
    midpoint = reference.start_stamp_ns + round(reference.duration_sec * 5e8)
    assert reference.position(midpoint) == pytest.approx(0.05)
    assert reference.position(midpoint) == reference.position(midpoint)
    assert reference.velocity(midpoint) == pytest.approx(0.05)


def test_small_displacement_and_no_displacement():
    small = QuinticReference(0.0, 0.01, 0)
    assert small.duration_sec == 2.0
    assert abs(small.velocity(1_000_000_000)) < 0.05
    fixed = QuinticReference(0.3, 0.3, 2_000_000_000)
    assert fixed.duration_sec == 0.0
    assert fixed.finished(fixed.start_stamp_ns)
    assert not fixed.finished(fixed.start_stamp_ns - 1)
    assert fixed.position(0) == fixed.position(20_000_000_000) == 0.3
    assert fixed.velocity(20_000_000_000) == 0.0


@pytest.mark.parametrize('speed', [0.05, 0.03])
def test_reference_speed_configuration_changes_duration(speed):
    reference = QuinticReference(0.0, 0.5, 0, max_speed_mps=speed)
    assert reference.duration_sec == pytest.approx(1.875 * 0.5 / speed)
    assert abs(reference.velocity(reference.end_stamp_ns // 2)) <= speed + 1e-12


@pytest.mark.parametrize('kwargs', [
    {'x_start': math.nan}, {'target': math.inf}, {'max_speed_mps': 0.0},
    {'max_speed_mps': -0.05}, {'min_duration_sec': 0.0}, {'start_stamp_ns': -1},
    {'start_stamp_ns': True}, {'start_stamp_ns': 0.5},
])
def test_reference_rejects_invalid_configuration(kwargs):
    values = {'x_start': 0.0, 'target': 0.3, 'start_stamp_ns': 0}
    values.update(kwargs)
    with pytest.raises(ValueError):
        QuinticReference(**values)


@pytest.mark.parametrize('required', [1.0, 3.0])
def test_new_feedback_accumulates_exact_move_and_hold_windows(required):
    stable = ContinuousStability(required)
    start = 2_000_000_000
    assert not stable.update(start, True)
    steps = round(required / 0.01)
    for i in range(1, steps):
        assert not stable.update(start + i * 10_000_000, True)
    assert stable.update(start + steps * 10_000_000, True)
    assert stable.window() == {'start_stamp_ns': start,
                               'end_stamp_ns': start + round(required * 1e9),
                               'duration_sim_sec': required}


def test_duplicate_and_old_feedback_do_not_accumulate_or_requalify():
    stable = ContinuousStability(1.0)
    stable.update(0, True)
    stable.update(20_000_000, True)
    saved = stable.window()
    assert not stable.update(20_000_000, False)
    assert not stable.update(10_000_000, True)
    assert stable.window() == saved
    for stamp in range(40_000_000, 1_000_000_001, 20_000_000):
        success = stable.update(stamp, True)
    assert success
    assert not stable.update(1_000_000_000, True)
    assert stable.duration_sim_sec == 1.0


def test_condition_failure_and_excess_feedback_gap_restart_the_window():
    stable = ContinuousStability(1.0)
    stable.update(0, True)
    stable.update(40_000_000, True)
    assert not stable.update(50_000_000, False)
    assert stable.window() is None and stable.duration_sim_sec == 0.0
    stable.update(60_000_000, True)
    assert stable.start_stamp_ns == 60_000_000
    assert not stable.update(110_000_000, True)  # Exact maximum gap is allowed.
    assert stable.duration_sim_sec == 0.05
    assert not stable.update(160_000_001, True)
    assert stable.start_stamp_ns == 160_000_001
    assert stable.duration_sim_sec == 0.0
    stable.reset()
    assert stable.window() is None
    assert stable.last_stamp_ns is None and stable.duration_sim_sec == 0.0


@pytest.mark.parametrize('duration,gap', [
    (0.0, 0.05), (-1.0, 0.05), (math.nan, 0.05),
    (1.0, 0.0), (1.0, math.inf),
])
def test_invalid_stability_configuration_is_rejected(duration, gap):
    with pytest.raises(ValueError):
        ContinuousStability(duration, gap)
