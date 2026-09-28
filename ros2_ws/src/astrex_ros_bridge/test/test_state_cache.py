"""Coherent joint-state and time-domain tests for the shared cache."""

import math

import pytest
import rclpy
from sensor_msgs.msg import JointState

from astrex_ros_bridge.state_cache import CartpoleStateCache
from astrex_ros_bridge.state_cache_node import StateCacheNode


def joint_state(stamp_ns=1, x=0.2, x_dot=0.03, theta=-0.01, theta_dot=0.04):
    msg = JointState()
    msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    msg.name = ['cart_to_pole', 'slider_to_cart']
    msg.position = [theta, x]
    msg.velocity = [theta_dot, x_dot]
    return msg


def test_snapshot_uses_names_and_keeps_two_clocks():
    cache = CartpoleStateCache()
    accepted, error = cache.update(joint_state(2_500_000_008), 900_000_000_000)
    assert accepted and error is None
    assert cache.snapshot.values == (0.2, 0.03, -0.01, 0.04)
    assert cache.snapshot.stamp_ns == 2_500_000_008
    assert cache.snapshot.received_monotonic_ns == 900_000_000_000
    assert cache.is_fresh(0.5, 900_400_000_000)
    assert not cache.is_fresh(0.5, 900_600_000_000)


@pytest.mark.parametrize('change', [
    lambda msg: msg.name.pop(),
    lambda msg: msg.name.append('slider_to_cart'),
    lambda msg: msg.position.pop(),
    lambda msg: msg.velocity.pop(),
    lambda msg: msg.position.__setitem__(0, float('nan')),
    lambda msg: msg.velocity.__setitem__(1, float('inf')),
])
def test_invalid_feedback_cannot_replace_or_refresh_snapshot(change):
    cache = CartpoleStateCache()
    assert cache.update(joint_state(1), 100)[0]
    original = cache.snapshot
    bad = joint_state(2)
    change(bad)
    accepted, error = cache.update(bad, 200)
    assert not accepted and error
    assert cache.snapshot is original
    assert not cache.is_fresh(0, 101)


def test_duplicate_and_out_of_order_stamp_do_not_count_as_progress():
    cache = CartpoleStateCache()
    assert cache.update(joint_state(100), 1_000)[0]
    original = cache.snapshot
    for stamp in (100, 99):
        accepted, reason = cache.update(joint_state(stamp, x=999), 2_000)
        assert not accepted
        assert 'did not advance' in reason
        assert cache.snapshot is original
    assert cache.update(joint_state(101, x=0.3), 3_000)[0]
    assert cache.snapshot.x == 0.3
    assert cache.snapshot.received_monotonic_ns == 3_000


def test_invalid_timestamp_and_freshness_limits():
    cache = CartpoleStateCache()
    msg = joint_state()
    msg.header.stamp.sec = -1
    assert not cache.update(msg)[0]
    assert cache.snapshot is None
    assert not cache.is_fresh(0.5)
    for max_age in (-1, math.inf, math.nan):
        with pytest.raises(ValueError):
            cache.is_fresh(max_age)


def test_ros_node_exposes_one_snapshot_and_rejects_repeated_time():
    rclpy.init()
    node = StateCacheNode()
    try:
        node.joint_state_callback(joint_state(3_000_000_004))
        snapshot = node.snapshot
        assert snapshot.values == (0.2, 0.03, -0.01, 0.04)
        assert node.header_stamp_ns == 3_000_000_004
        assert node.last_update_monotonic_ns == snapshot.received_monotonic_ns
        node.joint_state_callback(joint_state(3_000_000_004, x=99))
        assert node.snapshot is snapshot
        assert node.x == 0.2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
