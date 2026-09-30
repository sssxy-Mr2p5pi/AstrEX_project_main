"""Focused checks for the bounded ROS command source and stop behavior."""

import json
import math
import time

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
def trial(tmp_path, monkeypatch):
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
        TrialConfig(0.0, 0.0, 0.0, str(isaac)), tmp_path / 'trial')
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
