"""ROS-domain-isolated tests for the subscriber-only dry-run node."""

import math
import time

import pytest
import rclpy
from sensor_msgs.msg import JointState

from astrex_ros_bridge.balance_hold_dry_run_node import (
    BalanceHoldDryRunNode,
    extract_cartpole_state,
)


def _joint_state(x=0.0, x_dot=0.0, theta=0.0, theta_dot=0.0):
    msg = JointState()
    msg.name = ['cart_to_pole', 'slider_to_cart']  # Deliberately not cart-first.
    msg.position = [theta, x]
    msg.velocity = [theta_dot, x_dot]
    return msg


@pytest.fixture
def dry_run_node():
    rclpy.init()
    node = BalanceHoldDryRunNode()
    yield node
    node.destroy_node()
    rclpy.try_shutdown()


def test_joint_name_mapping_and_validation():
    state, error = extract_cartpole_state(_joint_state(0.3, 0.1, 0.02, -0.03))
    assert error is None
    assert state == (0.3, 0.1, 0.02, -0.03)

    missing = _joint_state()
    missing.name = ['slider_to_cart']
    assert extract_cartpole_state(missing)[0] is None
    duplicate = _joint_state()
    duplicate.name.append('slider_to_cart')
    assert extract_cartpole_state(duplicate)[0] is None
    incomplete = _joint_state()
    incomplete.velocity = [0.0]
    assert extract_cartpole_state(incomplete)[0] is None
    nonfinite = _joint_state(theta=float('nan'))
    assert extract_cartpole_state(nonfinite)[0] is None


def test_target_initializes_only_from_first_valid_state(dry_run_node):
    node = dry_run_node
    node.joint_state_callback(_joint_state(theta=float('inf')))
    assert node.target_x is None
    assert node.last_update_monotonic_ns is None

    node.joint_state_callback(_joint_state(x=0.25))
    assert node.target_x == 0.25
    first_update_ns = node.last_update_monotonic_ns

    node.joint_state_callback(_joint_state(x=0.5, theta=0.01))
    assert node.target_x == 0.25
    assert node.state == (0.5, 0.0, 0.01, 0.0)
    assert node.last_update_monotonic_ns >= first_update_ns

    node.joint_state_callback(_joint_state(x=0.9, theta=float('nan')))
    assert node.target_x == 0.25
    assert node.state == (0.5, 0.0, 0.01, 0.0)


def test_equilibrium_and_angle_recovery(dry_run_node):
    node = dry_run_node
    node.joint_state_callback(_joint_state(x=0.2))
    assert node.controller.compute_force(*node.state, node.target_x) == pytest.approx(0)
    node.joint_state_callback(_joint_state(x=0.2, theta=0.01))
    positive_theta_force = node.controller.compute_force(*node.state, node.target_x)
    node.joint_state_callback(_joint_state(x=0.2, theta=-0.01))
    negative_theta_force = node.controller.compute_force(*node.state, node.target_x)
    assert math.isfinite(positive_theta_force)
    assert positive_theta_force < 0 < negative_theta_force
    node.log_current_force()
    assert node._logged_count == 1


def test_stale_state_is_not_processed(dry_run_node):
    node = dry_run_node
    node.joint_state_callback(_joint_state())
    node.last_update_monotonic_ns = time.monotonic_ns() - 600_000_000
    node.log_current_force()
    assert node._logged_count == 0


def test_no_command_topic_publishers(dry_run_node):
    assert dry_run_node.get_publishers_info_by_topic('/joint_command') == []
    assert dry_run_node.get_publishers_info_by_topic('/astrex/raw_joint_command') == []
    assert dry_run_node.get_publishers_info_by_topic('/rosout') == []
