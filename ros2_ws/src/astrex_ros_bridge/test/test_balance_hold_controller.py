"""Isolated mathematical tests: no ROS node, Isaac process, or force publisher."""

import math

import numpy as np
import pytest

from astrex_ros_bridge.balance_hold_controller import (
    BalanceHoldController,
    CartPoleParameters,
    LQRWeights,
    controllability_matrix,
    design_lqr_gain,
    linearized_dynamics,
    nonlinear_accelerations,
)


def _state_derivative(state, force):
    _, x_dot, theta, theta_dot = state
    x_accel, theta_accel = nonlinear_accelerations(x_dot, theta, theta_dot, force)
    return np.array([x_dot, x_accel, theta_dot, theta_accel])


def test_asset_derived_parameters_and_equilibrium():
    params = CartPoleParameters()
    assert params.cart_mass == params.pole_mass == 1.0
    assert params.pole_com_length == 0.47
    assert params.pole_com_inertia == pytest.approx((0.06**2 + 1.0**2) / 12)
    assert nonlinear_accelerations(0, 0, 0, 0) == (0.0, 0.0)


def test_linearization_matches_nonlinear_finite_difference():
    a, b = linearized_dynamics()
    origin = np.zeros(4)
    epsilon = 1e-6
    for column in range(4):
        offset = np.zeros(4)
        offset[column] = epsilon
        derivative = (_state_derivative(origin + offset, 0)
                      - _state_derivative(origin - offset, 0)) / (2 * epsilon)
        np.testing.assert_allclose(derivative, a[:, column], rtol=1e-8, atol=1e-8)
    force_derivative = (_state_derivative(origin, epsilon)
                        - _state_derivative(origin, -epsilon)) / (2 * epsilon)
    np.testing.assert_allclose(force_derivative, b[:, 0], rtol=1e-8, atol=1e-8)
    assert b[1, 0] > 0  # Positive slider force accelerates toward +rail.
    assert b[3, 0] > 0  # Pole then leans toward -rail, i.e. positive theta.


def test_controllability_rank_four_and_stable_closed_loop():
    a, b = linearized_dynamics()
    assert np.linalg.matrix_rank(controllability_matrix(a, b)) == 4
    gain = design_lqr_gain()
    assert gain.shape == (1, 4)
    assert np.all(np.real(np.linalg.eigvals(a - b @ gain)) < 0)


def test_initial_weights_are_explicit_and_gain_is_reproducible():
    weights = LQRWeights()
    assert (weights.x, weights.x_dot, weights.theta, weights.theta_dot, weights.force) == (
        10.0, 1.0, 100.0, 1.0, 0.1,
    )
    np.testing.assert_allclose(
        design_lqr_gain(), [[-10.0, -27.45389966, 115.09023976, 26.65780946]],
        rtol=1e-7,
    )


def test_target_state_returns_zero_force():
    controller = BalanceHoldController()
    assert controller.compute_force(0.5, 0, 0, 0, target_x=0.5) == pytest.approx(0)


def test_pole_angle_sign_and_recovery_direction():
    controller = BalanceHoldController()
    positive_force = controller.compute_force(0, 0, 0.01, 0, target_x=0)
    negative_force = controller.compute_force(0, 0, -0.01, 0, target_x=0)
    assert math.isfinite(positive_force)
    assert positive_force < 0 < negative_force
    assert positive_force == pytest.approx(-negative_force)
    # Positive theta leans toward -rail. Negative force initially accelerates
    # cart toward the leaning pole and makes theta acceleration corrective.
    assert nonlinear_accelerations(0, 0.01, 0, positive_force)[1] < 0
    assert nonlinear_accelerations(0, -0.01, 0, negative_force)[1] > 0


def test_cart_position_sign_is_initially_nonminimum_phase():
    controller = BalanceHoldController()
    right_of_target = controller.compute_force(0.1, 0, 0, 0, target_x=0)
    left_of_target = controller.compute_force(-0.1, 0, 0, 0, target_x=0)
    # The upright pole needs a temporary tilt before the cart can return.
    # LQR initially pushes farther from target; this is not a sign error.
    assert right_of_target > 0 > left_of_target
    assert right_of_target == pytest.approx(-left_of_target)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_input_is_rejected(bad):
    controller = BalanceHoldController()
    with pytest.raises(ValueError):
        controller.compute_force(0, 0, bad, 0, target_x=0)
    with pytest.raises(ValueError):
        controller.compute_force(0, 0, 0, 0, target_x=bad)


def test_state_length_and_gain_dimensions_are_checked():
    controller = BalanceHoldController()
    with pytest.raises(ValueError):
        controller.compute_force_from_state([0, 0, 0], target_x=0)
    with pytest.raises(ValueError):
        BalanceHoldController(gain=np.zeros((4, 1)))
    with pytest.raises(ValueError):
        BalanceHoldController(gain=np.full((1, 4), np.nan))


def test_force_clamp_and_invalid_configuration():
    controller = BalanceHoldController(max_force=0.5)
    assert controller.compute_force(0, 0, 0.1, 0, target_x=0) == -0.5
    assert controller.compute_force(0, 0, -0.1, 0, target_x=0) == 0.5
    with pytest.raises(ValueError):
        BalanceHoldController(max_force=0)
    with pytest.raises(ValueError):
        design_lqr_gain(weights=LQRWeights(force=0))
