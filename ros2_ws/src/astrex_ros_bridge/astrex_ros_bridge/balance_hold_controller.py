"""Pure-Python upright Cartpole model and continuous-time BalanceHold LQR.

State: [cart position - target, cart velocity, pole angle, pole angular velocity].
Input: cart-slider force in N. The pole joint receives no commanded torque.

The Isaac Sim 5.1 Cartpole USD uses a +X slider axis rotated +90 degrees
about world Z, so positive slider displacement is world +Y. The pole rotates
about world +X; positive angle moves its centre of mass toward world -Y.
Consequently, relative to the slider coordinate, pole COM is x - l*sin(theta).

For cart mass M, pole mass m, COM distance l, COM inertia I about the hinge
axis, cart damping b, and pole damping c, let J = I + m*l*l. With theta=0
upright, the nonlinear planar equations are:

    (M+m)*xdd - m*l*cos(theta)*thetadd
        + m*l*sin(theta)*thetadot**2 = force - b*xdot
    -m*l*cos(theta)*xdd + J*thetadd
        - m*g*l*sin(theta) = -c*thetadot

Linearization at [target, 0, 0, 0], force=0 gives state_dot=A@state+B*force.
Rows 0 and 2 are position and angle kinematics. Rows 1 and 3 give cart and
pole accelerations, including the coupling, gravity, and damping terms.

Physical defaults come from Isaac Lab v2.3.2 CARTPOLE_CFG and the official
Isaac Sim 5.1 Cartpole USD and its referenced collision-geometry layer.
The USD does not author a nonzero diagonal inertia; I is derived for its
uniform 1 kg, 0.06 m by 1.0 m collision box about the +X COM axis. PhysX's
effective mass matrix has not yet been read back. The 400 N clamp matches
the Lab cart actuator effort_limit_sim; it is not a validated safety limit.
"""

from dataclasses import dataclass
import math
from typing import Sequence

import numpy as np
from scipy.linalg import solve_continuous_are


@dataclass(frozen=True)
class CartPoleParameters:
    cart_mass: float = 1.0
    pole_mass: float = 1.0
    pole_com_length: float = 0.47
    pole_com_inertia: float = (0.06**2 + 1.0**2) / 12.0
    gravity: float = 9.81
    cart_damping: float = 10.0
    pole_damping: float = 0.0


@dataclass(frozen=True)
class LQRWeights:
    """Initial tuning values, not validated in an Isaac closed loop."""

    x: float = 10.0
    x_dot: float = 1.0
    theta: float = 100.0
    theta_dot: float = 1.0
    force: float = 0.1


def _validate_parameters(params: CartPoleParameters) -> None:
    values = tuple(vars(params).values())
    if not all(math.isfinite(value) for value in values):
        raise ValueError("Cartpole parameters must be finite")
    if min(params.cart_mass, params.pole_mass, params.pole_com_length,
           params.pole_com_inertia, params.gravity) <= 0:
        raise ValueError("Mass, length, inertia, and gravity must be positive")
    if min(params.cart_damping, params.pole_damping) < 0:
        raise ValueError("Damping must be nonnegative")


def nonlinear_accelerations(
    x_dot: float, theta: float, theta_dot: float, force: float,
    params: CartPoleParameters = CartPoleParameters(),
) -> tuple[float, float]:
    """Return (cart acceleration, pole angular acceleration)."""
    _validate_parameters(params)
    if not all(math.isfinite(v) for v in (x_dot, theta, theta_dot, force)):
        raise ValueError("State and force must be finite")
    m = params.pole_mass
    h = m * params.pole_com_length
    total_mass = params.cart_mass + m
    j = params.pole_com_inertia + h * params.pole_com_length
    cosine = math.cos(theta)
    mass_matrix = np.array([[total_mass, -h * cosine], [-h * cosine, j]])
    right_side = np.array([
        force - params.cart_damping * x_dot - h * math.sin(theta) * theta_dot**2,
        h * params.gravity * math.sin(theta) - params.pole_damping * theta_dot,
    ])
    accelerations = np.linalg.solve(mass_matrix, right_side)
    return float(accelerations[0]), float(accelerations[1])


def linearized_dynamics(
    params: CartPoleParameters = CartPoleParameters(),
) -> tuple[np.ndarray, np.ndarray]:
    """Return continuous-time A (4x4) and B (4x1) at the upright point."""
    _validate_parameters(params)
    m = params.pole_mass
    total_mass = params.cart_mass + m
    h = m * params.pole_com_length
    j = params.pole_com_inertia + h * params.pole_com_length
    denominator = total_mass * j - h * h
    gravity_torque = h * params.gravity
    a = np.array([
        [0.0, 1.0, 0.0, 0.0],
        [0.0, -j * params.cart_damping / denominator,
         h * gravity_torque / denominator, -h * params.pole_damping / denominator],
        [0.0, 0.0, 0.0, 1.0],
        [0.0, -h * params.cart_damping / denominator,
         total_mass * gravity_torque / denominator,
         -total_mass * params.pole_damping / denominator],
    ])
    b = np.array([[0.0], [j / denominator], [0.0], [h / denominator]])
    return a, b


def controllability_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Return [B, AB, A²B, A³B] for a four-state, one-input system."""
    if a.shape != (4, 4) or b.shape != (4, 1):
        raise ValueError("A must be 4x4 and B must be 4x1")
    return np.column_stack((b, a @ b, a @ a @ b, a @ a @ a @ b))


def design_lqr_gain(
    params: CartPoleParameters = CartPoleParameters(),
    weights: LQRWeights = LQRWeights(),
) -> np.ndarray:
    """Solve the continuous algebraic Riccati equation; return K (1x4)."""
    a, b = linearized_dynamics(params)
    if np.linalg.matrix_rank(controllability_matrix(a, b)) != 4:
        raise ValueError("Cartpole model is not controllable (rank != 4)")
    weight_values = tuple(vars(weights).values())
    if not all(math.isfinite(value) and value > 0 for value in weight_values):
        raise ValueError("Q diagonal and R must be finite and positive")
    q = np.diag(weight_values[:4])
    r = np.array([[weights.force]])
    riccati = solve_continuous_are(a, b, q, r)
    gain = np.linalg.solve(r, b.T @ riccati)
    if gain.shape != (1, 4) or not np.all(np.isfinite(gain)):
        raise ValueError("Invalid LQR gain")
    return gain


class BalanceHoldController:
    """Compute a bounded cart force without ROS or simulation side effects."""

    def __init__(
        self,
        params: CartPoleParameters = CartPoleParameters(),
        weights: LQRWeights = LQRWeights(),
        max_force: float = 400.0,
        gain: np.ndarray | None = None,
    ) -> None:
        if not math.isfinite(max_force) or max_force <= 0:
            raise ValueError("max_force must be finite and positive")
        self.max_force = float(max_force)
        self.gain = np.asarray(
            design_lqr_gain(params, weights) if gain is None else gain,
            dtype=float,
        )
        if self.gain.shape != (1, 4) or not np.all(np.isfinite(self.gain)):
            raise ValueError("K must be a finite 1x4 matrix")

    def compute_force_from_state(self, state: Sequence[float], target_x: float) -> float:
        """Compute u=-K@[x-target, xdot, theta, thetadot], then clamp."""
        values = np.asarray(state, dtype=float)
        if values.shape != (4,):
            raise ValueError("State must have exactly four scalar values")
        if not np.all(np.isfinite(values)) or not math.isfinite(target_x):
            raise ValueError("State and target_x must be finite")
        error = values.copy()
        error[0] -= target_x
        raw_force = -(self.gain @ error).item()
        return float(np.clip(raw_force, -self.max_force, self.max_force))

    def compute_force(
        self, x: float, x_dot: float, theta: float, theta_dot: float,
        target_x: float,
    ) -> float:
        return self.compute_force_from_state((x, x_dot, theta, theta_dot), target_x)
