"""Cartpole feedback parsing and timestamp-aware cache shared by ROS nodes."""

from dataclasses import dataclass
import math
import time

from sensor_msgs.msg import JointState


CART_JOINT = 'slider_to_cart'
POLE_JOINT = 'cart_to_pole'


@dataclass(frozen=True)
class StateSnapshot:
    """Four values from one JointState, with two distinct time domains."""

    x: float
    x_dot: float
    theta: float
    theta_dot: float
    stamp_ns: int  # Message header: simulation time for the Isaac publisher.
    received_monotonic_ns: int  # Local receive time: for freshness/timeouts.

    @property
    def values(self) -> tuple[float, float, float, float]:
        return self.x, self.x_dot, self.theta, self.theta_dot


def extract_cartpole_state(
    msg: JointState,
) -> tuple[tuple[float, float, float, float] | None, str | None]:
    """Extract a coherent state by joint name; never assume array ordering."""
    if msg.name.count(CART_JOINT) != 1 or msg.name.count(POLE_JOINT) != 1:
        return None, 'Required joints are missing or duplicated.'
    cart_index = msg.name.index(CART_JOINT)
    pole_index = msg.name.index(POLE_JOINT)
    if (max(cart_index, pole_index) >= len(msg.position)
            or max(cart_index, pole_index) >= len(msg.velocity)):
        return None, 'JointState position/velocity data is incomplete.'
    state = (
        float(msg.position[cart_index]),
        float(msg.velocity[cart_index]),
        float(msg.position[pole_index]),
        float(msg.velocity[pole_index]),
    )
    if not all(math.isfinite(value) for value in state):
        return None, 'JointState contains NaN or Inf.'
    return state, None


def extract_stamp_ns(msg: JointState) -> tuple[int | None, str | None]:
    """Read a nonnegative ROS timestamp without using the local clock."""
    sec = msg.header.stamp.sec
    nanosec = msg.header.stamp.nanosec
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return None, 'JointState header.stamp is invalid.'
    return sec * 1_000_000_000 + nanosec, None


class CartpoleStateCache:
    """Accept only valid, strictly newer messages; retain the last snapshot."""

    def __init__(self) -> None:
        self.snapshot: StateSnapshot | None = None
        self.last_rejection: str | None = None

    def update(
        self, msg: JointState, received_monotonic_ns: int | None = None,
    ) -> tuple[bool, str | None]:
        state, error = extract_cartpole_state(msg)
        if error is None:
            stamp_ns, error = extract_stamp_ns(msg)
        if error is None and self.snapshot is not None:
            if stamp_ns <= self.snapshot.stamp_ns:
                error = 'JointState header.stamp did not advance.'
        if error is not None:
            self.last_rejection = error
            return False, error
        if received_monotonic_ns is None:
            received_monotonic_ns = time.monotonic_ns()
        if received_monotonic_ns < 0:
            raise ValueError('received_monotonic_ns must be nonnegative')
        self.snapshot = StateSnapshot(*state, stamp_ns, received_monotonic_ns)
        self.last_rejection = None
        return True, None

    def is_fresh(
        self, max_age_sec: float, now_monotonic_ns: int | None = None,
    ) -> bool:
        if not math.isfinite(max_age_sec) or max_age_sec < 0:
            raise ValueError('max_age_sec must be finite and nonnegative')
        if self.snapshot is None:
            return False
        if now_monotonic_ns is None:
            now_monotonic_ns = time.monotonic_ns()
        age_ns = now_monotonic_ns - self.snapshot.received_monotonic_ns
        return 0 <= age_ns <= max_age_sec * 1_000_000_000
