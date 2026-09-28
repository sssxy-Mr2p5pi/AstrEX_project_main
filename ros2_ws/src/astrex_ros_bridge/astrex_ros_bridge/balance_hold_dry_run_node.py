"""Observe real Cartpole joint states and print LQR force; publish nothing.

This is a diagnostic only. The pole inertia is geometry-derived rather than
read back from PhysX, Q/R are initial tuning values, and 400 N is an actuator
limit rather than an established safe closed-loop force limit.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.state_cache import extract_cartpole_state


class BalanceHoldDryRunNode(Node):
    """Subscribe to /joint_states and log a proposed force at about 5 Hz."""

    _LOG_PERIOD_SEC = 0.2
    _MAX_STATE_AGE_NS = 500_000_000

    def __init__(self):
        super().__init__('balance_hold_dry_run', enable_rosout=False)
        self.controller = BalanceHoldController()
        self.target_x = None
        self.state = None
        self.last_update_monotonic_ns = None
        self._last_warning_monotonic_ns = None
        self._limit_count = 0
        self._logged_count = 0

        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
        )
        self.timer = self.create_timer(self._LOG_PERIOD_SEC, self.log_current_force)
        self.get_logger().warning(
            'DRY RUN ONLY: no command publishers; this node cannot command Isaac.'
        )

    def _warn_throttled(self, message: str) -> None:
        now_ns = time.monotonic_ns()
        if (self._last_warning_monotonic_ns is None
                or now_ns - self._last_warning_monotonic_ns >= 1_000_000_000):
            self.get_logger().warning(message)
            self._last_warning_monotonic_ns = now_ns

    def joint_state_callback(self, msg: JointState) -> None:
        """Accept only complete finite feedback; initialize target once."""
        state, error = extract_cartpole_state(msg)
        if error is not None:
            self._warn_throttled(error)
            return

        now_ns = time.monotonic_ns()
        self.state = state
        self.last_update_monotonic_ns = now_ns
        if self.target_x is None:
            self.target_x = state[0]
            self.get_logger().info(
                f'BalanceHold target_x initialized to {self.target_x:+.6f} m'
            )

    def log_current_force(self) -> None:
        """Log one bounded calculation only while local feedback is fresh."""
        if self.state is None or self.target_x is None:
            self._warn_throttled('Waiting for a valid /joint_states message.')
            return
        age_ns = time.monotonic_ns() - self.last_update_monotonic_ns
        if age_ns > self._MAX_STATE_AGE_NS:
            self._warn_throttled('Latest /joint_states is stale; no force calculated.')
            return

        x, x_dot, theta, theta_dot = self.state
        force = self.controller.compute_force(
            x=x, x_dot=x_dot, theta=theta, theta_dot=theta_dot,
            target_x=self.target_x,
        )
        at_limit = abs(force) >= self.controller.max_force - 1e-9
        self._logged_count += 1
        self._limit_count += int(at_limit)
        self.get_logger().info(
            f'x={x:+.6f} m x_dot={x_dot:+.6f} m/s '
            f'theta={theta:+.6f} rad ({math.degrees(theta):+.3f} deg) '
            f'theta_dot={theta_dot:+.6f} rad/s '
            f'target_x={self.target_x:+.6f} m '
            f'LQR force={force:+.6f} N '
            f'at_400N_limit={at_limit} '
            f'limit_samples={self._limit_count}/{self._logged_count}'
        )


def main(args=None):
    """Run the subscriber-only diagnostic until Ctrl+C."""
    rclpy.init(args=args)
    node = None
    try:
        node = BalanceHoldDryRunNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
