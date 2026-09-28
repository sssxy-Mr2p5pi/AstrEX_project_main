"""ROS subscription exposing a timestamp-aware Cartpole state cache."""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from astrex_ros_bridge.state_cache import CartpoleStateCache


class StateCacheNode(Node):
    """Keep the latest valid Cartpole joint state."""

    def __init__(self):
        super().__init__('state_cache')
        self.cache = CartpoleStateCache()
        self.x = None
        self.x_dot = None
        self.theta = None
        self.theta_dot = None
        self.header_stamp_ns = None
        self.last_update_monotonic_ns = None
        self._last_log_ns = None
        self.subscription = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10,
        )
        self._log_throttled('StateCache started.')

    @property
    def snapshot(self):
        """Return one coherent feedback sample, including both timestamps."""
        return self.cache.snapshot

    def _log_throttled(self, message, warning=False):
        now_ns = time.monotonic_ns()
        if (self._last_log_ns is not None
                and now_ns - self._last_log_ns < 1_000_000_000):
            return
        if warning:
            self.get_logger().warning(message)
        else:
            self.get_logger().info(message)
        self._last_log_ns = now_ns

    def is_fresh(self, max_age_sec=0.5):
        """Use local monotonic receive time, not simulation header time."""
        return self.cache.is_fresh(max_age_sec)

    def joint_state_callback(self, msg: JointState):
        accepted, error = self.cache.update(msg)
        if not accepted:
            self._log_throttled(error, warning=True)
            return
        snapshot = self.cache.snapshot
        self.x, self.x_dot, self.theta, self.theta_dot = snapshot.values
        self.header_stamp_ns = snapshot.stamp_ns
        self.last_update_monotonic_ns = snapshot.received_monotonic_ns
        self._log_throttled(
            f'x={self.x:+.3f} m, x_dot={self.x_dot:+.3f} m/s, '
            f'theta={self.theta:+.3f} rad, theta_dot={self.theta_dot:+.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StateCacheNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
