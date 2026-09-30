"""Bounded, single-publisher Cartpole BalanceHold experiment.

This is a Step 3D test entry, not a robot safety controller or ROS Action.
The Isaac trial profile initializes the state only after this node is ready.
"""

import argparse
from dataclasses import asdict, dataclass
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.state_cache import CART_JOINT, POLE_JOINT, CartpoleStateCache


FORCE_LIMIT_N = 5.0
ABORT_ANGLE_RAD = math.radians(10.0)
ABORT_POSITION_ERROR_M = 0.25
MAX_SATURATION_SIM_SEC = 0.5
STATE_TIMEOUT_LOCAL_SEC = 0.2
MAX_SIM_SEC = 10.0
MAX_LOCAL_SEC = 30.0
CONTROL_HZ = 60.0
ZERO_COUNT = 10


@dataclass(frozen=True)
class TrialConfig:
    x0: float
    theta0: float
    hold_position: float
    isaac_run_dir: str
    control_hz: float = CONTROL_HZ
    force_limit_n: float = FORCE_LIMIT_N
    max_sim_sec: float = MAX_SIM_SEC
    max_local_sec: float = MAX_LOCAL_SEC
    state_timeout_local_sec: float = STATE_TIMEOUT_LOCAL_SEC
    abort_angle_rad: float = ABORT_ANGLE_RAD
    abort_position_error_m: float = ABORT_POSITION_ERROR_M
    max_saturation_sim_sec: float = MAX_SATURATION_SIM_SEC
    stable_position_m: float = 0.05
    stable_velocity_mps: float = 0.10
    stable_angle_rad: float = math.radians(2.0)
    stable_angular_velocity_radps: float = 0.20
    stable_window_sim_sec: float = 1.0
    max_state_gap_sim_sec: float = 0.05

    def validate(self) -> None:
        scalars = [value for value in asdict(self).values() if isinstance(value, (int, float))]
        if not all(math.isfinite(value) for value in scalars):
            raise ValueError('Trial configuration must be finite')
        if self.force_limit_n != FORCE_LIMIT_N or self.max_sim_sec > MAX_SIM_SEC:
            raise ValueError('This Step 3D entry fixes the 5 N and 10 sim-second limits')
        if self.control_hz <= 0 or self.state_timeout_local_sec <= 0 or self.max_local_sec <= 0:
            raise ValueError('Control frequency and local timeouts must be positive')
        if abs(self.x0) > 0.35 or abs(self.theta0) > math.radians(2.1):
            raise ValueError('Initial state exceeds the planned trial matrix')
        if abs(self.x0 - self.hold_position) > 0.051:
            raise ValueError('Initial cart error exceeds the planned trial matrix')
        if self.max_saturation_sim_sec != MAX_SATURATION_SIM_SEC:
            raise ValueError('Saturation limit is fixed for this trial')


class BalanceHoldClosedLoopNode(Node):
    """Own the only /joint_command publisher for one bounded trial."""

    def __init__(self, config: TrialConfig, trial_dir: Path):
        super().__init__('balance_hold_closed_loop', enable_rosout=False,
                         start_parameter_services=False)
        config.validate()
        self.config = config
        self.trial_dir = trial_dir
        self.isaac_run_dir = Path(config.isaac_run_dir).resolve()
        self._verify_isaac_identity()
        trial_dir.mkdir(parents=True, exist_ok=False)
        (trial_dir / 'config.json').write_text(json.dumps(asdict(config), indent=2))
        self.trace = (trial_dir / 'trace.jsonl').open('x', encoding='utf-8')
        self.events = (self.isaac_run_dir / 'ros_events.jsonl').open(encoding='utf-8')
        self.events.seek(0, os.SEEK_END)
        self.cache = CartpoleStateCache()
        self.controller = BalanceHoldController(max_force=FORCE_LIMIT_N)
        self.phase = 'WAIT_INITIALIZATION'
        self.stop_reason = None
        self.start_local_ns = time.monotonic_ns()
        self.initial_stamp_ns = None
        self.initial_state = None
        self.first_control_ns = None
        self.last_control_ns = None
        self.feedback_count = 0
        self.control_count = 0
        self.max_published_force = 0.0
        self.saturation_start_stamp_ns = None
        self.max_saturation_sim_sec = 0.0
        self.last_graph_check_ns = time.monotonic_ns()
        self.publisher = None
        self._min_initial_stamp_ns = None
        self.feedback_subscription = self.create_subscription(
            JointState, '/joint_states', self._on_feedback, 10)
        self._preflight_command_graph()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(JointState, '/joint_command', qos)
        deadline = time.monotonic() + 4.0
        while self.publisher.get_subscription_count() != 1 and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.publisher.get_subscription_count() != 1:
            raise RuntimeError('The only command publisher did not match Isaac subscriber')
        self.timer = self.create_timer(1.0 / config.control_hz, self._tick)
        request = {'x0': config.x0, 'theta0': config.theta0,
                   'hold_position': config.hold_position, 'controller_pid': os.getpid(),
                   'trial_id': self.isaac_run_dir.name}
        marker = self.isaac_run_dir / 'trial_ready.json'
        if marker.exists():
            raise RuntimeError('Trial ready marker already exists')
        pending = self.isaac_run_dir / f'trial_ready.{os.getpid()}.tmp'
        with pending.open('x', encoding='utf-8') as stream:
            stream.write(json.dumps(request) + '\n')
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(pending, marker)
        self._record('controller_ready', request=request)
        self.get_logger().info('Controller ready; waiting for trial_initialized and new feedback')

    def _verify_isaac_identity(self) -> None:
        ready = json.loads((self.isaac_run_dir / 'ready.json').read_text())
        pid = int(ready['pid'])
        cmdline = (Path('/proc') / str(pid) / 'cmdline').read_bytes()
        if b'run_ros_cartpole.py' not in cmdline or not ready['graph'].startswith(
                '/World/AstrEXROSGraph_'):
            raise RuntimeError('Isaac run directory does not identify a live Cartpole ROS profile')
        self.isaac_graph = ready['graph']

    def _preflight_command_graph(self) -> None:
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            publishers = self.get_publishers_info_by_topic('/joint_command')
            subscribers = self.get_subscriptions_info_by_topic('/joint_command')
            if publishers or len(subscribers) == 1:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if (publishers or len(subscribers) != 1 or
                subscribers[0].topic_type != 'sensor_msgs/msg/JointState'):
            raise RuntimeError(
                f'Expected zero command publishers and one Isaac subscriber; '
                f'found {len(publishers)} publishers and {len(subscribers)} subscribers')

    def _record(self, event: str, **fields) -> None:
        row = {'event': event, 'wall_time_ns': time.time_ns(),
               'local_monotonic_ns': time.monotonic_ns(), **fields}
        self.trace.write(json.dumps(row, allow_nan=False) + '\n')
        self.trace.flush()

    @staticmethod
    def _fields(snapshot) -> dict:
        return {'stamp_ns': snapshot.stamp_ns,
                'received_monotonic_ns': snapshot.received_monotonic_ns,
                'x': snapshot.x, 'x_dot': snapshot.x_dot,
                'theta': snapshot.theta, 'theta_dot': snapshot.theta_dot}

    def _poll_isaac_events(self) -> None:
        while True:
            offset = self.events.tell()
            line = self.events.readline()
            if not line:
                break
            if not line.endswith('\n'):
                self.events.seek(offset)
                break
            event = json.loads(line)
            kind = event.get('event')
            if kind in ('boundary', 'reset_ready', 'reset_failed', 'test_reset'):
                self.request_stop('ISAAC_' + kind.upper())
                return
            if kind == 'trial_initialized':
                if self.phase != 'WAIT_INITIALIZATION':
                    self.request_stop('DUPLICATE_INITIALIZATION')
                    return
                state = event['state']
                self._min_initial_stamp_ns = int(round(state['simulation_time'] * 1e9))
                self.phase = 'WAIT_INITIAL_FEEDBACK'
                self._record('trial_initialized', isaac_state=state)

    def _on_feedback(self, msg: JointState) -> None:
        accepted, reason = self.cache.update(msg)
        if not accepted:
            if reason != 'JointState header.stamp did not advance.':
                self._record('feedback_rejected', reason=reason)
                if self.phase == 'CONTROL':
                    self.request_stop('INVALID_FEEDBACK')
            return
        snapshot = self.cache.snapshot
        if self.phase == 'WAIT_INITIALIZATION':
            return
        if self.phase == 'WAIT_INITIAL_FEEDBACK':
            if snapshot.stamp_ns <= self._min_initial_stamp_ns:
                return
            self.initial_state = snapshot
            self.initial_stamp_ns = snapshot.stamp_ns
            self._record('initial_feedback', **self._fields(snapshot))
            if (abs(snapshot.x - self.config.x0) > 0.005
                    or abs(snapshot.theta - self.config.theta0) > math.radians(0.2)
                    or abs(snapshot.x_dot) > 0.05
                    or abs(snapshot.theta_dot) > 0.1):
                self.request_stop('INITIAL_STATE_MISMATCH')
                return
            self.phase = 'CONTROL'
            self._control_once(snapshot)
            return
        if self.phase == 'CONTROL':
            self.feedback_count += 1
            self._record('feedback', **self._fields(snapshot))

    def _publish(self, force: float, phase: str, snapshot=None, raw=None) -> None:
        if not math.isfinite(force):
            raise ValueError('Refuse a nonfinite command')
        command = max(-FORCE_LIMIT_N, min(FORCE_LIMIT_N, force))
        if abs(command) > FORCE_LIMIT_N:
            raise AssertionError('Published force exceeds 5 N')
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [CART_JOINT, POLE_JOINT]
        msg.effort = [float(command), 0.0]
        self.publisher.publish(msg)
        self.max_published_force = max(self.max_published_force, abs(command))
        row = {'phase': phase, 'f_raw': raw, 'f_cmd': command,
               'hold_position': self.config.hold_position,
               'clamp_ratio': (abs(command / raw) if raw else 1.0)}
        if snapshot is not None:
            row.update(self._fields(snapshot))
        self._record('command', **row)

    def request_stop(self, reason: str) -> None:
        if self.stop_reason is not None:
            return
        self.stop_reason = reason
        self.phase = 'STOPPING'
        if self.publisher is not None:
            try:
                self._publish(0.0, 'stop')
            except Exception as exc:
                self._record('zero_publish_error', error=repr(exc))

    def _control_once(self, snapshot) -> None:
        if self.stop_reason is not None:
            return
        if not self.cache.is_fresh(STATE_TIMEOUT_LOCAL_SEC):
            self.request_stop('STALE_FEEDBACK')
            return
        if (abs(snapshot.theta) > ABORT_ANGLE_RAD or
                abs(snapshot.x - self.config.hold_position) > ABORT_POSITION_ERROR_M):
            self.request_stop('TRIAL_BOUNDARY')
            return
        elapsed_sim = (snapshot.stamp_ns - self.initial_stamp_ns) / 1e9
        # Leave one control period of margin before the 10-second boundary.
        if elapsed_sim >= MAX_SIM_SEC - 1.0 / self.config.control_hz:
            self.request_stop('SIM_WINDOW_END')
            return
        try:
            raw = self.controller.compute_raw_force_from_state(
                snapshot.values, self.config.hold_position)
            if not math.isfinite(raw):
                raise ValueError('Nonfinite LQR output')
            command = self.controller.compute_force_from_state(
                snapshot.values, self.config.hold_position)
            if abs(command) > FORCE_LIMIT_N:
                raise ValueError('Controller returned force above 5 N')
        except (ValueError, OverflowError, FloatingPointError) as exc:
            self._record('control_error', detail=repr(exc))
            self.request_stop('NONFINITE_OR_INVALID_FORCE')
            return
        if abs(raw) > FORCE_LIMIT_N:
            if self.saturation_start_stamp_ns is None:
                self.saturation_start_stamp_ns = snapshot.stamp_ns
            saturation_sec = (snapshot.stamp_ns - self.saturation_start_stamp_ns) / 1e9
            self.max_saturation_sim_sec = max(self.max_saturation_sim_sec, saturation_sec)
            if saturation_sec > MAX_SATURATION_SIM_SEC:
                self.request_stop('SATURATION_TIMEOUT')
                return
        else:
            self.saturation_start_stamp_ns = None
        self._publish(command, 'control', snapshot, raw)
        now_ns = time.monotonic_ns()
        if self.first_control_ns is None:
            self.first_control_ns = now_ns
        self.last_control_ns = now_ns
        self.control_count += 1
        if self.control_count % 60 == 0:
            self.get_logger().info(
                f'control={self.control_count} x={snapshot.x:+.4f} '
                f'theta={math.degrees(snapshot.theta):+.3f}deg '
                f'raw={raw:+.3f}N sent={command:+.3f}N')

    def _tick(self) -> None:
        if self.stop_reason is not None:
            return
        self._poll_isaac_events()
        if self.stop_reason is not None:
            return
        now_ns = time.monotonic_ns()
        if (now_ns - self.start_local_ns) / 1e9 > MAX_LOCAL_SEC:
            self.request_stop('LOCAL_TIMEOUT')
            return
        if self.phase in ('WAIT_INITIALIZATION', 'WAIT_INITIAL_FEEDBACK'):
            return
        if self.phase != 'CONTROL':
            return
        if not self.cache.is_fresh(STATE_TIMEOUT_LOCAL_SEC, now_ns):
            self.request_stop('STALE_FEEDBACK')
            return
        if (now_ns - self.last_graph_check_ns) / 1e9 >= 1.0:
            publishers = self.get_publishers_info_by_topic('/joint_command')
            if len(publishers) != 1:
                self.request_stop('COMPETING_COMMAND_PUBLISHER')
                return
            self.last_graph_check_ns = now_ns
        self._control_once(self.cache.snapshot)

    def finish(self) -> dict:
        if self.stop_reason is None:
            self.request_stop('UNEXPECTED_EXIT')
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if self.publisher is not None:
            for _ in range(ZERO_COUNT - 1):
                try:
                    self._publish(0.0, 'final_zero')
                    time.sleep(1.0 / CONTROL_HZ)
                except Exception as exc:
                    self._record('zero_publish_error', error=repr(exc))
        snapshot = self.cache.snapshot
        final = self._fields(snapshot) if snapshot else None
        duration_ns = (self.last_control_ns - self.first_control_ns
                       if self.last_control_ns and self.first_control_ns else 0)
        result = {'reason': self.stop_reason, 'initial_state': (
                  self._fields(self.initial_state) if self.initial_state else None),
                  'final_state': final,
                  'max_published_force_n': self.max_published_force,
                  'max_continuous_saturation_sim_sec': self.max_saturation_sim_sec,
                  'control_count': self.control_count,
                  'feedback_count': self.feedback_count,
                  'control_rate_hz': (self.control_count * 1e9 / duration_ns
                                      if duration_ns else 0.0),
                  'feedback_rate_hz': (self.feedback_count * 1e9 / duration_ns
                                       if duration_ns else 0.0)}
        self._record('exit', **result)
        (self.trial_dir / 'result.json').write_text(json.dumps(result, indent=2))
        self.events.close()
        self.trace.close()
        return result


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-run-dir', type=Path, required=True)
    parser.add_argument('--trial-dir', type=Path, required=True)
    parser.add_argument('--x0', type=float, required=True)
    parser.add_argument('--theta0', type=float, required=True, help='Initial pole angle in radians')
    parser.add_argument('--hold-position', type=float, required=True)
    args = parser.parse_args()
    config = TrialConfig(args.x0, args.theta0, args.hold_position,
                         str(args.isaac_run_dir.resolve()))
    config.validate()
    if args.trial_dir.exists():
        parser.error('Trial output directory already exists')
    return config, args.trial_dir


def main():
    config, trial_dir = arguments()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = BalanceHoldClosedLoopNode(config, trial_dir)
        while node.stop_reason is None:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        if node is not None:
            node.request_stop('INTERRUPTED')
    except BaseException as exc:
        if node is not None:
            node._record('exception', detail=repr(exc))
            node.request_stop('EXCEPTION')
        else:
            raise
    finally:
        if node is not None:
            result = node.finish()
            node.destroy_node()
        rclpy.try_shutdown()
    return 0 if result['reason'] == 'SIM_WINDOW_END' else 1


if __name__ == '__main__':
    raise SystemExit(main())
