"""Persistent, bounded CartPole move-and-hold service on system ROS 2 Jazzy.

The simulator receives only standard JointState topics. This node owns one
command publisher and uses the shared task/controller modules.
"""

import argparse
from collections import deque
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState

from astrex_interfaces.srv import MoveCartTo
from astrex_ros_bridge.balance_hold_controller import BalanceHoldController
from astrex_ros_bridge.cartpole_task import CartpoleTask
from astrex_ros_bridge.state_cache import (
    CART_JOINT, POLE_JOINT, CartpoleStateCache, extract_stamp_ns,
)


SERVICE_NAME = '/astrex/cartpole/move_to'
NODE_NAME = 'cartpole_control_service'
CONTROL_HZ = 60.0
FORCE_LIMIT_N = 5.0
ZERO_COUNT = 10
ZERO_EVIDENCE_SEC = 2.0
GRAPH_CHECK_SEC = 0.25
STATE_MAX_AGE_SEC = 0.2
FAULT_EVENTS = {'boundary', 'reset_ready', 'reset_failed', 'test_reset',
                'physics_log_failed', 'physics_logging_failed'}
JOINT_TYPE = 'sensor_msgs/msg/JointState'


def snapshot_fields(snapshot):
    return None if snapshot is None else asdict(snapshot)


def process_identity(pid):
    """A PID alone does not identify the process across PID reuse."""
    proc = Path('/proc') / str(pid)
    try:
        fields = (proc / 'stat').read_text().rsplit(')', 1)[1].split()
        return {'pid': pid, 'start_ticks': int(fields[19]), 'state': fields[0],
                'cwd': str((proc / 'cwd').resolve(strict=True)),
                'cmdline': (proc / 'cmdline').read_bytes()}
    except (OSError, ValueError, IndexError):
        return None


def endpoint_fields(endpoint):
    return {'node_name': endpoint.node_name, 'node_namespace': endpoint.node_namespace,
            'topic_type': endpoint.topic_type,
            'endpoint_gid': list(endpoint.endpoint_gid)}


def endpoint_owner(endpoint):
    return endpoint.node_name, endpoint.node_namespace, endpoint.topic_type


class JsonlTail:
    """Read complete newly appended rows without consuming an incomplete row."""

    def __init__(self, path, *, start_at_end=True):
        self.stream = Path(path).open(encoding='utf-8')
        if start_at_end:
            self.stream.seek(0, os.SEEK_END)

    def read(self):
        rows = []
        while True:
            offset = self.stream.tell()
            line = self.stream.readline()
            if not line:
                break
            if not line.endswith('\n'):
                self.stream.seek(offset)
                break
            row = json.loads(line)
            if not isinstance(row, dict):
                raise ValueError('A JSONL evidence row must be an object')
            rows.append(row)
        return rows

    def close(self):
        self.stream.close()


class DenseZeroEvidence:
    """Confirm graph Controller input zero in a later real physics callback.

    This checks Controller input, not the unmeasured applied PhysX effort.
    Buffered rows can arrive after publication, so their simulation timestamps
    must also be strictly later than the last zero-publication feedback stamp.
    """

    def __init__(self, path):
        self.tail = JsonlTail(path)
        self.rows = deque(maxlen=1000)

    def poll(self, minimum_stamp_ns):
        self.rows.extend(self.tail.read())
        for row in reversed(self.rows):
            if row.get('logging_source') != 'physx_post_physics_step_single_articulation':
                continue
            stamp = row.get('simulation_time')
            values = row.get('controller_effort')
            names = row.get('joint_names', [])
            if (isinstance(stamp, bool) or not isinstance(stamp, (float, int))
                    or not math.isfinite(stamp)
                    or round(stamp * 1e9) <= minimum_stamp_ns
                    or names.count(CART_JOINT) != 1 or names.count(POLE_JOINT) != 1
                    or not isinstance(values, list) or len(values) != 2):
                continue
            if all(isinstance(v, (int, float)) and not isinstance(v, bool)
                   and math.isfinite(v) and v == 0.0 for v in values):
                return row
        return None

    def close(self):
        self.tail.close()


class CartpoleServiceNode(Node):
    """One active task; service waiters do not own the feedback callback group."""

    def __init__(self, isaac_run_dir, output_dir):
        super().__init__(NODE_NAME, enable_rosout=False, start_parameter_services=False)
        self.lock = threading.RLock()
        self.isaac_run_dir = Path(isaac_run_dir).resolve(strict=True)
        self.output_dir = Path(output_dir).resolve()
        self.output_dir.mkdir(parents=True, exist_ok=True)
        for name in ('trace.jsonl', 'ready.json', 'config.json', 'shutdown.json'):
            if (self.output_dir / name).exists():
                raise RuntimeError(f'Refuse to overwrite service evidence: {name}')
        self.isaac_identity, self.isaac_ready, self.scene_bounds = self._verify_isaac_identity()
        self.trace = (self.output_dir / 'trace.jsonl').open('x', encoding='utf-8')
        self.events = JsonlTail(self.isaac_run_dir / 'ros_events.jsonl')
        self.cache = CartpoleStateCache()
        self.feedback_fault = None
        self.controller = BalanceHoldController(max_force=FORCE_LIMIT_N)
        self.task = None
        self.current_request = None
        self.finalizing = False
        self.final_zero = None
        self.publisher = None
        self.ready = False
        self.stopping = False
        self.shutdown_reason = None
        self.shutdown_complete = threading.Event()
        self.started_local_ns = time.monotonic_ns()
        self.last_graph_check_ns = 0
        self.isaac_subscriber_owner = None
        self.isaac_publisher_owner = None
        self.isaac_subscriber_endpoint = None
        self.isaac_publisher_endpoint = None
        self.command_endpoint = None
        self.idle_rebind_after = None
        self.latest_graph = None
        self.control_group = MutuallyExclusiveCallbackGroup()
        self.service_group = ReentrantCallbackGroup()
        self.feedback_subscription = self.create_subscription(
            JointState, '/joint_states', self._on_feedback, 10,
            callback_group=self.control_group)
        self._preflight_command_graph()
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(JointState, '/joint_command', qos)
        self.service = self.create_service(
            MoveCartTo, SERVICE_NAME, self._on_request, callback_group=self.service_group)
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self._tick,
                                      callback_group=self.control_group)
        config = {'service': SERVICE_NAME, 'control_hz': CONTROL_HZ,
                  'force_limit_n': FORCE_LIMIT_N, 'isaac_run_dir': str(self.isaac_run_dir),
                  'scene_bounds': self.scene_bounds,
                  'lqr_gain': self.controller.gain.reshape(-1).tolist(),
                  'zero_count': ZERO_COUNT, 'zero_evidence_local_sec': ZERO_EVIDENCE_SEC,
                  'client_disconnect_cancels': False,
                  'applied_effort': 'N/A: Controller input is not measured PhysX applied effort'}
        self._write_json(self.output_dir / 'config.json', config)
        self._record('service_started', **config)

    @staticmethod
    def _write_json(path, value):
        with path.open('x', encoding='utf-8') as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write('\n')

    def _verify_isaac_identity(self):
        ready = json.loads((self.isaac_run_dir / 'ready.json').read_text())
        pid = ready.get('pid')
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 0:
            raise RuntimeError('Isaac ready file has no valid PID')
        identity = process_identity(pid)
        if (identity is None or identity['state'] == 'Z'
                or b'run_ros_cartpole.py' not in identity['cmdline']
                or Path(identity['cwd']) != self.isaac_run_dir
                or not str(ready.get('graph', '')).startswith('/World/AstrEXROSGraph_')
                or ready.get('device') != 'cpu' or ready.get('trial') is not None):
            raise RuntimeError('Expected a live, ordinary CPU Isaac Cartpole ROS profile')
        lower, upper = ready['cart_joint_limits']
        scene_limit = ready['scene_max_cart_pos']
        if (not all(math.isfinite(value) for value in (lower, upper, scene_limit))
                or lower >= upper or scene_limit <= 0):
            raise RuntimeError('Invalid actual Isaac cart/scene limits')
        bounds = max(lower, -scene_limit), min(upper, scene_limit)
        if bounds[0] > -0.5 or bounds[1] < 0.5:
            raise RuntimeError('Isaac scene does not support the fixed service target range')
        for name in ('ros_events.jsonl', 'ros_physics_samples.jsonl'):
            if not (self.isaac_run_dir / name).is_file():
                raise RuntimeError(f'Required real Isaac evidence is missing: {name}')
        return identity, ready, bounds

    def _isaac_alive(self):
        actual = process_identity(self.isaac_identity['pid'])
        expected = self.isaac_identity
        return bool(actual and actual['state'] != 'Z'
                    and actual['start_ticks'] == expected['start_ticks']
                    and actual['cwd'] == expected['cwd']
                    and actual['cmdline'] == expected['cmdline'])

    def _preflight_command_graph(self):
        deadline = time.monotonic() + 5.0
        expected_prefix = self.isaac_ready['graph'].replace('/', '_')
        expected_subscriber = expected_prefix + '_Subscribe', '/', JOINT_TYPE
        expected_publisher = expected_prefix + '_Joint', '/', JOINT_TYPE
        previous_candidate = None
        previous_observation = None
        while time.monotonic() < deadline:
            publishers = self.get_publishers_info_by_topic('/joint_command')
            subscribers = self.get_subscriptions_info_by_topic('/joint_command')
            states = self.get_publishers_info_by_topic('/joint_states')
            if publishers:
                raise RuntimeError('Another /joint_command publisher already exists')
            if len(subscribers) == 1 and len(states) == 1:
                if subscribers[0].topic_type != JOINT_TYPE or states[0].topic_type != JOINT_TYPE:
                    raise RuntimeError('Isaac topic endpoint type is incorrect')
                candidate = endpoint_fields(subscribers[0]), endpoint_fields(states[0])
                if candidate != previous_observation:
                    self._record('graph_discovery_observation', subscriber=candidate[0],
                                 state_publisher=candidate[1],
                                 expected_subscriber=expected_subscriber,
                                 expected_state_publisher=expected_publisher)
                    previous_observation = candidate
                if (endpoint_owner(subscribers[0]) == expected_subscriber
                        and endpoint_owner(states[0]) == expected_publisher):
                    if candidate == previous_candidate:
                        self.isaac_subscriber_owner = expected_subscriber
                        self.isaac_publisher_owner = expected_publisher
                        self.isaac_subscriber_endpoint, self.isaac_publisher_endpoint = candidate
                        self._record('graph_preflight', subscribers=[candidate[0]],
                                     state_publishers=[candidate[1]], stable_checks=2)
                        return
                    previous_candidate = candidate
                else:
                    # Discovery can expose GIDs before their node-name metadata.
                    # Never pin _NODE_NAME_UNKNOWN_ or an unrelated resolved name.
                    previous_candidate = None
            rclpy.spin_once(self, timeout_sec=0.05)
        raise RuntimeError('Expected two stable checks of the graph-derived official Isaac endpoint identities')

    def _graph_error(self):
        if not self._isaac_alive():
            return 'ISAAC_DISCONNECTED'
        try:
            publishers = self.get_publishers_info_by_topic('/joint_command')
            subscribers = self.get_subscriptions_info_by_topic('/joint_command')
            states = self.get_publishers_info_by_topic('/joint_states')
            graph = {'command_publishers': [endpoint_fields(item) for item in publishers],
                     'command_subscribers': [endpoint_fields(item) for item in subscribers],
                     'state_publishers': [endpoint_fields(item) for item in states]}
            if graph != self.latest_graph:
                self._record('graph_snapshot', **graph)
                self.latest_graph = graph
            if (len(publishers) != 1 or publishers[0].node_name != self.get_name()
                    or publishers[0].node_namespace != self.get_namespace()
                    or publishers[0].topic_type != JOINT_TYPE):
                return 'COMPETING_COMMAND_PUBLISHER'
            actual_command = endpoint_fields(publishers[0])
            if self.command_endpoint is None:
                self.command_endpoint = actual_command
            elif actual_command != self.command_endpoint:
                return 'COMMAND_ENDPOINT_CHANGED'
            if len(subscribers) != 1 or len(states) != 1:
                return 'ISAAC_ENDPOINT_COUNT_CHANGED'
            command_changed = endpoint_fields(subscribers[0]) != self.isaac_subscriber_endpoint
            state_changed = endpoint_fields(states[0]) != self.isaac_publisher_endpoint
            snapshot = self.cache.snapshot
            can_rebind = (self.idle_rebind_after is not None and self.task is None
                          and self.current_request is None and not self.finalizing
                          and snapshot is not None and self.cache.is_fresh(STATE_MAX_AGE_SEC)
                          and snapshot.received_monotonic_ns > self.idle_rebind_after['local_ns']
                          and snapshot.stamp_ns > self.idle_rebind_after['stamp_ns']
                          and endpoint_owner(subscribers[0]) == self.isaac_subscriber_owner
                          and endpoint_owner(states[0]) == self.isaac_publisher_owner)
            if (command_changed or state_changed) and not can_rebind:
                return 'ISAAC_COMMAND_ENDPOINT_CHANGED' if command_changed else 'ISAAC_STATE_ENDPOINT_CHANGED'
            if can_rebind:
                self._record('verified_idle_reset_endpoint_rebind',
                             previous_subscriber=self.isaac_subscriber_endpoint,
                             subscriber=endpoint_fields(subscribers[0]),
                             previous_publisher=self.isaac_publisher_endpoint,
                             publisher=endpoint_fields(states[0]),
                             reset_ready=self.idle_rebind_after)
                self.isaac_subscriber_endpoint = endpoint_fields(subscribers[0])
                self.isaac_publisher_endpoint = endpoint_fields(states[0])
                self.idle_rebind_after = None
            if self.publisher.get_subscription_count() != 1:
                return 'ISAAC_COMMAND_NOT_MATCHED'
        except Exception as exc:
            self._record('graph_query_failed', detail=repr(exc))
            return 'GRAPH_QUERY_FAILED'
        return None

    def _record(self, event, **fields):
        row = {'event': event, 'wall_time_ns': time.time_ns(),
               'local_monotonic_ns': time.monotonic_ns(), **fields}
        if self.current_request is not None:
            row['request_id'] = self.current_request['request_id']
        self.trace.write(json.dumps(row, allow_nan=False) + '\n')
        self.trace.flush()

    def _consider_ready(self):
        if (self.ready or self.stopping or self.feedback_fault is not None
                or not self.cache.is_fresh(STATE_MAX_AGE_SEC)):
            return
        if self.publisher is None or self._graph_error() is not None:
            return
        ready = {'service': SERVICE_NAME, 'pid': os.getpid(), 'status': 'READY',
                 'isaac_run_dir': str(self.isaac_run_dir),
                 'graph': self.isaac_ready['graph'], 'gui': self.isaac_ready.get('gui'),
                 'state': snapshot_fields(self.cache.snapshot)}
        self._write_json(self.output_dir / 'ready.json', ready)
        self.ready = True
        self._record('ready', **ready)
        self.get_logger().info(f'{SERVICE_NAME} READY; waiting for requests')

    def _poll_events(self):
        try:
            for row in self.events.read():
                self._record('isaac_event', isaac_event=row)
                if (row.get('event') == 'reset_ready' and row.get('verified') is True
                        and row.get('graph_path') == self.isaac_ready['graph']
                        and row.get('graph_creations') == 1 and row.get('astrex_graph_count') == 1
                        and row.get('command_gate') == 'open' and self._isaac_alive()
                        and row.get('checks') and all(value is True for value in row['checks'].values())):
                    sim_time = row.get('state', {}).get('simulation_time')
                    if isinstance(sim_time, (int, float)) and math.isfinite(sim_time):
                        self.idle_rebind_after = {'stamp_ns': round(sim_time * 1e9),
                                                 'local_ns': time.monotonic_ns()}
                if row.get('event') in FAULT_EVENTS:
                    self._abort('ISAAC_' + row['event'].upper())
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self._record('event_read_failed', detail=repr(exc))
            self._abort('ISAAC_EVENT_READ_FAILED')

    def _on_feedback(self, message):
        with self.lock:
            self._poll_events()
            stamp, error = extract_stamp_ns(message)
            if (error is None and self.cache.snapshot is not None
                    and stamp < self.cache.snapshot.stamp_ns):
                self.feedback_fault = 'STATE_TIME_REGRESSION'
                self._record('feedback_rejected', reason=self.feedback_fault)
                self._abort('STATE_TIME_REGRESSION')
                return
            accepted, reason = self.cache.update(message)
            if not accepted:
                if reason != 'JointState header.stamp did not advance.':
                    self.feedback_fault = reason
                    self._record('feedback_rejected', reason=reason)
                    self._abort('INVALID_FEEDBACK')
                return
            snapshot = self.cache.snapshot
            self.feedback_fault = None
            if self.finalizing and self.final_zero['count'] >= ZERO_COUNT:
                zero = self.final_zero
                if (zero['post_zero_feedback_stamp_ns'] is None
                        and snapshot.received_monotonic_ns > zero['last_zero_completed_ns']
                        and snapshot.stamp_ns > zero['minimum_stamp_ns']):
                    zero['post_zero_feedback_stamp_ns'] = snapshot.stamp_ns
                    self._record('post_zero_feedback_barrier', state=snapshot_fields(snapshot),
                                 last_zero_completed_ns=zero['last_zero_completed_ns'])
            self._record('feedback', state=snapshot_fields(snapshot),
                         phase=self.task.phase if self.task else 'IDLE')
            if self.task is not None and not self.finalizing:
                self._advance_task(snapshot, publish=False)
            self._consider_ready()

    def _on_request(self, request, response):
        with self.lock:
            if self.current_request is not None or self.finalizing:
                response.success, response.message = False, 'BUSY: one task is already active'
                self._record('request_rejected', reason='BUSY', target_repr=repr(request.target_position_m))
                return response
            if self.stopping:
                response.success, response.message = False, 'SHUTTING_DOWN'
                return response
            self._poll_events()
            now = time.monotonic_ns()
            error = CartpoleTask.admission_error(self.cache.snapshot, request.target_position_m, now)
            if error is None and self.feedback_fault is not None:
                error = 'INVALID_LATEST_STATE: ' + self.feedback_fault
            if error is None and not self.ready:
                error = 'NOT_READY'
            if error is None and self.idle_rebind_after is not None:
                snapshot = self.cache.snapshot
                if (snapshot.received_monotonic_ns <= self.idle_rebind_after['local_ns']
                        or snapshot.stamp_ns <= self.idle_rebind_after['stamp_ns']):
                    error = 'RESET_FEEDBACK_PENDING'
            if error is None:
                error = self._graph_error()
            if error is not None:
                response.success, response.message = False, error
                self._record('request_rejected', reason=error, target_repr=repr(request.target_position_m),
                             state=snapshot_fields(self.cache.snapshot))
                return response
            request_id = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ_') + uuid.uuid4().hex[:10]
            request_dir = self.output_dir / 'requests' / request_id
            request_dir.mkdir(parents=True, exist_ok=False)
            pending = {'request_id': request_id, 'directory': request_dir,
                       'complete': threading.Event(), 'result': None,
                       'max_published_force': 0.0, 'command_count': 0}
            self.current_request = pending
            self.task = CartpoleTask(controller=self.controller, scene_bounds=self.scene_bounds)
            try:
                self.task.begin(self.cache.snapshot, request.target_position_m, now)
            except ValueError as exc:
                self.current_request, self.task = None, None
                response.success, response.message = False, str(exc)
                self._record('request_rejected', reason=str(exc), target_repr=repr(request.target_position_m))
                return response
            self._record('request_started', task=self.task.fields(), state=snapshot_fields(self.cache.snapshot))
            local_limit = self.task.local_limit_sec
            # A disconnected client cannot cancel this node-owned bounded task.
            deadline = time.monotonic() + local_limit + ZERO_EVIDENCE_SEC + 5.0
        while not pending['complete'].wait(0.1):
            if time.monotonic() > deadline:
                with self.lock:
                    if self.current_request is pending:
                        self._abort('LOCAL_TIMEOUT')
                        if self.finalizing and time.monotonic_ns() > self.final_zero['deadline_ns'] + int(2e9):
                            self._complete_finalization(False, 'ZERO_UNCONFIRMED')
        with self.lock:
            result = pending['result']
            # Keep ownership until this completion waiter has actually resumed.
            if self.current_request is pending:
                self.current_request = None
        response.success, response.message = result['success'], result['message']
        return response

    def _publish(self, force, phase, raw=None, step=None):
        if not math.isfinite(force) or abs(force) > FORCE_LIMIT_N:
            raise ValueError('Refuse a nonfinite command or force above 5 N')
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [CART_JOINT, POLE_JOINT]
        message.effort = [float(force), 0.0]
        self.publisher.publish(message)
        if self.current_request is not None:
            self.current_request['max_published_force'] = max(
                self.current_request['max_published_force'], abs(force))
            self.current_request['command_count'] += 1
        fields = {'phase': phase, 'f_raw': raw, 'f_cmd': force, 'pole_torque': 0.0,
                  'state': snapshot_fields(self.cache.snapshot)}
        if step is not None:
            fields.update(x_ref=step.x_ref, reference_velocity_mps=step.reference_velocity_mps)
        self._record('command', **fields)

    def _advance_task(self, snapshot, *, publish):
        step = self.task.tick(snapshot, time.monotonic_ns())
        if step.event:
            self._record(step.event, task=self.task.fields(), state=snapshot_fields(snapshot))
        if step.done:
            self._begin_finalization(step.success, step.reason)
            return
        if step.event == 'move_success':
            try:
                self._publish(0.0, 'transition_zero', raw=0.0, step=step)
                completed = time.monotonic_ns()
                self.task.note_transition_zero(completed)
                self._record('phase_zero', zero_published_local_monotonic_ns=completed,
                             task=self.task.fields(), state=snapshot_fields(snapshot))
            except Exception as exc:
                self._record('zero_publish_failed', detail=repr(exc))
                self._abort('ZERO_PUBLISH_FAILED')
            return
        if publish and step.phase in ('MOVE', 'HOLD'):
            try:
                self._publish(step.f_cmd, step.phase, raw=step.f_raw, step=step)
            except Exception as exc:
                self._record('control_publish_failed', detail=repr(exc))
                self._abort('COMMAND_PUBLISH_FAILED')

    def _abort(self, reason):
        if self.current_request is None and not self.finalizing:
            return
        if self.finalizing:
            if self.final_zero['success_candidate']:
                self.final_zero['success_candidate'] = False
                self.final_zero['reason'] = reason
            self._record('final_zero_fault', reason=reason)
            return
        if self.task is None:
            return
        self.task.fail(reason)
        self._begin_finalization(False, reason)

    def _begin_finalization(self, success, reason):
        if self.finalizing:
            return
        self.finalizing = True
        now = time.monotonic_ns()
        snapshot = self.cache.snapshot
        evidence = None
        evidence_error = None
        try:
            evidence = DenseZeroEvidence(self.isaac_run_dir / 'ros_physics_samples.jsonl')
        except (OSError, ValueError) as exc:
            evidence_error = repr(exc)
        self.final_zero = {'success_candidate': bool(success), 'reason': reason,
                           'count': 0, 'evidence': evidence, 'evidence_error': evidence_error,
                           'minimum_stamp_ns': snapshot.stamp_ns if snapshot else -1,
                           'last_zero_completed_ns': None, 'post_zero_feedback_stamp_ns': None,
                           'started_ns': now, 'deadline_ns': now + int(ZERO_EVIDENCE_SEC * 1e9),
                           'zero_row': None}
        self._record('final_zero_started', reason=reason, success_candidate=bool(success),
                     minimum_stamp_ns=self.final_zero['minimum_stamp_ns'])
        self._finalize_tick()

    def _finalize_tick(self):
        zero = self.final_zero
        if zero['count'] < ZERO_COUNT:
            try:
                self._publish(0.0, 'final_zero', raw=0.0)
                zero['count'] += 1
                if self.cache.snapshot is not None:
                    zero['minimum_stamp_ns'] = max(zero['minimum_stamp_ns'], self.cache.snapshot.stamp_ns)
                if zero['count'] == ZERO_COUNT:
                    zero['last_zero_completed_ns'] = time.monotonic_ns()
            except Exception as exc:
                zero['evidence_error'] = repr(exc)
                zero['success_candidate'] = False
                zero['reason'] = 'ZERO_PUBLISH_FAILED'
        if (zero['count'] >= ZERO_COUNT and zero['evidence'] is not None
                and zero['post_zero_feedback_stamp_ns'] is not None):
            try:
                # Feedback is verified within two physics steps in this profile.
                # Skip that full latency allowance after the post-publication state.
                barrier = zero['post_zero_feedback_stamp_ns'] + round(2 * 1e9 / 120)
                zero['zero_row'] = zero['evidence'].poll(max(zero['minimum_stamp_ns'], barrier))
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                zero['evidence_error'] = repr(exc)
        if zero['count'] >= ZERO_COUNT and zero['zero_row'] is not None:
            self._complete_finalization(True)
        elif time.monotonic_ns() >= zero['deadline_ns']:
            self._complete_finalization(False, 'ZERO_UNCONFIRMED')

    def _complete_finalization(self, zero_confirmed, failure=None):
        zero = self.final_zero
        if zero_confirmed and zero['success_candidate']:
            graph_error = ('STALE_FEEDBACK' if not self.cache.is_fresh(STATE_MAX_AGE_SEC)
                           else self._graph_error())
            if graph_error is not None:
                zero['success_candidate'] = False
                zero['reason'] = graph_error
        success = bool(zero_confirmed and zero['success_candidate'] and failure is None)
        reason = failure or zero['reason']
        task_fields = self.task.fields() if self.task is not None else None
        result = {'success': success, 'message': 'SUCCESS: move, hold and zero completed' if success else reason,
                  'reason': reason, 'zero_confirmed': zero_confirmed, 'zero_count': zero['count'],
                  'zero_evidence': zero['zero_row'], 'zero_evidence_error': zero['evidence_error'],
                  'last_zero_completed_ns': zero['last_zero_completed_ns'],
                  'post_zero_feedback_stamp_ns': zero['post_zero_feedback_stamp_ns'],
                  'task': task_fields, 'final_state': snapshot_fields(self.cache.snapshot),
                  'applied_effort': 'N/A: Controller input is not measured PhysX applied effort'}
        pending = self.current_request
        if pending is not None:
            result.update(request_id=pending['request_id'],
                          max_published_force=pending['max_published_force'],
                          command_count=pending['command_count'])
            self._write_json(pending['directory'] / 'result.json', result)
        self._record('request_finished' if pending else 'shutdown_zero_finished', **result)
        if zero['evidence'] is not None:
            zero['evidence'].close()
        self.task = self.final_zero = None
        self.finalizing = False
        if pending is not None:
            pending['result'] = result
            pending['complete'].set()
        if self.stopping:
            self._write_json(self.output_dir / 'shutdown.json',
                             {'zero_confirmed': zero_confirmed, 'reason': self.shutdown_reason,
                              'zero_evidence': result['zero_evidence'], 'pid': os.getpid()})
            self.shutdown_complete.set()
        self.get_logger().info(f"Task finished: {result['message']}; zero_confirmed={zero_confirmed}")

    def _tick(self):
        with self.lock:
            self._poll_events()
            now = time.monotonic_ns()
            if now - self.last_graph_check_ns >= int(GRAPH_CHECK_SEC * 1e9):
                error = self._graph_error()
                self.last_graph_check_ns = now
                if error:
                    if self.task is not None or self.finalizing:
                        self._abort(error)
                        if not self.finalizing:
                            return
                    elif not self._isaac_alive():
                        self.request_shutdown(error)
                        return
            if self.finalizing:
                if not self.cache.is_fresh(STATE_MAX_AGE_SEC):
                    self._abort('STALE_FEEDBACK')
                self._finalize_tick()
                return
            if self.stopping:
                return
            if self.task is not None:
                self._advance_task(self.cache.snapshot, publish=True)
            else:
                self._consider_ready()
                if not self.ready and now - self.started_local_ns > int(10e9):
                    self.request_shutdown('STARTUP_FEEDBACK_TIMEOUT')

    def request_shutdown(self, reason='SHUTDOWN'):
        with self.lock:
            if self.stopping:
                return
            self.stopping = True
            self.shutdown_reason = reason
            self._record('shutdown_requested', reason=reason)
            if self.task is not None:
                self._abort(reason)
            elif not self.finalizing:
                self._begin_finalization(False, reason)

    def close_evidence(self):
        if self.final_zero and self.final_zero['evidence']:
            self.final_zero['evidence'].close()
        self.events.close()
        self.trace.close()


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--isaac-run-dir', required=True)
    parser.add_argument('--output-dir', required=True)
    parsed, ros_args = parser.parse_known_args(args)
    stopping = threading.Event()
    previous_signals = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_signals[sig] = signal.signal(sig, lambda *_: stopping.set())
    node, executor = None, None
    error = None
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = CartpoleServiceNode(parsed.isaac_run_dir, parsed.output_dir)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        while rclpy.ok() and not node.shutdown_complete.is_set():
            if stopping.is_set():
                node.request_shutdown()
            executor.spin_once(timeout_sec=0.1)
    except BaseException as exc:
        error = exc
        if node is not None:
            node.request_shutdown('UNEXPECTED_EXIT')
            deadline = time.monotonic() + 4.0
            while rclpy.ok() and not node.shutdown_complete.is_set() and time.monotonic() < deadline:
                if executor is not None:
                    executor.spin_once(timeout_sec=0.05)
                else:
                    node._tick()
                    time.sleep(1.0 / CONTROL_HZ)
    finally:
        if executor is not None:
            executor.shutdown(timeout_sec=3.0)
        if node is not None:
            node.close_evidence()
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous_signals.items():
            signal.signal(sig, handler)
    if error is not None:
        raise error


if __name__ == '__main__':
    main()
