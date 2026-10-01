"""Isolated service-adapter checks; synthetic feedback never uses domain 63."""

import json
import math
from dataclasses import replace
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import JointState

from astrex_interfaces.srv import MoveCartTo
from astrex_ros_bridge.cartpole_service_node import (
    DenseZeroEvidence, CartpoleServiceNode, JsonlTail, endpoint_fields,
)

ORIGINAL_GRAPH_ERROR = CartpoleServiceNode._graph_error
ORIGINAL_PREFLIGHT = CartpoleServiceNode._preflight_command_graph


def feedback(stamp_ns, x=0.0, theta=0.0, x_dot=0.0, theta_dot=0.0):
    message = JointState()
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    message.name = ['cart_to_pole', 'slider_to_cart']
    message.position = [theta, x]
    message.velocity = [theta_dot, x_dot]
    return message


def zero_row(stamp_ns):
    return {'logging_source': 'physx_post_physics_step_single_articulation',
            'simulation_time': stamp_ns / 1e9,
            'joint_names': ['cart_to_pole', 'slider_to_cart'],
            'controller_effort': [0.0, 0.0], 'applied_effort': None}


def append_row(path, row):
    with Path(path).open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row) + '\n')


class PublisherRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)

    def get_subscription_count(self):
        return 1


@pytest.fixture
def service_node(tmp_path, monkeypatch):
    # Do not initialize any test participant in the real control domain.
    monkeypatch.setenv('ROS_DOMAIN_ID', '97')
    rclpy.init()
    isaac = tmp_path / 'isaac'
    isaac.mkdir()
    (isaac / 'ros_events.jsonl').write_text('')
    (isaac / 'ros_physics_samples.jsonl').write_text('')
    monkeypatch.setattr(CartpoleServiceNode, '_verify_isaac_identity', lambda self: (
        {'pid': 1, 'start_ticks': 1}, {'graph': '/World/AstrEXROSGraph_test', 'gui': True},
        (-3.0, 3.0)))
    monkeypatch.setattr(CartpoleServiceNode, '_preflight_command_graph', lambda self: None)
    monkeypatch.setattr(CartpoleServiceNode, '_isaac_alive', lambda self: True)
    monkeypatch.setattr(CartpoleServiceNode, '_graph_error', lambda self: None)
    node = CartpoleServiceNode(isaac, tmp_path / 'service')
    node.timer.cancel()
    node.publisher = PublisherRecorder()
    node._on_feedback(feedback(1_000_000_000))
    yield node, isaac
    node.close_evidence()
    node.destroy_node()
    rclpy.try_shutdown()


def request(target):
    result = MoveCartTo.Request()
    result.target_position_m = float(target)
    return result


def start_request(node, target):
    results = []
    thread = threading.Thread(target=lambda: results.append(
        node._on_request(request(target), MoveCartTo.Response())))
    thread.start()
    deadline = time.monotonic() + 2
    while node.current_request is None and thread.is_alive() and time.monotonic() < deadline:
        time.sleep(0.001)
    assert node.current_request is not None
    return thread, results


def finish_zero(node, isaac):
    while node.finalizing and node.final_zero['count'] < 10:
        node._tick()
    stamp = node.final_zero['minimum_stamp_ns'] + round(1e9 / 120)
    node._on_feedback(feedback(stamp))
    append_row(isaac / 'ros_physics_samples.jsonl', zero_row(stamp + round(3e9 / 120)))
    node._tick()


def test_ready_and_callback_groups_do_not_create_trial_marker(service_node):
    node, isaac = service_node
    ready = json.loads((node.output_dir / 'ready.json').read_text())
    assert ready['status'] == 'READY'
    assert ready['service'] == '/astrex/cartpole/move_to'
    assert isinstance(node.control_group, MutuallyExclusiveCallbackGroup)
    assert isinstance(node.service_group, ReentrantCallbackGroup)
    assert not (isaac / 'trial_ready.json').exists()
    assert node.publisher.messages == []  # Idle does not run a hidden hold.


@pytest.mark.parametrize('target', [float('nan'), float('inf'), -float('inf'), 0.50001, -0.50001])
def test_invalid_target_rejects_without_publishing(service_node, target):
    node, _ = service_node
    response = node._on_request(request(target), MoveCartTo.Response())
    assert not response.success and response.message.startswith('INVALID_TARGET')
    assert node.current_request is None and node.task is None
    assert node.publisher.messages == []


@pytest.mark.parametrize('field,value', [('x', 0.56), ('x_dot', 0.101),
                                        ('theta', math.radians(2.01)), ('theta_dot', 0.201)])
def test_initial_state_admission_does_not_reset(service_node, field, value):
    node, isaac = service_node
    node._on_feedback(feedback(1_010_000_000, **{field: value}))
    response = node._on_request(request(0), MoveCartTo.Response())
    assert not response.success and response.message.startswith('INITIAL_STATE_REJECTED')
    assert not (isaac / 'trial_ready.json').exists()
    assert node.publisher.messages == []


def test_busy_is_immediate_and_preserves_current_task(service_node):
    node, isaac = service_node
    thread, results = start_request(node, 0.3)
    task = node.task
    started = time.monotonic()
    busy = node._on_request(request(-0.3), MoveCartTo.Response())
    assert time.monotonic() - started < 0.1
    assert not busy.success and busy.message.startswith('BUSY')
    assert node.task is task and node.task.target == 0.3
    node._abort('TEST_FAILURE')
    finish_zero(node, isaac)
    thread.join(2)
    assert not thread.is_alive()
    assert not results[0].success and results[0].message == 'TEST_FAILURE'
    assert node.current_request is None


def test_full_sequence_zero_evidence_then_reusable_endpoint(service_node):
    node, isaac = service_node
    for iteration in range(2):
        base = node.cache.snapshot.stamp_ns
        thread, results = start_request(node, 0.0)
        for step in range(1, 500):
            node._on_feedback(feedback(base + round(step * 1e9 / 120)))
            node._tick()
            if node.finalizing:
                break
        assert node.finalizing and not results
        assert node.task.move_success_stamp_ns < node.task.s1_state.stamp_ns
        assert node.task.s1_state.received_monotonic_ns > node.task.transition_zero_local_monotonic_ns
        assert node.task.hold_stable_window['duration_sim_sec'] >= 3.0
        assert node.task.move_stable_window['duration_sim_sec'] >= 1.0
        pending = node.current_request
        finish_zero(node, isaac)
        thread.join(2)
        assert not thread.is_alive() and results[0].success
        saved = json.loads((pending['directory'] / 'result.json').read_text())
        assert saved['zero_confirmed'] and saved['zero_count'] == 10
        assert saved['zero_evidence']['applied_effort'] is None
        assert node.current_request is None and node.task is None
        assert all(abs(message.effort[0]) <= 5 and message.effort[1] == 0
                   for message in node.publisher.messages)


def test_zero_unconfirmed_never_returns_success(service_node):
    node, _ = service_node
    thread, results = start_request(node, 0)
    node._begin_finalization(True, 'SUCCESS')
    node.final_zero['deadline_ns'] = 0
    node._tick()
    thread.join(2)
    assert not results[0].success and results[0].message == 'ZERO_UNCONFIRMED'


def test_fault_during_zero_wins_over_success(service_node, monkeypatch):
    node, isaac = service_node
    thread, results = start_request(node, 0)
    node._begin_finalization(True, 'SUCCESS')
    monkeypatch.setattr(node, '_graph_error', lambda: 'COMPETING_COMMAND_PUBLISHER')
    node.last_graph_check_ns = 0
    finish_zero(node, isaac)
    thread.join(2)
    assert not results[0].success
    assert results[0].message == 'COMPETING_COMMAND_PUBLISHER'


def test_boundary_event_fails_active_request_and_does_not_reset_it(service_node):
    node, isaac = service_node
    thread, results = start_request(node, 0.3)
    append_row(isaac / 'ros_events.jsonl', {'event': 'boundary', 'reason': ['pole']})
    node._tick()
    finish_zero(node, isaac)
    thread.join(2)
    assert not results[0].success and results[0].message == 'ISAAC_BOUNDARY'
    assert list(node.publisher.messages[-1].effort) == [0.0, 0.0]


def test_time_regression_fault_has_zero_cleanup(service_node):
    node, isaac = service_node
    thread, results = start_request(node, 0)
    node._on_feedback(feedback(900_000_000))
    assert node.finalizing
    finish_zero(node, isaac)
    thread.join(2)
    assert results[0].message == 'STATE_TIME_REGRESSION'


def test_shutdown_zeros_before_completion_marker(service_node):
    node, isaac = service_node
    node.request_shutdown()
    assert not node.shutdown_complete.is_set()
    assert not (node.output_dir / 'shutdown.json').exists()
    finish_zero(node, isaac)
    shutdown = json.loads((node.output_dir / 'shutdown.json').read_text())
    assert shutdown['zero_confirmed'] and node.shutdown_complete.is_set()
    assert len(node.publisher.messages) == 10


def test_zero_evidence_requires_new_real_physics_row_and_both_zero(tmp_path):
    path = tmp_path / 'physics.jsonl'
    append_row(path, zero_row(500))
    evidence = DenseZeroEvidence(path)
    assert evidence.poll(0) is None  # Existing EOF rows cannot count.
    append_row(path, zero_row(500))
    assert evidence.poll(500) is None  # Pending buffered old simulation row.
    wrong = zero_row(700)
    wrong['controller_effort'] = [0.0, 0.1]
    append_row(path, wrong)
    assert evidence.poll(500) is None
    wrong['controller_effort'] = [0.0, 0.0]
    wrong['logging_source'] = 'outer_loop'
    append_row(path, wrong)
    assert evidence.poll(500) is None
    append_row(path, zero_row(800))
    assert round(evidence.poll(500)['simulation_time'] * 1e9) == 800
    evidence.close()


def test_jsonl_tail_retries_partial_row(tmp_path):
    path = tmp_path / 'tail.jsonl'
    path.write_text('')
    tail = JsonlTail(path)
    with path.open('a') as stream:
        stream.write('{"event":')
    assert tail.read() == []
    with path.open('a') as stream:
        stream.write('"ready"}\n')
    assert tail.read() == [{'event': 'ready'}]
    tail.close()


def test_cached_missing_or_stale_state_rejects(service_node):
    node, _ = service_node
    node.cache.snapshot = replace(node.cache.snapshot,
                                  received_monotonic_ns=time.monotonic_ns() - 300_000_000)
    result = node._on_request(request(0.0), MoveCartTo.Response())
    assert not result.success and result.message.startswith('STALE_STATE')
    node.cache.snapshot = None
    result = node._on_request(request(0.0), MoveCartTo.Response())
    assert not result.success and result.message.startswith('NO_STATE')
    assert node.publisher.messages == []


def test_real_current_position_starts_reference(service_node):
    node, isaac = service_node
    node._on_feedback(feedback(1_010_000_000, x=0.12))
    thread, results = start_request(node, 0.3)
    assert node.task.reference.x_start == 0.12
    assert node.task.initial_state.x == 0.12
    node._abort('TEST_END')
    finish_zero(node, isaac)
    thread.join(2)
    assert not results[0].success


def test_queued_rows_do_not_confirm_zero_without_post_publish_feedback(service_node):
    node, isaac = service_node
    thread, results = start_request(node, 0)
    node._begin_finalization(True, 'SUCCESS')
    while node.final_zero['count'] < 10:
        node._tick()
    last = node.final_zero['minimum_stamp_ns']
    append_row(isaac / 'ros_physics_samples.jsonl', zero_row(last + round(2e9 / 120)))
    node._tick()
    assert node.finalizing and not results
    assert node.final_zero['post_zero_feedback_stamp_ns'] is None
    node._on_feedback(feedback(last + round(1e9 / 120)))
    node._tick()
    assert node.finalizing  # The queued row is inside the feedback-latency allowance.
    append_row(isaac / 'ros_physics_samples.jsonl', zero_row(last + round(5e9 / 120)))
    node._tick()
    thread.join(2)
    assert results[0].success


def test_stale_state_during_zero_is_not_success(service_node):
    node, isaac = service_node
    thread, results = start_request(node, 0)
    node._begin_finalization(True, 'SUCCESS')
    node.cache.snapshot = replace(node.cache.snapshot,
                                  received_monotonic_ns=time.monotonic_ns() - 300_000_000)
    node._tick()
    finish_zero(node, isaac)
    thread.join(2)
    assert not results[0].success and results[0].message == 'STALE_FEEDBACK'


def configure_graph(node, monkeypatch):
    def endpoint(name, gid):
        return SimpleNamespace(node_name=name, node_namespace='/',
                               topic_type='sensor_msgs/msg/JointState', endpoint_gid=[gid])
    own = endpoint(node.get_name(), 1)
    subscriber = endpoint('isaac_subscriber', 2)
    state_publisher = endpoint('isaac_publisher', 3)
    node.command_endpoint = endpoint_fields(own)
    node.isaac_subscriber_endpoint = endpoint_fields(subscriber)
    node.isaac_publisher_endpoint = endpoint_fields(state_publisher)
    node.isaac_subscriber_owner = ('isaac_subscriber', '/', 'sensor_msgs/msg/JointState')
    node.isaac_publisher_owner = ('isaac_publisher', '/', 'sensor_msgs/msg/JointState')
    monkeypatch.setattr(node, 'get_publishers_info_by_topic',
                        lambda topic: [own] if topic == '/joint_command' else [state_publisher])
    monkeypatch.setattr(node, 'get_subscriptions_info_by_topic', lambda topic: [subscriber])
    return own, subscriber, state_publisher


def test_same_name_replacement_endpoint_without_reset_proof_is_rejected(service_node, monkeypatch):
    node, _ = service_node
    _, subscriber, state_publisher = configure_graph(node, monkeypatch)
    assert ORIGINAL_GRAPH_ERROR(node) is None
    subscriber.endpoint_gid = [9]
    assert ORIGINAL_GRAPH_ERROR(node) == 'ISAAC_COMMAND_ENDPOINT_CHANGED'
    subscriber.endpoint_gid = [2]
    state_publisher.endpoint_gid = [10]
    assert ORIGINAL_GRAPH_ERROR(node) == 'ISAAC_STATE_ENDPOINT_CHANGED'


def test_only_verified_idle_reset_allows_one_endpoint_rebind(service_node, monkeypatch):
    node, isaac = service_node
    _, subscriber, _ = configure_graph(node, monkeypatch)
    subscriber.endpoint_gid = [9]
    append_row(isaac / 'ros_events.jsonl', {
        'event': 'reset_ready', 'verified': True,
        'graph_path': node.isaac_ready['graph'], 'graph_creations': 1,
        'astrex_graph_count': 1, 'command_gate': 'open',
        'checks': {'zero': True, 'rest': True}, 'state': {'simulation_time': 2.0}})
    node._poll_events()
    assert ORIGINAL_GRAPH_ERROR(node) == 'ISAAC_COMMAND_ENDPOINT_CHANGED'
    node._on_feedback(feedback(2_010_000_000))
    assert ORIGINAL_GRAPH_ERROR(node) is None
    assert node.idle_rebind_after is None
    subscriber.endpoint_gid = [11]
    assert ORIGINAL_GRAPH_ERROR(node) == 'ISAAC_COMMAND_ENDPOINT_CHANGED'


def test_invalid_latest_feedback_stays_rejected_until_new_valid_state(service_node):
    node, _ = service_node
    bad = feedback(1_010_000_000)
    bad.velocity = [float('nan'), 0.0]
    node._on_feedback(bad)
    assert node.cache.snapshot.stamp_ns == 1_000_000_000
    rejected = node._on_request(request(0), MoveCartTo.Response())
    assert not rejected.success and rejected.message.startswith('INVALID_LATEST_STATE')
    node._on_feedback(feedback(1_000_000_000))  # Duplicate cannot clear the fault.
    rejected = node._on_request(request(0), MoveCartTo.Response())
    assert not rejected.success and rejected.message.startswith('INVALID_LATEST_STATE')
    node._on_feedback(feedback(1_020_000_000))
    assert node.feedback_fault is None
    assert node.publisher.messages == []


def test_preflight_waits_for_resolved_names_and_two_stable_gid_checks(service_node, monkeypatch):
    node, _ = service_node
    prefix = node.isaac_ready['graph'].replace('/', '_')
    unknown_subscriber = SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',
                                        node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                                        topic_type='sensor_msgs/msg/JointState', endpoint_gid=[2])
    unknown_publisher = SimpleNamespace(node_name='_NODE_NAME_UNKNOWN_',
                                       node_namespace='_NODE_NAMESPACE_UNKNOWN_',
                                       topic_type='sensor_msgs/msg/JointState', endpoint_gid=[3])
    subscriber = SimpleNamespace(node_name=prefix + '_Subscribe', node_namespace='/',
                                 topic_type='sensor_msgs/msg/JointState', endpoint_gid=[2])
    publisher = SimpleNamespace(node_name=prefix + '_Joint', node_namespace='/',
                                topic_type='sensor_msgs/msg/JointState', endpoint_gid=[3])
    checks = {'count': 0}
    monkeypatch.setattr(node, 'get_publishers_info_by_topic', lambda topic: (
        [] if topic == '/joint_command' else [unknown_publisher if checks['count'] == 0 else publisher]))
    monkeypatch.setattr(node, 'get_subscriptions_info_by_topic',
                        lambda topic: [unknown_subscriber if checks['count'] == 0 else subscriber])
    monkeypatch.setattr(rclpy, 'spin_once',
                        lambda *args, **kwargs: checks.update(count=checks['count'] + 1))
    ORIGINAL_PREFLIGHT(node)
    assert checks['count'] >= 2
    assert node.isaac_subscriber_endpoint == endpoint_fields(subscriber)
    assert node.isaac_publisher_endpoint == endpoint_fields(publisher)
