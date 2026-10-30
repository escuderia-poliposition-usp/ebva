"""Isolated DDS software exercise; no sensor, authority, serial or actuator."""
from node_port_assertions import assert_controller_ports
import copy
import time
from pathlib import Path
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from ebva_path_control.csv_frenet.pure_pursuit_node import PurePursuitFrenetNode


def wait(executor, condition, timeout=3.):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline and not condition():
        executor.spin_once(timeout_sec=.01)
    assert condition()


def drain(executor, duration):
    deadline = time.monotonic()+duration
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=.01)


def test_node_synthetic_dds_replay_original_header_expiration_rejections_and_recovery(tmp_path):
    reference = tmp_path/'reference.csv'
    reference.write_text('s,e_y\n0,0\n1.2,0\n2.4,.2\n3.6,0\n4.8,-.2\n6,0\n')
    context = Context()
    rclpy.init(context=context)
    parameters = dict(csv_path=str(reference), reference_frame_id='fixture_map',
                      body_frame_id='fixture_rear_axle_flu', model_point='software_fixture_rear_axle',
                      reference_contract='closed_progress_si_v1',
                      progress_origin='first_accepted_odometry_s0', progress_direction='path_order')
    controller = PurePursuitFrenetNode(context=context, namespace='/path_unit_fixture',
                                     parameter_overrides=[Parameter(k, value=v) for k, v in parameters.items()])
    observer = Node('path_observer', context=context, namespace='/path_unit_fixture')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(controller)
    executor.add_node(observer)
    commands, statuses = [], []
    source = observer.create_publisher(Odometry, 'odometry', 1)
    observer.create_subscription(AckermannDriveStamped, 'ackermann_command', commands.append, 1)
    observer.create_subscription(DiagnosticArray, 'control/path_status', statuses.append, 1)
    try:
        wait(executor, lambda: source.get_subscription_count() == 1
             and controller.drive_pub.get_subscription_count() == 1
             and controller.status_pub.get_subscription_count() == 1)

        def odometry(x=0.):
            msg = Odometry()
            msg.header.stamp = observer.get_clock().now().to_msg()
            msg.header.frame_id = 'fixture_map'
            msg.child_frame_id = 'fixture_rear_axle_flu'
            msg.pose.pose.position.x = x
            msg.pose.pose.orientation.w = 1.
            return msg

        first = odometry()
        source.publish(first)
        wait(executor, lambda: len(commands) == 1 and len(statuses) == 1)
        assert commands[-1].header.stamp == first.header.stamp
        assert commands[-1].header.frame_id == first.child_frame_id
        assert commands[-1].drive.speed == pytest.approx(.5)
        assert commands[-1].drive.steering_angle == 0.
        assert statuses[-1].header == first.header
        assert statuses[-1].status[0].message == 'accepted'

        bad_samples = []
        nanosecond_overflow = odometry()
        nanosecond_overflow.header.stamp.nanosec = 1_000_000_000
        bad_samples.append((nanosecond_overflow, 'invalid_stamp'))
        negative_seconds = odometry()
        negative_seconds.header.stamp.sec = -1
        negative_seconds.header.stamp.nanosec = 1_500_000_000
        # Its naive float sum would be positive despite the negative seconds.
        bad_samples.append((negative_seconds, 'invalid_stamp'))
        original_deadline = controller.contract.lease.deadline
        original_stamp = controller.contract.lease.stamp
        duplicate = copy.deepcopy(first)
        bad_samples.append((duplicate, 'measurement_time_or_frame'))
        stale = odometry()
        stale.header.stamp.sec -= 1
        bad_samples.append((stale, 'measurement_time_or_frame'))
        wrong_body = odometry()
        wrong_body.child_frame_id = 'unknown_body'
        bad_samples.append((wrong_body, 'body_frame'))
        wrong_parent = odometry()
        wrong_parent.header.frame_id = 'unknown_map'
        bad_samples.append((wrong_parent, 'measurement_time_or_frame'))
        invalid = odometry()
        invalid.pose.pose.orientation.w = 0.
        bad_samples.append((invalid, 'nonunit_quaternion'))
        for msg, reason in bad_samples:
            before = len(statuses)
            source.publish(msg)
            wait(executor, lambda: len(statuses) > before)
            assert statuses[-1].status[0].message == reason
            assert len(commands) == 1
            assert controller.contract.lease.stamp == original_stamp
            assert controller.contract.lease.deadline == original_deadline
            assert controller.contract.core.s == 0.
        drain(executor, .23)
        assert len(commands) == 1  # No timer refreshes stale measurement time.

        fresh = odometry(1.2)
        source.publish(fresh)
        wait(executor, lambda: len(commands) == 2)
        assert commands[-1].header.stamp == fresh.header.stamp
        assert commands[-1].drive.steering_angle > 0.
        assert controller.contract.core.s == pytest.approx(1.2)
        assert_controller_ports(controller,
            {'ackermann_command': 'ackermann_msgs/msg/AckermannDriveStamped',
             'control/path_status': 'diagnostic_msgs/msg/DiagnosticArray'},
            {'odometry': 'nav_msgs/msg/Odometry'})
    finally:
        executor.remove_node(controller)
        executor.remove_node(observer)
        controller.destroy_node()
        observer.destroy_node()
        executor.shutdown()
        rclpy.shutdown(context=context)


def test_main_destroys_node_when_sigint_has_already_shutdown_context(monkeypatch):
    from types import SimpleNamespace
    from ebva_path_control.csv_frenet import pure_pursuit_node as entry
    destroyed = []
    node = SimpleNamespace(destroy_node=lambda: destroyed.append(True))
    monkeypatch.setattr(entry, 'PurePursuitFrenetNode', lambda: node)

    def interrupted(actual):
        assert actual is node
        rclpy.shutdown()
        raise KeyboardInterrupt

    monkeypatch.setattr(entry.rclpy, 'spin', interrupted)
    entry.main(args=[])
    assert destroyed == [True]
    assert not rclpy.ok()


def test_main_cleans_up_but_does_not_suppress_unrelated_failure(monkeypatch):
    from types import SimpleNamespace
    from ebva_path_control.csv_frenet import pure_pursuit_node as entry
    destroyed = []
    node = SimpleNamespace(destroy_node=lambda: destroyed.append(True))
    monkeypatch.setattr(entry, 'PurePursuitFrenetNode', lambda: node)

    def failed(actual):
        raise RuntimeError('deliberate software failure')

    monkeypatch.setattr(entry.rclpy, 'spin', failed)
    with pytest.raises(RuntimeError, match='deliberate software failure'):
        entry.main(args=[])
    assert destroyed == [True]
    assert not rclpy.ok()
