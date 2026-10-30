"""DDS fixtures only: no lidar driver, authority, serial backend or actuators."""
from node_port_assertions import assert_controller_ports
import copy
import math
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from ebva_control_msgs.msg import FrenetLateralErrorStamped
from ebva_path_control.stanley_frenet.stanley_node import StanleyFrenetNode
from test_stanley_frenet_contract import PARAMS, REFERENCE_CONTRACT


def wait(executor, condition, timeout=3.):
    deadline=time.monotonic()+timeout
    while time.monotonic()<deadline and not condition():
        executor.spin_once(timeout_sec=.005)
    assert condition()


def drain(executor, duration):
    deadline=time.monotonic()+duration
    while time.monotonic()<deadline:
        executor.spin_once(timeout_sec=.005)


@pytest.mark.parametrize('oldest_source',['odometry','lateral_error'])
def test_dds_original_oldest_header_guards_silence_and_recovery(oldest_source):
    context=Context();rclpy.init(context=context)
    controller=StanleyFrenetNode(context=context,namespace='/stanley_unit_fixture',
        parameter_overrides=[Parameter(k,value=v) for k,v in PARAMS.items()])
    observer=Node('stanley_observer',context=context,namespace='/stanley_unit_fixture')
    executor=SingleThreadedExecutor(context=context)
    executor.add_node(controller);executor.add_node(observer)
    commands=[];statuses=[]
    odom_pub=observer.create_publisher(Odometry,'odometry',1)
    error_pub=observer.create_publisher(FrenetLateralErrorStamped,'lateral_error',1)
    observer.create_subscription(AckermannDriveStamped,'ackermann_command',commands.append,1)
    observer.create_subscription(DiagnosticArray,'control/path_status',statuses.append,1)
    try:
        wait(executor,lambda:odom_pub.get_subscription_count()==1
             and error_pub.get_subscription_count()==1
             and controller.drive_pub.get_subscription_count()==1
             and controller.status_pub.get_subscription_count()==1)

        def odometry(stamp=None):
            m=Odometry();m.header.stamp=stamp or observer.get_clock().now().to_msg()
            m.header.frame_id=PARAMS['reference_frame_id'];m.child_frame_id=PARAMS['body_frame_id']
            m.pose.pose.orientation.w=1.;m.twist.twist.linear.x=-.6
            return m

        def lateral(stamp=None):
            m=FrenetLateralErrorStamped();m.header.stamp=stamp or observer.get_clock().now().to_msg()
            m.header.frame_id=PARAMS['reference_frame_id'];m.reference_id=PARAMS['reference_id']
            m.body_frame_id=PARAMS['body_frame_id'];m.model_point=PARAMS['model_point']
            m.contract_id=REFERENCE_CONTRACT;m.lateral_error_m=.3
            return m

        older=observer.get_clock().now().to_msg()
        drain(executor,.025)
        newer=observer.get_clock().now().to_msg()
        o=odometry(older if oldest_source=='odometry' else newer)
        e=lateral(older if oldest_source=='lateral_error' else newer)
        error_pub.publish(e)
        wait(executor,lambda:len(statuses)==1)
        assert not commands and statuses[-1].status[0].message=='odometry_missing_or_expired'
        odom_pub.publish(o)
        wait(executor,lambda:len(commands)==1 and len(statuses)==2)
        assert commands[-1].header.stamp==older
        assert commands[-1].header.frame_id==PARAMS['body_frame_id']
        assert commands[-1].drive.speed==pytest.approx(-.6,abs=3.e-8) and commands[-1].drive.steering_angle<0
        values={kv.key:kv.value for kv in statuses[-1].status[0].values}
        assert values['oldest_source']==oldest_source
        for name,message in [('odometry',o),('lateral_error',e)]:
            assert values[name+'_stamp_sec']==str(message.header.stamp.sec)
            assert values[name+'_stamp_nanosec']==str(message.header.stamp.nanosec)
            assert values[name+'_frame_id']==message.header.frame_id
        bad_inputs=[]
        bad=odometry();bad.header.stamp.nanosec=1_000_000_000
        bad_inputs.append((odom_pub,bad,'invalid_odometry_stamp'))
        bad=lateral();bad.header.stamp.sec=-1
        bad_inputs.append((error_pub,bad,'invalid_lateral_error_stamp'))
        bad=odometry();bad.child_frame_id='wrong_body'
        bad_inputs.append((odom_pub,bad,'body_frame_mismatch'))
        bad=lateral();bad.reference_id='wrong_reference'
        bad_inputs.append((error_pub,bad,'lateral_error_reference_contract'))
        bad=lateral();bad.lateral_error_m=float('nan')
        bad_inputs.append((error_pub,bad,'nonfinite_lateral_error'))
        bad_inputs.append((error_pub,copy.deepcopy(e),'lateral_error_time_or_frame'))
        deadlines=(controller.contract.odom_lease.deadline,controller.contract.error_lease.deadline)
        for publisher,message,reason in bad_inputs:
            count=len(statuses);publisher.publish(message)
            wait(executor,lambda:len(statuses)>count)
            assert statuses[-1].status[0].message==reason and len(commands)==1
        assert deadlines==(controller.contract.odom_lease.deadline,controller.contract.error_lease.deadline)
        drain(executor,.23)
        assert len(commands)==1  # No timer command or measurement renewal.
        # A new odometry alone cannot recover the expired lateral error.
        count=len(statuses);odom_pub.publish(odometry())
        wait(executor,lambda:len(statuses)>count)
        assert statuses[-1].status[0].message=='lateral_error_missing_or_expired'
        assert len(commands)==1
        recovered=lateral();recovered.lateral_error_m=-.3
        error_pub.publish(recovered)
        wait(executor,lambda:len(commands)==2)
        assert commands[-1].drive.steering_angle>0
        # Inverse loss: continuing errors do not keep old odometry alive.
        drain(executor,.23);count=len(statuses);error_pub.publish(lateral())
        wait(executor,lambda:len(statuses)>count)
        assert statuses[-1].status[0].message=='odometry_missing_or_expired'
        assert len(commands)==2
        odom_pub.publish(odometry());wait(executor,lambda:len(commands)==3)
        assert commands[-1].drive.speed==pytest.approx(-.6,abs=3.e-8)
        assert_controller_ports(controller,
            {'ackermann_command': 'ackermann_msgs/msg/AckermannDriveStamped',
             'control/path_status': 'diagnostic_msgs/msg/DiagnosticArray'},
            {'odometry': 'nav_msgs/msg/Odometry',
             'lateral_error': 'ebva_control_msgs/msg/FrenetLateralErrorStamped'})
    finally:
        executor.remove_node(controller);executor.remove_node(observer)
        controller.destroy_node();observer.destroy_node();executor.shutdown()
        rclpy.shutdown(context=context)


def test_main_cleanup_after_shutdown_and_unrelated_failure(monkeypatch):
    from types import SimpleNamespace
    from ebva_path_control.stanley_frenet import stanley_node as entry
    destroyed=[];node=SimpleNamespace(destroy_node=lambda:destroyed.append(True))
    monkeypatch.setattr(entry,'StanleyFrenetNode',lambda:node)
    def interrupted(actual):
        assert actual is node
        rclpy.shutdown();raise KeyboardInterrupt
    monkeypatch.setattr(entry.rclpy,'spin',interrupted)
    entry.main(args=[])
    assert destroyed==[True] and not rclpy.ok()
    def failed(actual):raise RuntimeError('deliberate software failure')
    monkeypatch.setattr(entry.rclpy,'spin',failed)
    with pytest.raises(RuntimeError,match='deliberate software failure'):entry.main(args=[])
    assert destroyed==[True,True] and not rclpy.ok()


@pytest.mark.parametrize('direct_endpoint', ['publisher', 'subscriber'])
def test_port_audit_ignores_neighbor_but_rejects_owned_actuator_endpoint(direct_endpoint):
    from std_msgs.msg import Float64
    context = Context(); rclpy.init(context=context)
    controller = StanleyFrenetNode(context=context, namespace='/stanley_scope_fixture',
        parameter_overrides=[Parameter(k, value=v) for k, v in PARAMS.items()])
    neighbor = Node('unrelated_backend_fixture', context=context, namespace='/stanley_scope_fixture/neighbor')
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(controller); executor.add_node(neighbor)
    # DDS messages only: this publisher has no backend, serial port or device.
    neighbor.create_publisher(Float64, 'commands/motor', 1)
    expected_publishers = {'ackermann_command': 'ackermann_msgs/msg/AckermannDriveStamped',
                           'control/path_status': 'diagnostic_msgs/msg/DiagnosticArray'}
    expected_subscribers = {'odometry': 'nav_msgs/msg/Odometry',
                           'lateral_error': 'ebva_control_msgs/msg/FrenetLateralErrorStamped'}
    try:
        wait(executor, lambda: any('/neighbor/commands/motor' in topic
             for topic, _ in controller.get_topic_names_and_types()))
        # The old global assertion would fail solely because of this neighbor.
        assert_controller_ports(controller, expected_publishers, expected_subscribers)
        if direct_endpoint == 'publisher':
            controller.create_publisher(Float64, 'commands/motor', 1)
            query = controller.get_publisher_names_and_types_by_node
        else:
            controller.create_subscription(Float64, 'commands/motor', lambda message: None, 1)
            query = controller.get_subscriber_names_and_types_by_node
        wait(executor, lambda: '/stanley_scope_fixture/commands/motor' in
             dict(query(controller.get_name(), controller.get_namespace())))
        with pytest.raises(AssertionError, match='Actuator endpoint owned by controller'):
            assert_controller_ports(controller, expected_publishers, expected_subscribers)
    finally:
        executor.remove_node(controller); executor.remove_node(neighbor)
        controller.destroy_node(); neighbor.destroy_node(); executor.shutdown()
        rclpy.shutdown(context=context)
