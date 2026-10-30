"""DDS fixture only: stamped poses to kinematic targets, without actuators."""
from node_port_assertions import assert_controller_ports
import copy
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped
from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticArray
from ebva_path_control.waypoint_pp.pure_pursuit_node import PurePursuitController
from test_csv_frenet_node import wait, drain


def test_waypoint_dds_replay_stamps_frames_rejections_gap_and_recovery(tmp_path):
    path=tmp_path/'path.csv';path.write_text('0,0\n3,0\n3,3\n0,3\n0,0\n')
    context=Context();rclpy.init(context=context)
    params=dict(csv_path=str(path),path_frame_id='fixture_odom',body_frame_id='fixture_rear_axle_flu',
                model_point='software_fixture_rear_axle',path_contract='ordered_closed_xy_si_v1',
                path_origin='shared_reference_frame',path_direction='csv_order',
                pose_contract='cartesian_body_pose_si_v1')
    controller=PurePursuitController(context=context,namespace='/waypoint_unit_fixture',
        parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    observer=Node('waypoint_observer',context=context,namespace='/waypoint_unit_fixture')
    executor=SingleThreadedExecutor(context=context);executor.add_node(controller);executor.add_node(observer)
    commands=[];statuses=[]
    publisher=observer.create_publisher(PoseWithCovarianceStamped,'pose',1)
    observer.create_subscription(AckermannDriveStamped,'ackermann_command',commands.append,1)
    observer.create_subscription(DiagnosticArray,'control/path_status',statuses.append,1)
    try:
        wait(executor,lambda:publisher.get_subscription_count()==1 and controller.drive_pub.get_subscription_count()==1 and controller.status_pub.get_subscription_count()==1)
        def pose(y=-.2):
            message=PoseWithCovarianceStamped()
            message.header.stamp=observer.get_clock().now().to_msg();message.header.frame_id='fixture_odom'
            message.pose.pose.position.x=.5;message.pose.pose.position.y=y;message.pose.pose.orientation.w=1.
            return message
        first=pose();publisher.publish(first);wait(executor,lambda:len(commands)==1 and len(statuses)==1)
        assert commands[-1].header.stamp==first.header.stamp
        assert commands[-1].header.frame_id=='fixture_rear_axle_flu'
        assert commands[-1].drive.speed>0. and commands[-1].drive.steering_angle>0.
        deadline=controller.contract.lease.deadline;stamp=controller.contract.lease.stamp
        rejected=[]
        duplicate=copy.deepcopy(first);rejected.append((duplicate,'measurement_time_or_frame'))
        stale=pose();stale.header.stamp.sec-=1;rejected.append((stale,'measurement_time_or_frame'))
        wrong=pose();wrong.header.frame_id='other';rejected.append((wrong,'measurement_time_or_frame'))
        invalid=pose();invalid.pose.pose.orientation.w=0.;rejected.append((invalid,'nonunit_quaternion'))
        nan=pose();nan.pose.pose.position.y=float('nan');rejected.append((nan,'nonfinite_pose'))
        overflow=pose();overflow.header.stamp.nanosec=1_000_000_000;rejected.append((overflow,'invalid_stamp'))
        negative=pose();negative.header.stamp.sec=-1;negative.header.stamp.nanosec=1_500_000_000;rejected.append((negative,'invalid_stamp'))
        for message,reason in rejected:
            before=len(statuses);publisher.publish(message);wait(executor,lambda:len(statuses)>before)
            assert statuses[-1].status[0].message==reason
            assert len(commands)==1
            assert controller.contract.lease.stamp==stamp and controller.contract.lease.deadline==deadline
        drain(executor,.23);assert len(commands)==1
        fresh=pose(.2);publisher.publish(fresh);wait(executor,lambda:len(commands)==2 and statuses[-1].header==fresh.header)
        assert commands[-1].drive.steering_angle<0.
        assert commands[-1].header.stamp==fresh.header.stamp
        assert statuses[-1].header==fresh.header
        assert_controller_ports(controller,
            {'ackermann_command': 'ackermann_msgs/msg/AckermannDriveStamped',
             'control/path_status': 'diagnostic_msgs/msg/DiagnosticArray'},
            {'pose': 'geometry_msgs/msg/PoseWithCovarianceStamped'})
    finally:
        executor.remove_node(controller);executor.remove_node(observer)
        controller.destroy_node();observer.destroy_node();executor.shutdown();rclpy.shutdown(context=context)


def test_waypoint_main_handles_shutdown_sigint_and_propagates_other_failures(monkeypatch):
    from types import SimpleNamespace
    from ebva_path_control.waypoint_pp import pure_pursuit_node as entry
    destroyed=[];node=SimpleNamespace(destroy_node=lambda:destroyed.append(True))
    monkeypatch.setattr(entry,'PurePursuitController',lambda:node)
    def interrupted(actual):
        rclpy.shutdown();raise KeyboardInterrupt
    monkeypatch.setattr(entry.rclpy,'spin',interrupted)
    entry.main(args=[]);assert destroyed==[True] and not rclpy.ok()
    def failed(actual):
        raise RuntimeError('waypoint failure')
    monkeypatch.setattr(entry.rclpy,'spin',failed)
    with pytest.raises(RuntimeError,match='waypoint failure'):entry.main(args=[])
    assert destroyed==[True,True] and not rclpy.ok()
