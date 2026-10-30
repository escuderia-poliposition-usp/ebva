"""Isolated DDS boundaries, with no actuator/device dependencies."""
from node_port_assertions import assert_controller_ports
import copy
import importlib
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from diagnostic_msgs.msg import DiagnosticArray
from ebva_control_msgs.msg import LongitudinalEffortSteering
from ebva_path_control.waypoint_debug.pp_debug_controller_node import PPDebugController, default_reference_path
from ebva_path_control.lidar_frenet.frenet_pure_pursuit_node import FrenetPurePursuit
from ebva_path_control.effort_contract import load_effort_waypoints
from test_effort_contract import PARAMS, scan
from test_csv_frenet_node import wait, drain


@pytest.mark.parametrize('mode',['carrot','stanley','pure_pursuit'])
def test_waypoint_atomic_dds_original_header_silence_and_recovery(tmp_path,mode):
    path=tmp_path/'xy.csv';path.write_text('x_center,y_center\n0,0\n1,0\n2,0\n3,0\n')
    context=Context();rclpy.init(context=context)
    params=dict(PARAMS,csv_path=str(path),control_mode=mode)
    controller=PPDebugController(context=context,namespace='/effort_'+mode,
        parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    observer=Node('observer',context=context,namespace='/effort_'+mode)
    executor=SingleThreadedExecutor(context=context);executor.add_node(controller);executor.add_node(observer)
    outputs=[];statuses=[];markers=[]
    publisher=observer.create_publisher(Odometry,'odometry',1)
    observer.create_subscription(LongitudinalEffortSteering,'effort_steering_command',outputs.append,1)
    observer.create_subscription(DiagnosticArray,'control/path_status',statuses.append,1)
    observer.create_subscription(Marker,'control/lookahead',markers.append,1)
    try:
        wait(executor,lambda:publisher.get_subscription_count()==1 and controller.drive_pub.get_subscription_count()==1 and controller.status_pub.get_subscription_count()==1 and controller.marker_pub.get_subscription_count()==1)
        def message(y=-.2):
            out=Odometry();out.header.stamp=observer.get_clock().now().to_msg()
            out.header.frame_id='fixture_odom';out.child_frame_id='fixture_rear_axle'
            out.pose.pose.position.y=y;out.pose.pose.orientation.w=1.
            return out
        first=message();publisher.publish(first)
        wait(executor,lambda:len(outputs)==1 and len(statuses)==1 and len(markers)==1)
        assert outputs[-1].header==first.header and markers[-1].header==first.header
        assert outputs[-1].contract=='waypoint_control_'+mode+'_v1'
        assert outputs[-1].longitudinal_effort==.2 and outputs[-1].steering_angle_rad>0
        assert markers[-1].pose.orientation.w==1.
        deadline=controller.contract.lease.deadline
        rejected=[]
        duplicate=copy.deepcopy(first);rejected.append((duplicate,'measurement_time_or_frame'))
        stale=message();stale.header.stamp.sec-=1;rejected.append((stale,'measurement_time_or_frame'))
        wrong=message();wrong.child_frame_id='other';rejected.append((wrong,'body_frame_mismatch'))
        parent=message();parent.header.frame_id='other';rejected.append((parent,'measurement_time_or_frame'))
        nan=message();nan.pose.pose.position.y=float('nan');rejected.append((nan,'nonfinite_pose'))
        invalid=message();invalid.header.stamp.nanosec=1_000_000_000;rejected.append((invalid,'invalid_stamp'))
        for msg,reason in rejected:
            previous=len(statuses);publisher.publish(msg);wait(executor,lambda:len(statuses)>previous)
            assert statuses[-1].status[0].message==reason
            assert len(outputs)==1 and len(markers)==1 and controller.contract.lease.deadline==deadline
        drain(executor,.23)
        assert len(outputs)==1 and len(markers)==1
        fresh=message(.2);publisher.publish(fresh)
        wait(executor,lambda:len(outputs)==2 and len(markers)==2)
        assert outputs[-1].header==fresh.header and outputs[-1].steering_angle_rad<0
        assert_controller_ports(controller,
            {'effort_steering_command': 'ebva_control_msgs/msg/LongitudinalEffortSteering',
             'control/path_status': 'diagnostic_msgs/msg/DiagnosticArray',
             'control/lookahead': 'visualization_msgs/msg/Marker'},
            {'odometry': 'nav_msgs/msg/Odometry'})
    finally:
        executor.remove_node(controller);executor.remove_node(observer)
        controller.destroy_node();observer.destroy_node();executor.shutdown();rclpy.shutdown(context=context)


def test_lidar_atomic_dds_no_odometry_dependency_or_repeated_command():
    context=Context();rclpy.init(context=context)
    params=dict(scan_frame_id='fixture_lidar',scan_contract='planar_body_flu_ranges_m_v1')
    controller=FrenetPurePursuit(context=context,namespace='/lidar_effort',
        parameter_overrides=[Parameter(k,value=v) for k,v in params.items()])
    observer=Node('observer',context=context,namespace='/lidar_effort')
    executor=SingleThreadedExecutor(context=context);executor.add_node(controller);executor.add_node(observer)
    outputs=[];statuses=[]
    publisher=observer.create_publisher(LaserScan,'scan',1)
    observer.create_subscription(LongitudinalEffortSteering,'effort_steering_command',outputs.append,1)
    observer.create_subscription(DiagnosticArray,'control/path_status',statuses.append,1)
    try:
        wait(executor,lambda:publisher.get_subscription_count()==1 and controller.drive_pub.get_subscription_count()==1 and controller.status_pub.get_subscription_count()==1)
        def message(left=3.,right=1.):
            source=scan(left,right);out=LaserScan()
            out.header.stamp=observer.get_clock().now().to_msg();out.header.frame_id='fixture_lidar'
            for name in ('angle_min','angle_max','angle_increment','range_min','range_max','ranges'):
                setattr(out,name,getattr(source,name))
            return out
        first=message();publisher.publish(first)
        wait(executor,lambda:len(outputs)==1 and len(statuses)==1)
        assert outputs[-1].header==first.header and outputs[-1].contract=='lidar_frenet_pp_v1'
        assert outputs[-1].longitudinal_effort==.4 and outputs[-1].steering_angle_rad>0
        deadline=controller.contract.lease.deadline
        rejected=[]
        duplicate=copy.deepcopy(first);rejected.append((duplicate,'measurement_time_or_frame'))
        stale=message();stale.header.stamp.sec-=1;rejected.append((stale,'measurement_time_or_frame'))
        wrong=message();wrong.header.frame_id='other';rejected.append((wrong,'measurement_time_or_frame'))
        missing=message();missing.ranges=[float('nan')]*361;rejected.append((missing,'missing_side_returns'))
        bad=message();bad.range_min=-1.;rejected.append((bad,'invalid_scan_geometry'))
        invalid=message();invalid.header.stamp.sec=-1;rejected.append((invalid,'invalid_stamp'))
        for msg,reason in rejected:
            previous=len(statuses);publisher.publish(msg);wait(executor,lambda:len(statuses)>previous)
            assert statuses[-1].status[0].message==reason
            assert len(outputs)==1 and controller.contract.lease.deadline==deadline
        drain(executor,.23);assert len(outputs)==1
        fresh=message(1.,3.);publisher.publish(fresh)
        wait(executor,lambda:len(outputs)==2)
        assert outputs[-1].header==fresh.header and outputs[-1].steering_angle_rad<0
        assert not hasattr(controller,'odom_sub')
    finally:
        executor.remove_node(controller);executor.remove_node(observer)
        controller.destroy_node();observer.destroy_node();executor.shutdown();rclpy.shutdown(context=context)


def test_canonical_waypoint_resource_resolves_and_loads_from_installed_package():
    path=default_reference_path()
    assert path.endswith('/share/ebva_path_control/data/waypoint_debug/centerline.csv')
    assert len(load_effort_waypoints(path))>2


@pytest.mark.parametrize('module_name,factory',[
    ('ebva_path_control.waypoint_debug.pp_debug_controller_node','PPDebugController'),
    ('ebva_path_control.lidar_frenet.frenet_pure_pursuit_node','FrenetPurePursuit')])
def test_effort_main_idempotent_shutdown_and_failure_propagation(monkeypatch,module_name,factory):
    from types import SimpleNamespace
    entry=importlib.import_module(module_name)
    destroyed=[];node=SimpleNamespace(destroy_node=lambda:destroyed.append(True))
    monkeypatch.setattr(entry,factory,lambda:node)
    def interrupted(actual):
        rclpy.shutdown();raise KeyboardInterrupt
    monkeypatch.setattr(entry.rclpy,'spin',interrupted)
    entry.main(args=[]);assert destroyed==[True] and not rclpy.ok()
    def failed(actual):raise RuntimeError('effort failure')
    monkeypatch.setattr(entry.rclpy,'spin',failed)
    with pytest.raises(RuntimeError,match='effort failure'):entry.main(args=[])
    assert destroyed==[True,True] and not rclpy.ok()
