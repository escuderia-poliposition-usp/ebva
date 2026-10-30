import math,time
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan, JointState
from ebva_control_msgs.msg import FtgPdCommand
from ebva_mapless_control.ftg_pd import FtgPdNode


def test_two_stamped_inputs_required_and_invalid_or_silent_feedback_cannot_renew_output():
    c=Context();rclpy.init(context=c)
    node=FtgPdNode(context=c,namespace="pd_fixture",parameter_overrides=[Parameter("expected_frame",value="fixture"),Parameter("feedback_purpose",value="software_fixture_angle_rad")])
    source=Node("pd_source",context=c,namespace="pd_fixture");ex=SingleThreadedExecutor(context=c)
    for n in (node,source):ex.add_node(n)
    scan_pub=source.create_publisher(LaserScan,"sensors/scan",10);joint_pub=source.create_publisher(JointState,"sensors/steering",10)
    outputs=[];source.create_subscription(FtgPdCommand,"control/autonomy/ftg_pd",outputs.append,10)
    def pump(t):
        end=time.monotonic()+t
        while time.monotonic()<end:ex.spin_once(timeout_sec=.005)
    def scan():
        m=LaserScan();m.header.stamp=source.get_clock().now().to_msg();m.header.frame_id="fixture"
        m.angle_min=-3*math.pi/4;m.angle_increment=math.pi/720;m.angle_max=3*math.pi/4
        m.range_min=.01;m.range_max=10.;m.ranges=[2.]*1081;return m
    try:
        end=time.monotonic()+2.
        while (scan_pub.get_subscription_count()<1 or joint_pub.get_subscription_count()<1) and time.monotonic()<end:pump(.02)
        scan_pub.publish(scan());pump(.03);assert not outputs
        joint=JointState();joint.header.stamp=source.get_clock().now().to_msg();joint.header.frame_id="fixture"
        joint.name=["virtual_steering"];joint.position=[.1];joint_pub.publish(joint);pump(.03)
        primary=scan();scan_pub.publish(primary);pump(.06)
        assert outputs and outputs[-1].header==primary.header and outputs[-1].feedback_header==joint.header
        assert outputs[-1].contract=="ftg_pd_sample_v1"
        count=len(outputs)
        for mode in ("empty","inf","geometry"):
            m=scan()
            if mode=="empty":m.ranges=[]
            if mode=="inf":m.ranges[100]=float('inf')
            if mode=="geometry":m.angle_increment=0.
            scan_pub.publish(m);pump(.025);assert len(outputs)==count
        pump(.15);joint_pub.publish(joint);scan_pub.publish(scan());pump(.06)
        assert len(outputs)==count  # Duplicated JointState cannot renew original age.
        joint.header.stamp=source.get_clock().now().to_msg();joint.position=[1e308]
        joint_pub.publish(joint);pump(.03);scan_pub.publish(scan());pump(.06)
        assert len(outputs)==count and node.reason=="unrepresentable_controller_output"
    finally:
        ex.shutdown();node.destroy_node();source.destroy_node();c.shutdown()
