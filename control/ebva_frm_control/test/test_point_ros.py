import copy
import math
import time
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from ebva_control_msgs.msg import DriveEffort
from ebva_frm_control.point_sample import FrmPointNode


def test_guarded_atomic_output_preserves_source_age_and_does_not_pair_old_fields():
    context=Context();rclpy.init(context=context)
    controller=FrmPointNode(context=context,namespace="frm_fixture",
        parameter_overrides=[Parameter("expected_scan_frame",value="fixture_lidar_flu")])
    source=Node("frm_source",context=context,namespace="frm_fixture")
    executor=SingleThreadedExecutor(context=context)
    for node in (controller,source):executor.add_node(node)
    pub=source.create_publisher(LaserScan,"sensors/scan",10)
    messages=[];source.create_subscription(DriveEffort,"control/autonomy/effort",messages.append,10)
    def pump(duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:executor.spin_once(timeout_sec=.005)
    def scan():
        m=LaserScan();m.header.stamp=source.get_clock().now().to_msg();m.header.frame_id="fixture_lidar_flu"
        m.angle_min=-3*math.pi/4;m.angle_increment=math.pi/720;m.angle_max=m.angle_min+1079*m.angle_increment
        m.range_min=.001;m.range_max=20.;m.ranges=[5.]*1080;return m
    try:
        end=time.monotonic()+2.
        while pub.get_subscription_count()<1 and time.monotonic()<end:pump(.02)
        original=scan();pub.publish(original);pump(.08)
        assert messages and messages[-1].header==original.header
        assert messages[-1].contract=="frm_point_sample_v1"
        assert (messages[-1].longitudinal,messages[-1].steering)==pytest.approx((.2,0.))
        count=len(messages);pub.publish(original);pump(.04);assert len(messages)==count
        for defect in ("short","nan","geometry","frame","old","future","zero"):
            m=scan()
            if defect=="short":m.ranges=[1.]*100
            if defect=="nan":m.ranges[540]=float("nan")
            if defect=="geometry":m.angle_increment=.01
            if defect=="frame":m.header.frame_id="wrong"
            if defect in ("old","future","zero"):
                ns=source.get_clock().now().nanoseconds+(-800_000_000 if defect=="old" else 100_000_000)
                if defect=="zero":ns=0
                m.header.stamp.sec,m.header.stamp.nanosec=divmod(ns,1_000_000_000)
            pub.publish(m);pump(.04)
            assert len(messages)==count,defect
        # Invalid intervals never differentiate a new sample against old state.
        m=scan();m.ranges=[1.]*1080;pub.publish(m);pump(.08)
        assert messages[-1].longitudinal==pytest.approx(.04)
        assert messages[-1].header==m.header
        assert not controller.count_publishers('/autodrive/f1tenth_1/throttle_command')
    finally:
        executor.shutdown();controller.destroy_node();source.destroy_node();context.shutdown()


def test_missing_frame_is_rejected_before_any_subscription():
    c=Context();rclpy.init(context=c)
    try:
        with pytest.raises(ValueError):FrmPointNode(context=c)
    finally:c.shutdown()
