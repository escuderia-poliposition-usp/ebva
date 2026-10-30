"""Real adapter callbacks: source header retained, invalid input never renewed."""
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from ebva_control_msgs.msg import LongitudinalEffortSteering
from ebva_control_msgs.msg import ActuatorCommand
from ebva_vehicle_control.effort_steering_node import EffortSteeringAdapter
from ebva_vehicle_control.effort_steering import EffortSteeringMapping

def fixture():
    return EffortSteeringMapping(100.,-.2,.5,1.,1.2,600.,.4,.6)


def parameters():
    return {**fixture().__dict__,"mapping_purpose":"software_fixture",
            "calibration_id":"unit-fixture-not-vehicle","expected_frame":"fixture_rear_axle","expected_contract":"waypoint_control_carrot_v1"}


def test_fixture_identity_is_required_and_unqualified_vehicle_map_is_refused():
    context=Context();rclpy.init(context=context)
    try:
        for purpose in ("","measured_vehicle"):
            values={**parameters(),"mapping_purpose":purpose}
            with pytest.raises(ValueError):
                EffortSteeringAdapter(context=context,parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
    finally:context.shutdown()


def test_real_adapter_keeps_stamp_and_drops_stale_frame_nonfinite_and_duplicates():
    context=Context();rclpy.init(context=context)
    adapter=EffortSteeringAdapter(context=context,namespace="effort_fixture",
                            parameter_overrides=[Parameter(k,value=v) for k,v in parameters().items()])
    source=Node("input_fixture",context=context,namespace="effort_fixture")
    executor=SingleThreadedExecutor(context=context)
    for node in (adapter,source):executor.add_node(node)
    pub=source.create_publisher(LongitudinalEffortSteering,"control/autonomy/effort_steering",10)
    outputs=[];source.create_subscription(ActuatorCommand,"control/autonomy/actuators",outputs.append,10)
    def pump(duration):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.005)
    try:
        pump(.15)
        msg=LongitudinalEffortSteering();msg.header.stamp=source.get_clock().now().to_msg()
        msg.header.frame_id="fixture_rear_axle";msg.contract="waypoint_control_carrot_v1";msg.longitudinal_effort=-1.;msg.steering_angle_rad=.2
        pub.publish(msg);pump(.1)
        assert outputs and outputs[-1].erpm == -100. and outputs[-1].servo == pytest.approx(.46)
        assert outputs[-1].header == msg.header
        count=len(outputs)
        pub.publish(msg);pump(.08)
        assert len(outputs)==count
        for age,frame,speed in [(1.,"fixture_rear_axle",1.),(-.1,"fixture_rear_axle",1.),
                                 (0.,"wrong",1.),(0.,"fixture_rear_axle",float("nan"))]:
            now=source.get_clock().now().nanoseconds-int(age*1e9)
            msg.header.stamp.sec,msg.header.stamp.nanosec=divmod(now,1_000_000_000)
            msg.header.frame_id=frame;msg.longitudinal_effort=speed
            pub.publish(msg);pump(.08)
        assert len(outputs)==count
        msg.header.stamp=source.get_clock().now().to_msg();msg.header.frame_id="fixture_rear_axle"
        msg.longitudinal_effort=.2;msg.contract="other_dialect";pub.publish(msg);pump(.05)
        assert len(outputs)==count
    finally:
        executor.shutdown();adapter.destroy_node();source.destroy_node();context.shutdown()
