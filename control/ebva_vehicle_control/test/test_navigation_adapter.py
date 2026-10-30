"""Unqualified physical calibration cannot emit an autonomous actuator command."""
from pathlib import Path
import yaml,rclpy
from rclpy.context import Context
from rclpy.parameter import Parameter
from ackermann_msgs.msg import AckermannDriveStamped
from ebva_vehicle_control.navigation_adapter import NavigationAdapter
def test_inherited_calibration_blocks_mapping():
    from ament_index_python.packages import get_package_share_directory
    values=yaml.safe_load((Path(get_package_share_directory("ebva_vehicle_control"))/"config/navigation_adapter.yaml").read_text())["navigation_adapter"]["ros__parameters"]
    assert values["qualified"] is False and not values["calibration_id"]
    context=Context();rclpy.init(context=context)
    node=NavigationAdapter(context=context,parameter_overrides=[Parameter(k,value=v) for k,v in values.items()])
    try:
        commands=[];reports=[]
        class Sink:
            def __init__(self,items):self.items=items
            def publish(self,value):self.items.append(value)
        node.pub=Sink(commands);node.status=Sink(reports)
        message=AckermannDriveStamped();message.header.stamp=node.get_clock().now().to_msg()
        message.header.frame_id=values["expected_frame"];message.drive.speed=1.;message.drive.steering_angle=.1
        node.receive(message)
        assert not commands and reports[0].status[0].message=="calibration_pending"
    finally:
        node.destroy_node();context.shutdown()
