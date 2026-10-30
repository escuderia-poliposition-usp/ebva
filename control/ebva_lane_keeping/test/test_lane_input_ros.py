"""ROS measurement-age regression; no camera/driver/devices."""
import time
import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from lane_msgs.msg import LaneBoundaries
from ebva_lane_keeping.lane_keeping_node import LaneKeepingNode


@pytest.mark.parametrize("age,frame,duplicate",[(.8,"camera_link",False),(-.1,"camera_link",False),
                                               (0.,"wrong_frame",False),(0.,"camera_link",True)])
def test_old_lane_observations_cannot_be_restamped_as_fresh_control(age,frame,duplicate):
    rclpy.init()
    controller = LaneKeepingNode()
    source = Node("lane_age_fixture")
    executor = SingleThreadedExecutor()
    for node in (controller, source):
        executor.add_node(node)
    publisher = source.create_publisher(LaneBoundaries, "/lane_detection/lane_boundaries", 10)
    speeds = []
    source.create_subscription(AckermannDriveStamped, "/ackermann_cmd",
                               lambda message:speeds.append(message.drive.speed), 10)
    try:
        frozen_ns = source.get_clock().now().nanoseconds
        deadline = time.monotonic()+.8
        while time.monotonic() < deadline:
            message = LaneBoundaries()
            now = frozen_ns if duplicate else source.get_clock().now().nanoseconds-int(age*1e9)
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(now, 1_000_000_000)
            message.header.frame_id = frame
            message.left_lane = [Point(x=x, y=-.25, z=0.) for x in (.2,.4,.6,.8,1.)]
            message.right_lane = [Point(x=x, y=.25, z=0.) for x in (.2,.4,.6,.8,1.)]
            publisher.publish(message)
            for _ in range(12):
                executor.spin_once(timeout_sec=0.)
            time.sleep(.01)
        assert speeds, "Actual ROS output required"
        assert max(speeds) == 0., "Old measurement was promoted to a fresh moving command"
    finally:
        executor.shutdown()
        controller.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
