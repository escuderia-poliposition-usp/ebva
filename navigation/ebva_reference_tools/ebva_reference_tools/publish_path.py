"""Publica uma referência estática JSON como nav_msgs/Path transitório local."""
import copy
import json
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped


def load_waypoints(filename):
    if not filename:
        raise ValueError("Defina waypoints_file explicitamente")
    data = json.loads(Path(filename).expanduser().read_text())
    if not isinstance(data, list) or not data:
        raise ValueError("A referência deve ser uma lista não vazia de {x,y,yaw}")
    result = []
    for point in data:
        if not isinstance(point, dict) or any(k not in point for k in ("x", "y", "yaw")):
            raise ValueError("Cada waypoint precisa de x, y e yaw")
        values = tuple(point[k] for k in ("x", "y", "yaw"))
        if any(type(value) not in (int, float) or not math.isfinite(value) for value in values):
            raise ValueError("Waypoints devem conter números finitos")
        result.append(tuple(float(value) for value in values))
    return result


class PathPublisher(Node):
    def __init__(self):
        super().__init__("path_publisher")
        self.declare_parameter("waypoints_file", "")
        self.declare_parameter("frame_id", "map")
        points = load_waypoints(self.get_parameter("waypoints_file").value)
        frame = self.get_parameter("frame_id").value
        if not frame:
            raise ValueError("frame_id deve ser explícito")
        qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(RosPath, "/reference_path", qos)
        path = RosPath()
        path.header.frame_id = frame
        path.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in points:
            point = PoseStamped()
            point.header = copy.deepcopy(path.header)
            point.pose.position.x, point.pose.position.y = x, y
            point.pose.orientation.z = math.sin(yaw / 2)
            point.pose.orientation.w = math.cos(yaw / 2)
            path.poses.append(point)
        self.publisher.publish(path)
        self.get_logger().info("Referência estática publicada: " + str(len(points)) + " pontos")


def main(argv=None):
    rclpy.init(args=argv)
    node = PathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
