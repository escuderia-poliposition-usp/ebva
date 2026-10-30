"""Variante Pure Pursuit por curvatura, com saída em velocidade e ângulo."""
import copy
import math
from pathlib import Path

from ackermann_msgs.msg import AckermannDriveStamped
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node

from .waypoint_curvature import calculatePurePursuit


class CurvatureController(Node):
    def __init__(self):
        super().__init__("waypoint_curvature")
        default = Path(get_package_share_directory("ebva_path_control")) / "data/waypoint_curvature/config_example_map.yaml"
        for name, value in {
            "reference_file": str(default), "input_topic": "state/pose",
            "output_topic": "control/autonomy/ackermann", "expected_frame": "map",
            "source_timeout_s": 0.2, "lookahead_distance": 0.5,
            "velocity_gain": 0.5, "wheelbase": 0.15597534362552312 + 0.15875,
            "curvature_window": 5,
        }.items():
            self.declare_parameter(name, value)
        self.reference = Path(self.get_parameter("reference_file").value).expanduser()
        if not self.reference.is_file():
            raise ValueError("reference_file deve apontar para um YAML existente")
        self.timeout = self.get_parameter("source_timeout_s").value
        self.expected_frame = self.get_parameter("expected_frame").value
        if not math.isfinite(self.timeout) or self.timeout <= 0 or not self.expected_frame:
            raise ValueError("Timeout positivo e expected_frame explícito são obrigatórios")
        self.options = {
            name: self.get_parameter(name).value for name in
            ("lookahead_distance", "velocity_gain", "wheelbase", "curvature_window")}
        if any(not math.isfinite(self.options[name]) or self.options[name] <= 0
               for name in ("lookahead_distance", "velocity_gain", "wheelbase")):
            raise ValueError("Parâmetros geométricos e ganho devem ser finitos e positivos")
        if type(self.options["curvature_window"]) is not int or self.options["curvature_window"] < 1:
            raise ValueError("curvature_window deve ser inteiro positivo")
        self.last_stamp = None
        self.publisher = self.create_publisher(
            AckermannDriveStamped, self.get_parameter("output_topic").value, 10)
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, self.get_parameter("input_topic").value,
            self.receive, 10)

    def receive(self, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        p, q = message.pose.pose.position, message.pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (message.header.frame_id != self.expected_frame or stamp <= 0
                or not 0 <= age <= self.timeout
                or (self.last_stamp is not None and stamp <= self.last_stamp)
                or not all(math.isfinite(value) for value in values)
                or not math.isclose(sum(v*v for v in (q.x, q.y, q.z, q.w)), 1.0, abs_tol=1e-3)):
            return
        try:
            speed, steering = calculatePurePursuit(
                message, str(self.reference), **self.options)
        except (OSError, ValueError, KeyError, IndexError) as error:
            self.get_logger().warning("Referência inválida: " + str(error))
            return
        if not all(math.isfinite(value) for value in (speed, steering)):
            return
        output = AckermannDriveStamped()
        output.header = copy.deepcopy(message.header)
        output.drive.speed, output.drive.steering_angle = speed, steering
        self.publisher.publish(output)
        self.last_stamp = stamp


def main(args=None):
    rclpy.init(args=args)
    node = CurvatureController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
