#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS Image -> LaneBoundaries wrapper for explicit preserved segmentation methods.

The default remains road_mask_v4_80. The output plane is x-forward/y-right in
assumed homography metres, with the original image stamp and declared camera
frame. Parameters are consumed at construction. No temporal tracker, coasting,
TF, sensor driver, actuator or physical calibration is introduced here.
See docs/lane-method-contract.md for encodings, envelope checks and limits.
"""

import dataclasses
import copy
import json
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, String

from lane_msgs.msg import LaneBoundaries

from ebva_lane_detection.lane_segmentation import LaneConfig, process_frame
from ebva_lane_detection.segmentation_methods import config_for_method, validate_config
from ebva_lane_detection.image_contract import validate_image_message


# Campos de LaneConfig que não são declaráveis como parâmetro ROS 2 (não são
# tipos primitivos/arrays primitivos) — ficam com o valor padrão do dataclass.
_NON_PARAM_FIELDS = {"camera_matrix", "dist_coeffs"}


def build_config_from_node_params(node: Node) -> LaneConfig:
    """Declara cada campo (primitivo) de LaneConfig como parâmetro ROS 2,
    usando defaults do método selecionado e devolve uma LaneConfig com os
    valores fornecidos na construção do nó (YAML ou linha de comando)."""
    node.declare_parameter("segmentation_method", "road_mask_v4_80")
    method = node.get_parameter("segmentation_method").value
    defaults = config_for_method(method)
    values = {}
    for f in dataclasses.fields(defaults):
        if f.name in _NON_PARAM_FIELDS or f.name == "segmentation_method":
            continue
        default_value = getattr(defaults, f.name)
        # tuplas (ex.: sobel_thresh, morph_close_kernel) viram listas p/ o ROS 2
        is_tuple = isinstance(default_value, tuple)
        ros_default = list(default_value) if is_tuple else default_value
        node.declare_parameter(f.name, ros_default)
        value = node.get_parameter(f.name).value
        values[f.name] = tuple(value) if is_tuple else value
    return LaneConfig(segmentation_method=method, **values)


class LaneDetectionNode(Node):
    def __init__(self, *, context=None, namespace="", parameter_overrides=None, cli_args=None):
        super().__init__("lane_detection_node", context=context, namespace=namespace,
                         parameter_overrides=parameter_overrides, cli_args=cli_args)

        try:
            self._configure()
        except Exception:
            self.destroy_node()
            raise

    def _configure(self):
        self.declare_parameter("image_topic", "/zed/zed_node/rgb/color/rect/image")
        self.declare_parameter("camera_frame_id", "camera_link")
        self.declare_parameter("publish_debug_image", True)

        self.cfg = validate_config(build_config_from_node_params(self))
        self.bridge = CvBridge()

        image_topic = self.get_parameter("image_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.publish_debug_image = self.get_parameter("publish_debug_image").value
        if not self.camera_frame_id or self.camera_frame_id.strip() != self.camera_frame_id:
            raise ValueError("camera_frame_id must be an explicit nonempty camera-coordinate frame")

        self.status_pub = self.create_publisher(String, "/lane_detection/status", 10)
        self.lane_pub = self.create_publisher(LaneBoundaries, "/lane_detection/lane_boundaries", 10)
        self.debug_pub = None
        if self.publish_debug_image:
            self.debug_pub = self.create_publisher(Image, "/lane_detection/birdseye_debug", 1)

        # QoS de sensor (best-effort, profundidade curta) — compatível com a
        # maioria dos drivers de câmera em ROS 2.
        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"lane_detection_node pronto — ouvindo '{image_topic}', "
            f"publicando em '/lane_detection/lane_boundaries'"
            + (" e '/lane_detection/birdseye_debug'." if self.debug_pub else ".")
        )

    def _status(self, reason, msg):
        self.status_pub.publish(String(data=json.dumps({
            "reason": reason, "segmentation_method": self.cfg.segmentation_method,
            "input_stamp_sec": msg.header.stamp.sec,
            "input_stamp_nanosec": msg.header.stamp.nanosec,
            "input_frame": msg.header.frame_id, "output_frame": self.camera_frame_id,
            "temporal_coasting": False, "physical_calibration_verified": False})))

    def image_callback(self, msg: Image):
        try:
            validate_image_message(msg)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if frame.dtype != np.uint8 or frame.shape != (msg.height, msg.width, 3):
                raise ValueError("converted_image_shape_or_dtype")
        except Exception as exc:
            self._status(str(exc), msg)
            self.get_logger().warn(f"Imagem rejeitada: {exc}")
            return

        # Deliberately no LaneTracker: do not stamp historical/coasted geometry as new.
        debug_img, left_xy, right_xy, left_uv, right_uv, binary, birdseye_view, overlay_img = process_frame(
            frame, self.cfg, return_uv=True)
        points = [*left_xy, *right_xy]
        if any(not np.isfinite(x) or not np.isfinite(y) for x, y in points):
            self._status("nonfinite_lane_points", msg)
            return
        header = copy.deepcopy(msg.header)
        # This is the declared x-forward/y-right camera plane, not an implicit TF transform.
        header.frame_id = self.camera_frame_id
        out = LaneBoundaries()
        out.header = header
        out.left_lane = [Point(x=float(x), y=float(y), z=0.0) for x, y in left_xy]
        out.right_lane = [Point(x=float(x), y=float(y), z=0.0) for x, y in right_xy]
        self.lane_pub.publish(out)
        if self.debug_pub is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(overlay_img, encoding="bgr8")
            # Overlay pixels remain in the original image frame, unlike metric lane points.
            debug_msg.header = copy.deepcopy(msg.header)
            self.debug_pub.publish(debug_msg)
        self._status("accepted", msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LaneDetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
