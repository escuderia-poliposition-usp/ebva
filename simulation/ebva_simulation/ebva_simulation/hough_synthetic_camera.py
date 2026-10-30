#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
synthetic_camera_publisher.py
==============================

Nó auxiliar para testar o lane_detection_node sem uma câmera real: publica
periodicamente a cena sintética de lane_segmentation.generate_synthetic_frame()
(reta + curva de ~90°, faixa tracejada, ruído) como sensor_msgs/Image.

    ros2 run lane_perception synthetic_camera_publisher

Parâmetros:
    image_topic     (string, default "/camera/image_raw")
    rate_hz         (double, default 10.0)
    camera_frame_id (string, default "camera_link")
"""

import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from ebva_lane_detection.lane_segmentation import LaneConfig, generate_synthetic_frame


class SyntheticCameraPublisher(Node):
    def __init__(self):
        super().__init__("synthetic_camera_publisher")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("camera_frame_id", "camera_link")

        topic = self.get_parameter("image_topic").value
        rate_hz = self.get_parameter("rate_hz").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value

        self.bridge = CvBridge()
        # cena fixa (mesma pista sempre) — suficiente para validar o pipeline;
        # troque por uma leitura de vídeo/arquivo se quiser variar os quadros.
        self.frame = generate_synthetic_frame(LaneConfig())

        self.pub = self.create_publisher(Image, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(f"Publicando cena sintética em '{topic}' a {rate_hz:.1f} Hz.")

    def _tick(self):
        msg = self.bridge.cv2_to_imgmsg(self.frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame_id
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
