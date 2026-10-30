import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class TestCameraPublisher(Node):

    def __init__(self):
        super().__init__('test_camera_publisher')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.publish_image
        )

        self.get_logger().info(
            'Test camera publisher started'
        )

    def publish_image(self):

        # Crear imagen artificial
        image = np.zeros(
            (480, 640, 3),
            dtype=np.uint8
        )

        # Simular carril izquierdo
        cv2.line(
            image,
            (230, 480),
            (350, 240),
            (255, 255, 255),
            8
        )

        # Simular carril derecho
        cv2.line(
            image,
            (510, 480),
            (390, 240),
            (255, 255, 255),
            8
        )

        # OpenCV -> ROS2 Image
        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = TestCameraPublisher()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
