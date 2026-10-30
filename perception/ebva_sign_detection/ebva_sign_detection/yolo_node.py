# Copyright 2026 EBVA (Escuderia Brasileira de Veículos Autônomos)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""ROS 2 YOLO detector with explicit model, device, topics and QoS."""

import json
from pathlib import Path

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from ebva_sign_detection.backend import YoloBackend
from std_srvs.srv import Trigger
from vision_msgs.msg import (
    Detection2D, Detection2DArray, ObjectHypothesisWithPose,
)


def detection_message(result, header):
    """Translate YOLO boxes to the ROS Humble vision_msgs contract."""
    message = Detection2DArray()
    message.header = header
    for box in result.boxes:
        detection = Detection2D()
        detection.header = header
        x, y, width, height = box.xywh[0].tolist()
        detection.bbox.center.position.x = float(x)
        detection.bbox.center.position.y = float(y)
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(int(box.cls[0].item()))
        hypothesis.hypothesis.score = float(box.conf[0].item())
        detection.results.append(hypothesis)
        message.detections.append(detection)
    return message


class YoloPublisherNode(Node):
    """Publish detections and annotated images with original headers."""

    def __init__(self, model_factory=YoloBackend, **kwargs):
        super().__init__('yolo_publisher_node', **kwargs)
        defaults = dict(
            model_path='', device='cpu', imgsz=640, confidence=0.25,
            compressed_input=False,
            input_topic='/camera/image_raw',
            detections_topic='/yolo/detections',
            annotated_topic='/yolo/annotated_image',
            input_reliability='best_effort', queue_depth=20)
        values = {
            name: self.declare_parameter(name, value).value
            for name, value in defaults.items()}
        path = Path(values['model_path']).expanduser()
        if (not values['model_path'] or not path.is_file()
                or path.suffix != '.pt'):
            raise ValueError('model_path must identify an existing checkpoint')
        if values['imgsz'] <= 0 or not 0 <= values['confidence'] <= 1:
            raise ValueError('Invalid imgsz or confidence')
        if values['queue_depth'] <= 0:
            raise ValueError('queue_depth must be positive')
        policies = dict(
            reliable=ReliabilityPolicy.RELIABLE,
            best_effort=ReliabilityPolicy.BEST_EFFORT)
        if values['input_reliability'] not in policies:
            raise ValueError('input_reliability: reliable or best_effort')
        self.bridge = CvBridge()
        self.compressed_input = values['compressed_input']
        self.backend = model_factory(
            path, values['device'], values['imgsz'], values['confidence'])
        self.state = dict(
            ready=True, processed=0, errors=0, detections=0,
            predict_wall_ms_total=0.0, parameters=values,
            runtime=self.backend.metadata)
        output_qos = QoSProfile(
            depth=values['queue_depth'],
            reliability=ReliabilityPolicy.RELIABLE)
        self.detection_pub = self.create_publisher(
            Detection2DArray, values['detections_topic'], output_qos)
        self.image_pub = self.create_publisher(
            Image, values['annotated_topic'], output_qos)
        self.subscription = self.create_subscription(
            CompressedImage if self.compressed_input else Image,
            values['input_topic'], self.image_callback,
            QoSProfile(
                depth=values['queue_depth'],
                reliability=policies[values['input_reliability']]))
        self.status_service = self.create_service(
            Trigger, '/yolo/status', self.status_callback)
        self.get_logger().info(
            'YOLO ready: ' + json.dumps(self.backend.metadata))

    def status_callback(self, request, response):
        """Expose counters so replay can detect loss and processing errors."""
        response.success = self.state['errors'] == 0
        response.message = json.dumps(self.state)
        return response

    def image_callback(self, message):
        """Infer once and publish one result pair for each received image."""
        try:
            frame = (
                self.bridge.compressed_imgmsg_to_cv2(
                    message, desired_encoding='bgr8')
                if self.compressed_input else
                self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'))
            result, elapsed = self.backend.predict(frame)
            detections = detection_message(result, message.header)
            annotated = self.bridge.cv2_to_imgmsg(
                result.plot(), encoding='bgr8')
            annotated.header = message.header
            self.detection_pub.publish(detections)
            self.image_pub.publish(annotated)
            self.state['processed'] += 1
            self.state['detections'] += len(detections.detections)
            self.state['predict_wall_ms_total'] += elapsed
        except Exception as error:
            self.state['errors'] += 1
            self.get_logger().error('Image processing failed: ' + str(error))


def main(args=None):
    """Run the detector as an installed ROS executable."""
    rclpy.init(args=args)
    node = None
    try:
        node = YoloPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
