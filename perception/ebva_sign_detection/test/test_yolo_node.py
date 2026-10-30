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

"""Functional ROS contracts using a deterministic backend, without GPU."""

import importlib
from pathlib import Path
import tempfile
import time
import unittest
from types import SimpleNamespace

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray

YoloPublisherNode = importlib.import_module(
    'ebva_sign_detection.yolo_node').YoloPublisherNode


class FakeBackend:
    def __init__(self, path, device, imgsz, confidence):
        self.metadata = dict(device='test-backend')
        self.fail = False

    def predict(self, frame):
        if self.fail:
            raise RuntimeError('test inference failure')
        box = SimpleNamespace(
            xywh=np.array([[10., 11., 12., 13.]]),
            cls=np.array([2]), conf=np.array([0.8]))
        return SimpleNamespace(
            boxes=[box], plot=lambda: frame.copy()), 1.5


class TestYoloRos(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.temp = tempfile.TemporaryDirectory()
        self.checkpoint = Path(self.temp.name) / 'test.pt'
        self.checkpoint.write_bytes(b'test backend placeholder')
        self.executor = SingleThreadedExecutor()
        self.nodes = []

    def tearDown(self):
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        self.temp.cleanup()

    def wait(self, condition):
        deadline = time.monotonic() + 8
        while not condition() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
        self.assertTrue(condition())

    def prepare(self, compressed=False):
        overrides = [
            Parameter('model_path', value=str(self.checkpoint)),
            Parameter('input_topic', value='/test/yolo/input'),
            Parameter('compressed_input', value=compressed),
            Parameter('input_reliability', value='reliable'),
        ]
        detector = YoloPublisherNode(
            model_factory=FakeBackend, parameter_overrides=overrides)
        peer = Node('yolo_contract_peer')
        from sensor_msgs.msg import CompressedImage
        publisher = peer.create_publisher(
            CompressedImage if compressed else Image, '/test/yolo/input', 10)
        images, detections = [], []
        peer.create_subscription(
            Image, '/yolo/annotated_image', images.append, 10)
        peer.create_subscription(
            Detection2DArray, '/yolo/detections', detections.append, 10)
        self.nodes = [detector, peer]
        for node in self.nodes:
            self.executor.add_node(node)
        self.wait(lambda: (
            publisher.get_subscription_count() == 1
            and detector.image_pub.get_subscription_count() == 1
            and detector.detection_pub.get_subscription_count() == 1))
        bridge = CvBridge()
        frame = np.full((32, 32, 3), 60, dtype=np.uint8)
        message = (
            bridge.cv2_to_compressed_imgmsg(frame)
            if compressed else bridge.cv2_to_imgmsg(frame, encoding='bgr8'))
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = 456
        message.header.frame_id = 'camera_original'
        return detector, publisher, message, images, detections

    def check_pair(self, images, detections):
        self.assertEqual(images[0].header, detections[0].header)
        self.assertEqual(images[0].header.stamp.sec, 123)
        self.assertEqual(images[0].header.stamp.nanosec, 456)
        self.assertEqual(images[0].header.frame_id, 'camera_original')
        item = detections[0].detections[0]
        self.assertEqual(item.bbox.center.position.x, 10.)
        self.assertEqual(item.bbox.center.position.y, 11.)
        self.assertEqual(item.bbox.size_x, 12.)
        self.assertEqual(item.bbox.size_y, 13.)
        self.assertEqual(item.results[0].hypothesis.class_id, '2')
        self.assertAlmostEqual(item.results[0].hypothesis.score, 0.8)
        self.assertEqual(images[0].encoding, 'bgr8')

    def test_raw_image_is_published_with_original_header(self):
        detector, publisher, message, images, detections = self.prepare()
        publisher.publish(message)
        self.wait(lambda: len(images) == len(detections) == 1)
        self.check_pair(images, detections)
        self.assertEqual(detector.state['parameters']['device'], 'cpu')
        self.assertEqual(detector.state['processed'], 1)
        self.assertEqual(detector.state['errors'], 0)

    def test_compressed_image_is_decoded_and_published(self):
        detector, publisher, message, images, detections = self.prepare(True)
        publisher.publish(message)
        self.wait(lambda: len(images) == len(detections) == 1)
        self.check_pair(images, detections)
        self.assertEqual(detector.state['parameters']['device'], 'cpu')
        self.assertEqual(detector.state['processed'], 1)

    def test_inference_error_is_counted_without_false_output(self):
        detector, publisher, message, images, detections = self.prepare()
        detector.backend.fail = True
        publisher.publish(message)
        self.wait(lambda: detector.state['errors'] == 1)
        self.assertEqual(detector.state['processed'], 0)
        self.assertEqual(images, [])
        self.assertEqual(detections, [])
