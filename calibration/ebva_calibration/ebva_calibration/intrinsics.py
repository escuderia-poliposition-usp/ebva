import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber
from cv_bridge import CvBridge
import cv2
import yaml
import numpy as np
from datetime import datetime

from .configuration import load_configuration
from .intrinsic_math import checkerboard_points, fit_intrinsics

class CameraCalibration(Node):
    def __init__(self):
        super().__init__('camera_calibration')

        config = load_configuration(self.declare_parameter('configuration_file', '').value, 'intrinsic')

        self.image_topic = config['camera']['camera_topic']
        self.image_width = config['camera']['image_size']['width']
        self.image_length = config['camera']['image_size']['length']

        self.chessboard_rows = config['chessboard']['rows']
        self.chessboard_columns = config['chessboard']['columns']
        self.square_size = config['chessboard']['square_size_m']

        self.output_path = config['general']['config_folder']
        self.file = config['general']['camera_intrinsic_calibration']

        self.image_sub = Subscriber(self, Image, self.image_topic)
        self.image_sub.registerCallback(self.image_callback)
        self.show_debug = self.declare_parameter('show_debug', False).value
        self.bridge = CvBridge()

        self.obj_points = []
        self.img_points = []

        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        
        self.objp = checkerboard_points(self.chessboard_rows, self.chessboard_columns, self.square_size)

        self.get_logger().info("Camera calibration node initialized. Waiting for images...")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(gray, (self.chessboard_columns, self.chessboard_rows), None)

            if ret:
                self.obj_points.append(self.objp)
                refined_corners = cv2.cornerSubPix(gray, corners, (11,11), (-1, -1), self.criteria)
                self.img_points.append(refined_corners)

                cv2.drawChessboardCorners(cv_image, (self.chessboard_columns, self.chessboard_rows), refined_corners, ret)
                self.get_logger().info("Chessboard detected and points added.")
            else:
                self.get_logger().warn("Chessboard not detected.")
            
            if self.show_debug:
                cv2.imshow("Image", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process the image: {e}")

    def save_calibration(self):
        if len(self.obj_points) < 10:
            self.get_logger().error("Not enough images for calibration. At least 10 are required.")
            return
    
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = fit_intrinsics(self.obj_points, self.img_points, (self.image_width, self.image_length))

        calibrationdata = {
            'calibration_date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'camera_matrix': {
                'rows' : 3,
                'columns' : 3,
                'data' :  camera_matrix.tolist()
            },
            'distortion_coefficients': {
                'rows': 1,
                'columns': len(dist_coeffs[0]),
                'data': dist_coeffs[0].tolist()
            },
            'image_size': {
                'width': self.image_width,
                'length': self.image_length,
            },
            'rsm_reprojection_error': ret
        }

        output_file = f"{self.output_path}/{self.file}"
        try:
            from pathlib import Path
            Path(self.output_path).mkdir(parents=True, exist_ok=True)
            with open(output_file, 'x') as file:
                yaml.dump(calibrationdata, file)
            self.get_logger().info(f"calibration saved to {output_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to save the calibration: {e}")

def main(args = None):
    rclpy.init(args=args)
    node = CameraCalibration()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_calibration()
        node.get_logger().info("Calibration process completed.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
