from .output_paths import output_path
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32
import csv
import os

class EscuderiaWaypointsGetter(Node):
    def __init__(self):
        super().__init__('escuderia_waypoints_getter')

        self.declare_parameter('target_frame', 'f1tenth_1')
        self.declare_parameter('source_frame', 'map')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        self.source_frame = self.get_parameter('source_frame').get_parameter_value().string_value

        # Lap number
        self.lap_number = 0
        self.create_subscription(Int32, '/autodrive/f1tenth_1/lap_count', self.lap_callback, 10)

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Timer to periodically record transform
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Prepare CSV file
        self.declare_parameter('output_file', '')
        self.csv_path = str(output_path(self.get_parameter('output_file').value))
        self.csv_file = open(self.csv_path, mode='x', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['lap_number', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])  # Header

        self.get_logger().info(f"Logging waypoints to {self.csv_path}")

    def lap_callback(self, msg: Int32):
        self.lap_number = msg.data

    def timer_callback(self):
        try:
            now = Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                now
            )

            t = trans.transform.translation
            r = trans.transform.rotation

            self.csv_writer.writerow([
                self.lap_number, t.x, t.y, t.z, r.x, r.y, r.z, r.w
            ])

            self.get_logger().info(
                f"Lap {self.lap_number} | Pos: ({t.x:.2f}, {t.y:.2f}, {t.z:.2f}) | "
                f"Orient: ({r.x:.2f}, {r.y:.2f}, {r.z:.2f}, {r.w:.2f})"
            )

        except TransformException as ex:
            self.get_logger().warn(f'Could not transform {self.source_frame} -> {self.target_frame}: {ex}')

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info("CSV file closed.")
        super().destroy_node()

def main():
    rclpy.init()
    node = EscuderiaWaypointsGetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

