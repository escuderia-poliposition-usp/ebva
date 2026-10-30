import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
from datetime import datetime
from .output_paths import output_path

class OdomToCSV(Node):
    def __init__(self):
        super().__init__('odom_to_csv')

        # Nome do arquivo com timestamp
        now = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.declare_parameter('output_file', '')
        self.declare_parameter('topic', '/odom')
        self.filename = str(output_path(self.get_parameter('output_file').value))

        # Abre o arquivo
        self.file = open(self.filename, mode='x', newline='')
        self.writer = csv.writer(self.file)

        # Cabeçalho
        self.writer.writerow([
            'time_sec',
            'pos_x', 'pos_y', 'pos_z',
            'orient_x', 'orient_y', 'orient_z', 'orient_w'
        ])

        # Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            self.get_parameter('topic').value,
            self.odom_callback,
            10
        )

        self.get_logger().info(f'Salvando dados em {self.filename}')

    def odom_callback(self, msg):
        # Tempo
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Posição
        pos = msg.pose.pose.position

        # Orientação (quaternion)
        orient = msg.pose.pose.orientation

        # Escreve linha no CSV
        self.writer.writerow([
            t,
            pos.x, pos.y, pos.z,
            orient.x, orient.y, orient.z, orient.w
        ])

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OdomToCSV()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()