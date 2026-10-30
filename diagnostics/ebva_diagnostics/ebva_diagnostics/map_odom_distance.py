import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from std_msgs.msg import Float32
import math
import os
from datetime import datetime

class MapOdomDistance(Node):
    def __init__(self):
        super().__init__('map_odom_distance')

        # Parâmetros configuráveis
        self.declare_parameter('log_directory', '/tmp')
        self.declare_parameter('log_filename', 'map_odom_distance.txt')

        log_dir = self.get_parameter('log_directory').get_parameter_value().string_value
        log_file = self.get_parameter('log_filename').get_parameter_value().string_value

        # Garante que a pasta existe
        os.makedirs(log_dir, exist_ok=True)

        self.file_path = os.path.join(log_dir, log_file)

        # Abre arquivo em modo append
        self.log_file = open(self.file_path, 'a')

        self.get_logger().info(f"Salvando log em: {self.file_path}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher_ = self.create_publisher(Float32, '/map_odom_distance', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'odom',
                rclpy.time.Time()
            )

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            distance = math.sqrt(x**2 + y**2 + z**2)

            msg = Float32()
            msg.data = distance
            self.publisher_.publish(msg)

            # Timestamp
            now = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

            # Escreve no arquivo
            self.log_file.write(f"{now}, {distance:.6f}\n")
            self.log_file.flush()  # garante que salva mesmo se travar

            self.get_logger().info(f'Distance: {distance:.4f}m')

        except Exception as e:
            self.get_logger().warn(f'Erro: {e}')

    def destroy_node(self):
        # Fecha arquivo ao encerrar
        if hasattr(self, 'log_file'):
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = MapOdomDistance()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()