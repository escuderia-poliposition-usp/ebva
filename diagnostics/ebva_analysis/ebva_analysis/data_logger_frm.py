import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point
from std_msgs.msg import Float32
import csv
import os
from .output_paths import output_path

class IpsLoggerNode(Node):
    def __init__(self):
        super().__init__('ips_logger_node')
        
        self.declare_parameter('output_file', '')
        self.output_file = self.get_parameter('output_file').get_parameter_value().string_value
        
        # Obter o caminho completo para o diretório do pacote
        self.output_file_path = str(output_path(self.output_file))

        # Variável para armazenar a última velocidade recebida
        self.current_speed = 0.0

        # Subscriber para a posição (IPS)
        self.subscription_ips = self.create_subscription(
            Point,
            '/autodrive/f1tenth_1/ips',
            self.ips_callback,
            10
        )
        
        # Novo subscriber para a velocidade
        self.subscription_speed = self.create_subscription(
            Float32,
            '/autodrive/f1tenth_1/speed',
            self.speed_callback,
            10
        )
        
        # Inicializar o arquivo CSV e escrever o novo cabeçalho com 'speed'
        with open(self.output_file_path, mode='x', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'speed'])
        
        self.get_logger().info(f'Nó IPS Logger iniciado. Salvando dados em: {self.output_file_path}')

    def ips_callback(self, msg):
        """
        Callback para a posição. Salva a posição junto com a última velocidade conhecida.
        """
        # Obter o timestamp atual com precisão de nanosegundos
        current_time_nanos = self.get_clock().now().nanoseconds
        timestamp_sec = current_time_nanos / 1e9

        # Escrever os dados no arquivo CSV, incluindo a velocidade
        with open(self.output_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp_sec, msg.x, msg.y, msg.z, self.current_speed])
            
        self.get_logger().info(
            f'Dados salvos: tempo={timestamp_sec:.6f}, x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}, speed={self.current_speed:.2f}'
        )

    def speed_callback(self, msg):
        """
        Callback para a velocidade. Apenas atualiza o valor da velocidade.
        """
        self.current_speed = msg.data

def main(args=None):
    rclpy.init(args=args)
    ips_logger_node = IpsLoggerNode()
    rclpy.spin(ips_logger_node)
    ips_logger_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
