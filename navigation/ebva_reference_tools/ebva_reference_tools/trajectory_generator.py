import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, PoseStamped

class PathPublisherNode(Node):
    def __init__(self):
        super().__init__('path_publisher_node')
        
        # Cria o subscriber para o tópico de IPS
        self.subscription = self.create_subscription(
            Point,
            '/autodrive/f1tenth_1/ips',
            self.ips_callback,
            10
        )
        self.subscription  # Previne aviso de variável não utilizada

        # Cria o publisher para o tópico do RViz
        self.path_publisher = self.create_publisher(Path, '/autodrive/f1tenth_1/path', 10)
        
        # Inicializa a mensagem Path que irá acumular as poses
        self.path_msg = Path()
        # O frame_id é crucial para o RViz. Deve ser o mesmo frame de referência
        # usado no simulador, que geralmente é 'map' ou o frame raiz.
        self.path_msg.header.frame_id = 'map'
        
        self.get_logger().info('Nó Path Publisher iniciado. Publicando em /autodrive/f1tenth_1/path')

    def ips_callback(self, msg):
        # 1. Cria uma nova mensagem PoseStamped a partir do Point recebido
        pose_stamped = PoseStamped()
        
        # Adiciona o timestamp e o frame_id para a nova pose
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = self.path_msg.header.frame_id
        
        # Preenche a posição (x, y, z)
        pose_stamped.pose.position.x = msg.x
        pose_stamped.pose.position.y = msg.y
        pose_stamped.pose.position.z = msg.z
        
        # A orientação pode ser deixada como padrão (quaternion de identidade)
        # já que o Point não fornece essa informação.
        pose_stamped.pose.orientation.w = 1.0

        # 2. Adiciona a nova pose à lista de poses da mensagem Path
        self.path_msg.poses.append(pose_stamped)
        
        # 3. Publica a mensagem Path completa
        self.path_publisher.publish(self.path_msg)
        
        # Opcional: log para confirmar que a mensagem foi publicada
        # self.get_logger().info(f'Ponto adicionado e Path publicado. Total de pontos: {len(self.path_msg.poses)}')

def main(args=None):
    rclpy.init(args=args)
    path_publisher_node = PathPublisherNode()
    rclpy.spin(path_publisher_node)
    path_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()