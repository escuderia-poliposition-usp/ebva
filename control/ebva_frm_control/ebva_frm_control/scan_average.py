import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
#from nav_msgs.msg import Odometry, Path
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
#from geometry_msgs.msg import Point
import numpy as np



class ControleNode(Node):
    def __init__(self):
        super().__init__('controle')
        
        # Criação dos publishers para os tópicos de direção e aceleração
        self.steer_publisher = self.create_publisher(Float32,'/autodrive/f1tenth_1/steering_command',10)
        self.throttle_publisher = self.create_publisher(Float32,'/autodrive/f1tenth_1/throttle_command',10)
        #self.throttle_publisher = self.create_publisher(Float64, '/commands/motor/speed', 10)
        #self.steer_publisher = self.create_publisher(Float64, '/commands/servo/position', 10)
        
        #Lidar Subscriber
        self.lidar_subscriber = self.create_subscription(LaserScan,'/scan',self.lidar_callback,10)
        self.lidar_subscriber

        # Definição dos valores constantes
        self.steer_value = 0.0  # Valor constante de direção (em radianos)
        self.throttle_value = 0.06 # Valor constante de aceleração (entre 0 e 1)
        self.e1_1 = 0.0
        self.e2_1 = 0.0
        self.int_e1 = 0.0
        self.int_e2 = 0.0
        self.distance_front_1 = 0.0
        self.get_logger().info('Node de controle inicializado.')

    def media_lidar(self, pos_central, numero_pontos, lidar):
        media = 0
        for i in range(numero_pontos):
            leitura = lidar[pos_central + i - (int)(numero_pontos/2)]
            if leitura > 10:
                leitura = 10
            media += leitura
        return media/numero_pontos
    
    def lidar_callback(self, msg):
        # Esta função é chamada toda vez que uma nova mensagem do LIDAR é recebida
        # Acessando os dados do LIDAR
        steer_msg = Float32()
        throttle_msg = Float32()
        ranges = msg.ranges
        #intensities = msg.intensities

        # Parâmetros do controle proporcional
        K1 = 0.6 #0.8 # Ganho proporcional para o erro e1 (0.6 e 2 funciona)
        K3 = 4.0
        K5 = 0.0

        K2 = 0.02 # 0.002 0.03 # Ganho proporcional para o erro e2
        K4 = 0.3 #0.4     0.5
        K6 = 0.0
        
        # Exemplo de como usar os dados:
        # Pega a distância do primeiro sensor (direto à frente)
        #distance_front = ranges[540]
        #d_m90 = ranges[180]
        #d_90  = ranges[900]
        #d_m30 = ranges[360] #420
        #d_30  = ranges[720] #660

        n_media_lidar = 10

        distance_front = self.media_lidar(540,n_media_lidar,ranges)
        d_m90 = self.media_lidar(180,n_media_lidar,ranges)
        d_90  = self.media_lidar(900,n_media_lidar,ranges)
        d_m30 = self.media_lidar(360,n_media_lidar,ranges)
        d_30  = self.media_lidar(720,n_media_lidar,ranges)

        # CONTROLE
        # 2. Cálculo dos erros
        #e1 = (d_90 - d_m90)/abs(max(d_90,d_m90))
        #e2 = (d_30 - d_m30)/abs(max(d_30,d_m30))
        e1 = d_90 - d_m90
        e2 = d_30 - d_m30

        if e1 > 10.0:
            e1 = 10.0
        elif e1 < -10.0:
            e1 = -10.0
        if e2 > 10.0:
            e2 = 10.0
        elif e2 < -10.0:
            e2 = -10.0
        
        de1 = e1 - self.e1_1
        de2 = e2 - self.e2_1

        alpha = 0.2
        self.int_e1 = (1-alpha)*self.int_e1 + alpha*e1
        self.int_e2 = (1-alpha)*self.int_e2 + alpha*e2
        int1 = self.int_e1
        int2 = self.int_e2

        self.e1_1 = e1
        self.e2_1 = e2

        # 3. Dinâmica do controle
        d_yaw = K1*e1 + K3*de1 + K5*int1 + K2*e2 + K4*de2 + K6*int2 # Derivada do ângulo de yaw

        D_YAW_MAX = 1.0
        D_YAW_MIN = -1.0

        if d_yaw > D_YAW_MAX:
            d_yaw = D_YAW_MAX
        elif d_yaw < -D_YAW_MAX:
            d_yaw = -D_YAW_MAX

        self.steer_value = d_yaw

        # Variáveis controlador
        kp = 0.1
        #ki = 0.2
        kd = 0.01
        a_max  = 0.2

        if  distance_front > 10:
            distance_front = 10

        d_distance = distance_front-self.distance_front_1

        a_frenagem = kp*distance_front + kd*(d_distance)
        self.distance_front_1 = distance_front

        if(a_frenagem < -a_max):
            a_frenagem = -a_max
        elif(a_frenagem > a_max):
            a_frenagem = a_max

        self.throttle_value = a_frenagem

        steer_msg.data = self.steer_value
        self.steer_publisher.publish(steer_msg)

        throttle_msg.data = self.throttle_value
        self.throttle_publisher.publish(throttle_msg)
        
        # Prints no terminal
        #self.get_logger().info(f'f= {distance_front:.2f}m, -90 = {d_m90}, 90 = {d_90}, -30 = {d_m30}, 30 = {d_30}')
        #self.get_logger().info(f'E1 = {e1:.2f}, dE1 = {de1:.2}, Ie1 = {int1:.2}, E2 = {e2:.2f}, dE2 = {de2:.2}, d_yaw = {d_yaw:.2}')
        self.get_logger().info(f'd = {distance_front:.2f}, d_m90 = {d_m90:.2}, d_m30 = {d_m30:.2}, d_30 = {d_30:.2}, d_90 = {d_90:.2},a = {a_frenagem:.2}')


def main(args=None):
    rclpy.init(args=args)
    node = ControleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
