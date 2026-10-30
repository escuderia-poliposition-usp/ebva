import rclpy # Biblioteca oficial de ROS2 para escrever nós.
from rclpy.node import Node # Classe base para criar um nó.
from sensor_msgs.msg import LaserScan, Image # Pacote de mensagens padrão com LaserScan sendo a mensagem publicada pelo LiDAR 2D e Image pela câmera.
from message_filters import Subscriber, ApproximateTimeSynchronizer # Pacote para sincronizar, com suas funções bem caracterizadas.
from cv_bridge import CvBridge # Transforma msgs do tipo ROS para opencv com arrays e tals.
import numpy as np
import cv2
import os
from datetime import datetime

from .configuration import load_configuration

class Synchronizer(Node):
    def __init__(self):
        super().__init__('synchronizer')

        config = load_configuration(self.declare_parameter('configuration_file', '').value, 'pairs')
        
        self.lidar_topic = config['lidar']['lidar_topic']
        self.image_topic = config['camera']['camera_topic']
        from pathlib import Path
        from uuid import uuid4
        self.storage_path = str(Path(config['general']['data_folder']) / ('capture_' + uuid4().hex))
        self.slop = config['general']['slop']
        self.max_files = config['general']['max_file_saved']
        self.counter = 0

        if not os.path.exists(self.storage_path):
            os.makedirs(self.storage_path)

        self.get_logger().info(f"Subscribing to {self.lidar_topic} and {self.image_topic}")

        self.lidar_sub = Subscriber(self, LaserScan, self.lidar_topic)
        self.image_sub = Subscriber(self, Image, self.image_topic)

        self.ts = ApproximateTimeSynchronizer(
            [self.lidar_sub, self.image_sub], # Quais tópicos serão comparados.
            queue_size=10, # Quantas msgs recentes ele guarda.
            slop=self.slop # Tolerância no timestamp (segundos).
        )
        self.ts.registerCallback(self.synchronize_data) # Sempre que encontrar um par de dados bons, ele chama a função synchronized_data


    def synchronize_data(self, lidar_msg, image_msg): # Função salva os dados dos tópicos sempre que eles estiver sincronizados, salvando na pasta /data
        if self.counter >= self.max_files:
            self.get_logger().info("Max files reached. Exiting ...")
            rclpy.shutdown()
            return

        file_name = f"{datetime.now().strftime('%Y%m%d_%H%M%S')}_{self.counter}"

        self.save_data(image_msg, lidar_msg, file_name)

        self.get_logger().info(f"Synchronized pair saved {self.counter+1}/{self.max_files}: {file_name}")
        self.counter += 1


    def save_data(self, image_msg, laserscan_msg, file_name):
        bridge = CvBridge()
        
        image = bridge.imgmsg_to_cv2(image_msg, 'bgr8')
        if not cv2.imwrite(f'{self.storage_path}/{file_name}.png', image):
            raise IOError('Could not save synchronized image')

        ranges = np.array(laserscan_msg.ranges)
        angles = np.arange(len(ranges)) * laserscan_msg.angle_increment + laserscan_msg.angle_min
        np.savez_compressed(f'{self.storage_path}/{file_name}_lidar.npz', ranges = ranges, angles = angles,
                            image_stamp_sec=image_msg.header.stamp.sec,
                            image_stamp_nanosec=image_msg.header.stamp.nanosec,
                            scan_stamp_sec=laserscan_msg.header.stamp.sec,
                            scan_stamp_nanosec=laserscan_msg.header.stamp.nanosec,
                            image_frame=image_msg.header.frame_id, scan_frame=laserscan_msg.header.frame_id,
                            range_min=laserscan_msg.range_min,range_max=laserscan_msg.range_max)

def main(args=None):
    rclpy.init(args=args) # Inicia o sistema de comunicação.
    node = Synchronizer() # Instancia.
    try:
        rclpy.spin(node) # Fica loopando a leitura das msgs, se não tiver loop ele só morre.
    except KeyboardInterrupt: # Ctrl + C.
        pass
    finally:
        node.destroy_node() # Mata o nó.
        rclpy.shutdown() # Mata o sistema.

if __name__ == '__main__':
    main()
