import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os

class ImageSaverNode(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        
        # Define o diretório onde as imagens serão salvas
        self.save_dir = 'zed_images_saved'
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Diretório "{self.save_dir}" criado.')

        # Inscreve-se no tópico da ZED
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/color/rect/image',
            self.image_callback,
            10
        )
        self.subscription  # Evita aviso de variável não utilizada
        
        self.bridge = CvBridge()
        self.image_count = 0
        self.get_logger().info('Nó iniciado. Aguardando imagens...')

    def image_callback(self, msg):
        try:
            # Converte a mensagem ROS para uma imagem OpenCV (BGR para salvar corretamente)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Falha ao converter a imagem: {e}')
            return

        # Gera o nome do arquivo sequencialmente (ex: image_0001.png)
        filename = os.path.join(self.save_dir, f'image_{self.image_count:04d}.png')
        
        # Salva a imagem no disco
        cv2.imwrite(filename, cv_image)
        self.get_logger().info(f'Imagem salva: {filename}')
        
        self.image_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaverNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando o nó com segurança...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()