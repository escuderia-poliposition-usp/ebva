import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.timer = self.create_timer(3.0, self.publish_pose)
        self.published = False

    def publish_pose(self):

        if self.published:
            return
        
        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Matriz 6x6 representando incerteza na posição e orientação (x, y, z, roll, pitch, yaw), quanto menor mais preciso
        # Como sempre tentaremos começar com o carrinho na mesma posição da pista, um covariance pequeno deve servir bem.
        msg.pose.covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.2
        ]

        self.publisher.publish(msg)

        self.get_logger().info('Initial pose published')

        self.published = True

def main(args=None):

    rclpy.init(args=args)

    node = InitialPosePublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
