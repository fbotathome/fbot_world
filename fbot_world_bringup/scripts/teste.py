#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class PoseWithCovariancePublisher(Node):
    """
    A ROS 2 node that publishes a fixed PoseWithCovarianceStamped message on the /target_pose topic.
    """

    def __init__(self):
        super().__init__('pose_with_covariance_publisher')

        # Cria um publisher para PoseWithCovarianceStamped
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/target_pose', 10)

        # Publica a cada 0.5 segundos
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.publish_pose)

        self.get_logger().info('PoseWithCovariance publisher node started.')

    def publish_pose(self):
        # Cria a mensagem
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'  # Modifique conforme o frame desejado

        msg.pose.pose.position.x = -11.205980123210482
        msg.pose.pose.position.y = -5.971964121576364
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = -0.2535824278252746
        msg.pose.pose.orientation.w = 0.967313781716274

        # Define uma matriz de covariância (exemplo: identidade simples nas posições)
        msg.pose.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1,
        ]

        # Publica a mensagem
        self.publisher_.publish(msg)
        self.get_logger().info('Published PoseWithCovarianceStamped to /target_pose')


def main(args=None):
    rclpy.init(args=args)
    node = PoseWithCovariancePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()