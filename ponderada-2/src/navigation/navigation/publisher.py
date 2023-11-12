#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_messages.message import Pose


class Publisher(Node):

    def __init__(self):
        super().__init__('topic_publisher')
        self.publisher_ = self.create_publisher(Pose, 'topic_publisher', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        message = Pose()
        message.position.x = float(input('Enter x position: '))
        message.position.y = float(input('Enter y position: '))
        message.position.z = float(input('Enter z position: '))
        self.publisher_.publish(message)
        self.get_logger().info('Publishing Pose: x={}, y={}, z={}'.format(message.position.x, message.position.y, message.position.z))


def main(args=None):
    rclpy.init(args=args)
    my_publisher = MyPublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()