import rclpy
import logging
from rclpy.node import Node
from geometry_msgs.msg import Pose
from navigation.publisher import Publisher


class Interface(Node):
    def __init__(self):
        super().__init__("interface")
        self.oracle = Publisher(self, "enqueue", "/enqueue", Pose)
        self.oracle.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        message = Pose()
        message.position.x = float(input('Enter x position: '))
        message.position.y = float(input('Enter y position: '))
        message.position.z = 0.0 

        self.oracle.publish(message)

        self.get_logger().info(f"Pose {message.position.x, message.position.y} published.")

    def destroy(self):
        self.oracle.destroy_pub()
        self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    interface_node = Interface()
    rclpy.spin(interface_node)
    interface_node.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
