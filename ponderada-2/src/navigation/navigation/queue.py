import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from queue import Queue

class PoseQueue(Node):

    def __init__(self):
        super().__init__('pose_queue')
        self.pose_queue = Queue()
        self.navigator_status = True
        self.create_subscription(Pose, 'topic_publisher', self.cli_callback, 10)
        self.create_subscription(String, 'status', self.status_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, 'pose_queue', 10)

    def cli_callback(self, msg):
        self.pose_queue.put(msg)

    def status_callback(self, msg):
        self.navigator_status = True if msg.data == 'Free' else False
        if self.navigator_status and not self.pose_queue.empty():
            next_pose = self.pose_queue.get()
            print(f'Sending new pose: {next_pose}')
            self.pose_publisher.publish(next_pose)

def main(args=None):
    rclpy.init(args=args)
    node = PoseQueue()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
