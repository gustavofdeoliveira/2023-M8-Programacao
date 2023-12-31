import rclpy
from enum import Enum
from queue import Queue
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from navigation.subscriber import Subscriber
from navigation.publisher import Publisher


class RobotStatus(Enum):
    FREE = 0
    BUSY = 1


class PoseQueue(Node):
    def __init__(self):
        super().__init__("pose_queue")
        self.enqueue = Subscriber(self, "enqueue", "/enqueue", Pose)
        self.status = Subscriber(self, "status", "/status", String)
        self.dequeue = Publisher(self, "dequeue", "/dequeue", Pose)

        self.enqueue.create_sub(self.enqueue_pose)
        self.status.create_sub(self.status_callback)

        self.queue = Queue()
        self.robot_status = RobotStatus.FREE

    def enqueue_pose(self, message) -> None:
        self.queue.put(message)

    def status_callback(self, message: String) -> None:
        self.robot_status = RobotStatus.FREE if message.data == "FREE" else RobotStatus.BUSY

        if self.robot_status == RobotStatus.BUSY:
            self.get_logger().info(f"Robot is busy, waiting for it to be free.")
        elif self.robot_status == RobotStatus.FREE and not self.queue.empty():
            next_pose = self.queue.get()
            self.dequeue.publish(next_pose)
            self.get_logger().info(f"Publishing {next_pose} on /dequeue")
        elif self.robot_status == RobotStatus.FREE and self.queue.empty():
            self.get_logger().info("Queue is empty, waiting for new poses.")
        else:
            self.get_logger().info("Something went wrong.")


def main(args=None):
    rclpy.init(args=args)
    pose_queue_node = PoseQueue()
    rclpy.spin(pose_queue_node)
    pose_queue_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
