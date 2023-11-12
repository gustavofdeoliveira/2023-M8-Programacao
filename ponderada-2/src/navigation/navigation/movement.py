import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
from math import pi
from std_msgs.msg import String


class Movement(Node):

    def __init__(self, navigator):
        super().__init__('movement')
        self.publisher = self.create_publisher(String, 'status', 10)
        self.subscription = self.create_subscription(
            Pose,
            'pose_queue',
            self.listener_callback,
            10
        )
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.navigator = navigator
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        position_x = msg.position.x
        position_y = msg.position.y
        rot_z = 0.0  # Adjust as needed
        pose_destination = create_pose_stamped(self.navigator, position_x, position_y, rot_z)
        self.navigator.goToPose(pose_destination)

        
    def timer_callback(self):
        busy_msg = String()
        busy_msg.status = "Busy" if not self.navigator.isTaskComplete() else "Free"
        self.publisher.publish(busy_msg)

def create_pose_stamped(navigator, position_x, position_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = position_x
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose

def set_initial_pose():
    navigator = BasicNavigator()
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, 0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0
    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    navigator.setInitialPose(initial_pose)
    navigator.waitUntilNav2Active()
    return navigator


def main(args=None):
    rclpy.init(args=args)
    navigator = set_initial_pose()
    navigator_node = Movement(navigator)
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
