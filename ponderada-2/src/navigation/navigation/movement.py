import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler
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

    def listener_callback(self, message):
        position_x = message.position.x
        position_y = message.position.y
        rot_z = 0.0  # Adjust as needed
        pose_destination = create_pose_stamped(self.navigator, position_x, position_y, rot_z)
        self.navigator.goToPose(pose_destination)

        
    def timer_callback(self):
        busy_msg = String()
        busy_msg.data = "Busy" if not self.navigator.isTaskComplete() else "Free"
        self.publisher.publish(busy_msg)

def create_pose_stamped(navigator, position_x, position_y, rot_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rot_z)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = position_x
    goal_pose.pose.position.y = position_y
    goal_pose.pose.position.z = position_x
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    return goal_pose


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator_node = Movement(navigator)
    rclpy.spin(navigator_node)
    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
