import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='navigation',
            executable='movement',
            output='screen',
            name="movement"
        ),
        Node(
            package='navigation',
            executable='topic_publisher',
            output='screen',
            name="topic_publisher",
        ),
        Node(
            package='navigation',
            executable='pose_queue',
            output='screen',
            name="pose_queue",
            ),
            
    ])