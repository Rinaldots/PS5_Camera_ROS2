import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ps5_publisher',
            executable='ps5_pub',
            name='ps5_publisher_node',
            output='screen',
        ),

    ])
