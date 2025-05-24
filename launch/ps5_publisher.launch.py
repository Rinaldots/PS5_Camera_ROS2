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
            parameters=[
                {'video_source': '0'},  # Default to camera index 0
                {'resolution_scale': 640 / 2560}  # Scale to 640p
            ]
        ),

    ])
