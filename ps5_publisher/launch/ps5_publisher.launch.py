import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Declare configurable parameters
    return LaunchDescription([
        

        Node(
            package='ps5_publisher',
            executable='ps5_pub',
            name='ps5_publisher_node',
            output='screen',
            parameters=[{
            }]
        )
    ])
