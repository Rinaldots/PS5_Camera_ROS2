from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Namespace for all loaded components.'
        ),
        DeclareLaunchArgument(
            name='left_namespace', default_value='left',
            description='Namespace for the left camera.'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right',
            description='Namespace for the right camera.'
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'namespace': LaunchConfiguration('left_namespace')}.items()
                ),
            ],
            condition=IfCondition('True'),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'namespace': LaunchConfiguration('right_namespace')}.items()
                ),
            ],
            condition=IfCondition('True'),
        ),
    ])
