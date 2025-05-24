from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='isaac_rectify_left',
            namespace=LaunchConfiguration('left_namespace'),
            parameters=[{
                'output_width': 1280,
                'output_height': 720,
            }],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
                ('camera_info_rect', 'camera_info_rect'),
            ],
        ),
        ComposableNode(
            package='isaac_ros_image_proc',
            plugin='nvidia::isaac_ros::image_proc::RectifyNode',
            name='isaac_rectify_right',
            namespace=LaunchConfiguration('right_namespace'),
            parameters=[{
                'output_width': 1280,
                'output_height': 720,
            }],
            remappings=[
                ('image', 'image_raw'),
                ('camera_info', 'camera_info'),
                ('image_rect', 'image_rect'),
                ('camera_info_rect', 'camera_info_rect'),
            ],
        ),
    ]

    load_composable_nodes = LoadComposableNodes(
        composable_node_descriptions=composable_nodes,
        target_container=LaunchConfiguration('container'),
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('container'), "' != ''"]))
    )

    image_processing_container = ComposableNodeContainer(
        name='image_proc_container',
        namespace=LaunchConfiguration('namespace'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
        condition=UnlessCondition(PythonExpression(["'", LaunchConfiguration('container'), "' != ''"]))
    )

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
        DeclareLaunchArgument(
            name='container', default_value='',
            description='Container name to load composable nodes into (leave empty to create a new container).'
        ),
        image_processing_container,
        load_composable_nodes,
    ])
