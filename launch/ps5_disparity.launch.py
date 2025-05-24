from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::DisparityNode',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'stereo_algorithm': LaunchConfiguration('stereo_algorithm'),
                'prefilter_size': LaunchConfiguration('prefilter_size'),
                'prefilter_cap': LaunchConfiguration('prefilter_cap'),
                'correlation_window_size': LaunchConfiguration('correlation_window_size'),
                'min_disparity': LaunchConfiguration('min_disparity'),
                'disparity_range': LaunchConfiguration('disparity_range'),
                'texture_threshold': LaunchConfiguration('texture_threshold'),
                'speckle_size': LaunchConfiguration('speckle_size'),
                'speckle_range': LaunchConfiguration('speckle_range'),
                'disp12_max_diff': LaunchConfiguration('disp12_max_diff'),
                'uniqueness_ratio': LaunchConfiguration('uniqueness_ratio'),
                'P1': LaunchConfiguration('P1'),
                'P2': LaunchConfiguration('P2'),
                'sgbm_mode': LaunchConfiguration('sgbm_mode'),
            }],
            remappings=[
                ('left/image_rect', [LaunchConfiguration('left_namespace'), '/image_rect']),
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/image_rect', [LaunchConfiguration('right_namespace'), '/image_rect']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
            ]
        ),
        ComposableNode(
            package='stereo_image_proc',
            plugin='stereo_image_proc::PointCloudNode',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'approximate_sync': LaunchConfiguration('approximate_sync'),
                'avoid_point_cloud_padding': LaunchConfiguration('avoid_point_cloud_padding'),
                'use_color': LaunchConfiguration('use_color'),
            }],
            remappings=[
                ('left/camera_info', [LaunchConfiguration('left_namespace'), '/camera_info']),
                ('right/camera_info', [LaunchConfiguration('right_namespace'), '/camera_info']),
                ('left/image_rect_color', [LaunchConfiguration('left_namespace'), '/image_rect_color']),
                ('right/image_rect_color', [LaunchConfiguration('right_namespace'), '/image_rect_color']),
            ]
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            name='approximate_sync', default_value='True',
            description='Better tolerance for time variations.'
        ),
        DeclareLaunchArgument(
            name='avoid_point_cloud_padding', default_value='True',
            description='Avoid padding alignment in the generated point cloud.'
        ),
        DeclareLaunchArgument(
            name='use_color', default_value='True',
            description='Generate point cloud with RGB data.'
        ),
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
            description='Name of an existing node container to load launched nodes.'
        ),
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='1',
            description='Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1).'
        ),
        DeclareLaunchArgument(
            name='prefilter_size', default_value='9',
            description='Normalization window size in pixels (must be odd).'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31',
            description='Cap on normalized pixel values.'
        ),
        DeclareLaunchArgument(
            name='correlation_window_size', default_value='5',
            description='Width of the SAD correlation window in pixels (must be odd).'
        ),
        DeclareLaunchArgument(
            name='min_disparity', default_value='0',
            description='Disparity to start the search in pixels.'
        ),
        DeclareLaunchArgument(
            name='disparity_range', default_value='128',
            description='Number of disparities to search in pixels (must be a multiple of 16).'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10',
            description='Filter if the SAD window response does not exceed the texture threshold.'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='200',
            description='Reject regions smaller than this size in pixels.'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='2',
            description='Maximum allowed difference between detected disparities.'
        ),
        DeclareLaunchArgument(
            name='disp12_max_diff', default_value='1',
            description='Maximum allowed difference in left-right disparity check in pixels.'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='12.0',
            description='Filter if the best match does not sufficiently exceed the next best match.'
        ),
        DeclareLaunchArgument(
            name='P1', default_value='1176.0',
            description='First parameter controlling disparity smoothness.'
        ),
        DeclareLaunchArgument(
            name='P2', default_value='2400.0',
            description='Second parameter controlling disparity smoothness.'
        ),
        DeclareLaunchArgument(
            name='sgbm_mode', default_value='1',
            description='SGBM matcher mode to use.'
        ),
        ComposableNodeContainer(
            condition=LaunchConfigurationEquals('container', ''),
            package='rclcpp_components',
            executable='component_container',
            name='stereo_image_proc_container',
            namespace='',
            composable_node_descriptions=composable_nodes,
        ),
        LoadComposableNodes(
            condition=LaunchConfigurationNotEquals('container', ''),
            composable_node_descriptions=composable_nodes,
            target_container=LaunchConfiguration('container'),
        ),
    ])
