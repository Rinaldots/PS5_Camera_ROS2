from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetLaunchConfiguration
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


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
            name='approximate_sync', default_value='False',
            description='Whether to use approximate synchronization of topics.'
        ),
        DeclareLaunchArgument(
            name='avoid_point_cloud_padding', default_value='False',
            description='Avoid alignment padding in the generated point cloud.'
        ),
        DeclareLaunchArgument(
            name='use_color', default_value='True',
            description='Generate point cloud with RGB data.'
        ),
        DeclareLaunchArgument(
            name='launch_image_proc', default_value='True',
            description='Whether to launch debayer and rectify nodes from image_proc.'
        ),
        DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Namespace for all components loaded.'
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
            description='Name of an existing node container to load launched nodes into.'
        ),
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='0',
            description='Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1).'
        ),
        DeclareLaunchArgument(
            name='prefilter_size', default_value='9',
            description='Normalization window size in pixels (must be odd).'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31',
            description='Bound on normalized pixel values.'
        ),
        DeclareLaunchArgument(
            name='correlation_window_size', default_value='15',
            description='SAD correlation window width in pixels (must be odd).'
        ),
        DeclareLaunchArgument(
            name='min_disparity', default_value='0',
            description='Disparity to begin search at in pixels.'
        ),
        DeclareLaunchArgument(
            name='disparity_range', default_value='64',
            description='Number of disparities to search in pixels (must be a multiple of 16).'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10',
            description='Filter out if SAD window response does not exceed texture threshold.'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='100',
            description='Reject regions smaller than this size in pixels.'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='4',
            description='Maximum allowed difference between detected disparities.'
        ),
        DeclareLaunchArgument(
            name='disp12_max_diff', default_value='0',
            description='Maximum allowed difference in the left-right disparity check in pixels.'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='15.0',
            description='Filter out if best match does not sufficiently exceed the next-best match.'
        ),
        DeclareLaunchArgument(
            name='P1', default_value='200.0',
            description='The first parameter controlling the disparity smoothness.'
        ),
        DeclareLaunchArgument(
            name='P2', default_value='400.0',
            description='The second parameter controlling the disparity smoothness.'
        ),
        DeclareLaunchArgument(
            name='sgbm_mode', default_value='0',
            description='The mode of the SGBM matcher to be used.'
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
        SetLaunchConfiguration(
            condition=LaunchConfigurationEquals('container', ''),
            name='container',
            value=PythonExpression([
                '"stereo_image_proc_container"', ' if ',
                '"', LaunchConfiguration('namespace', default=''), '"',
                ' == "" else ', '"',
                LaunchConfiguration('namespace', default=''), '/stereo_image_proc_container"'
            ]),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container'),
                                      'namespace': LaunchConfiguration('left_namespace')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        ),
        GroupAction(
            [
                PushRosNamespace(LaunchConfiguration('namespace')),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        FindPackageShare('image_proc'), '/launch/image_proc.launch.py'
                    ]),
                    launch_arguments={'container': LaunchConfiguration('container'),
                                      'namespace': LaunchConfiguration('right_namespace')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        ),
    ])
