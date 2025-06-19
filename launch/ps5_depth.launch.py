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
            name='approximate_sync', default_value='True',
            description='Melhor tolerância a variações de tempo.'
        ),
        DeclareLaunchArgument(
            name='avoid_point_cloud_padding', default_value='True',
            description='Evitar preenchimento de alinhamento na nuvem de pontos gerada.'
        ),
        DeclareLaunchArgument(
            name='use_color', default_value='True',
            description='Gerar nuvem de pontos com dados RGB.'
        ),
        DeclareLaunchArgument(
            name='launch_image_proc', default_value='True',
            description='Se deve lançar os nós de debayer e retificação do image_proc.'
        ),
        DeclareLaunchArgument(
            name='namespace', default_value='',
            description='Namespace para todos os componentes carregados.'
        ),
        DeclareLaunchArgument(
            name='left_namespace', default_value='left',
            description='Namespace para a câmera esquerda.'
        ),
        DeclareLaunchArgument(
            name='right_namespace', default_value='right',
            description='Namespace para a câmera direita.'
        ),
        DeclareLaunchArgument(
            name='container', default_value='',
            description='Nome de um container de nó existente para carregar os nós lançados.'
        ),
        DeclareLaunchArgument(
            name='stereo_algorithm', default_value='1',
            description='Algoritmo estéreo: Block Matching (0) ou Semi-Global Block Matching (1).'
        ),
        DeclareLaunchArgument(
            name='prefilter_size', default_value='9',
            description='Tamanho da janela de normalização para BM (stereo_algorithm=0) em pixels (deve ser ímpar).'
        ),
        DeclareLaunchArgument(
            name='prefilter_cap', default_value='31',
            description='Limite nos valores de pixel normalizados.'
        ),
        DeclareLaunchArgument(
            name='correlation_window_size', default_value='5',
            description='Tamanho do bloco de correspondência (SAD window para BM, blockSize para SGBM, deve ser ímpar).'
        ),
        DeclareLaunchArgument(
            name='min_disparity', default_value='0',
            description='Disparidade para iniciar a busca em pixels.'
        ),
        DeclareLaunchArgument(
            name='disparity_range', default_value='128',
            description='Número de disparidades para buscar em pixels (deve ser múltiplo de 16).'
        ),
        DeclareLaunchArgument(
            name='texture_threshold', default_value='10',
            description='Limite de textura para BM (stereo_algorithm=0). Regiões com textura abaixo são filtradas.'
        ),
        DeclareLaunchArgument(
            name='speckle_size', default_value='200',
            description='Rejeitar regiões menores que este tamanho em pixels.'
        ),
        DeclareLaunchArgument(
            name='speckle_range', default_value='2',
            description='Diferença máxima permitida entre disparidades detectadas para considerá-las parte da mesma região (filtragem de speckle).'
        ),
        DeclareLaunchArgument(
            name='disp12_max_diff', default_value='1',
            description='Diferença máxima permitida na verificação de disparidade esquerda-direita em pixels.'
        ),
        DeclareLaunchArgument(
            name='uniqueness_ratio', default_value='12.0',
            description='Filtrar se a melhor correspondência (menor custo) não for suficientemente melhor que a segunda melhor. (Ex: (cost2-cost1)/cost1 > uniqueness_ratio/100)'
        ),
        DeclareLaunchArgument(
            name='P1', default_value='200.0', # Anteriormente 1176.0
            description='Parâmetro P1 do SGBM (controla suavidade). Típico: 8*canais*blockSize^2. Aumentar para mais suavidade.'
        ),
        DeclareLaunchArgument(
            name='P2', default_value='800.0', # Anteriormente 2400.0
            description='Parâmetro P2 do SGBM (controla suavidade). Típico: 32*canais*blockSize^2. Aumentar para mais suavidade. Deve ser > P1.'
        ),
        DeclareLaunchArgument(
            name='sgbm_mode', default_value='1',
            description='O modo do matcher SGBM a ser usado.'
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
                                      'namespace': LaunchConfiguration('left_namespace'),
                                      'approximate_sync': LaunchConfiguration('approximate_sync')}.items()
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
                                      'namespace': LaunchConfiguration('right_namespace'),
                                      'approximate_sync': LaunchConfiguration('approximate_sync')}.items()
                ),
            ],
            condition=IfCondition(LaunchConfiguration('launch_image_proc')),
        ),
    ])
