from multiprocessing import Value
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    stdout = LaunchConfiguration('stdout')
    camera_name = LaunchConfiguration('camera_name')
    frame_name = LaunchConfiguration('frame_name')
    sensor_id = LaunchConfiguration('sensor_id')

    # args that can be set from the command line or a default will be used
    stdout_launch_arg = DeclareLaunchArgument(
        'stdout', default_value='screen'
    )
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value='nerian_stereo'
    )
    frame_name_launch_arg = DeclareLaunchArgument(
        "frame_name", default_value='camera_link'
    )
    sensor_id_launch_arg = DeclareLaunchArgument(
        'sensor_id', default_value='0'
    )

    #region Configurable

    approximate_sync = LaunchConfiguration('approximate_sync')
    avoid_point_cloud_padding = LaunchConfiguration('avoid_point_cloud_padding')
    use_color = LaunchConfiguration('use_color')
    use_system_default_qos = LaunchConfiguration('use_system_default_qos')
    launch_image_proc = LaunchConfiguration('launch_image_proc')
    left_namespace = LaunchConfiguration('left_namespace')
    right_namespace = LaunchConfiguration('right_namespace')
    container = LaunchConfiguration('container')
    stereo_algorithm = LaunchConfiguration('stereo_algorithm')
    prefilter_size = LaunchConfiguration('prefilter_size')
    prefilter_cap = LaunchConfiguration('prefilter_cap')
    correlation_window_size = LaunchConfiguration('correlation_window_size')
    min_disparity = LaunchConfiguration('min_disparity')
    disparity_range = LaunchConfiguration('disparity_range')
    texture_threshold = LaunchConfiguration('texture_threshold')
    speckle_size = LaunchConfiguration('speckle_size')
    speckle_range = LaunchConfiguration('speckle_range')
    disp12_max_diff = LaunchConfiguration('disp12_max_diff')
    uniqueness_ratio = LaunchConfiguration('uniqueness_ratio')
    P1_conf = LaunchConfiguration('P1')
    P2_conf = LaunchConfiguration('P2')
    full_dp = LaunchConfiguration('full_dp')

    approximate_sync_launch_arg = DeclareLaunchArgument(
        name='approximate_sync', default_value='False',
        description='Whether to use approximate synchronization of topics. Set to true if '
                    'the left and right cameras do not produce exactly synced timestamps.'
    )
    avoid_point_cloud_padding_launch_arg = DeclareLaunchArgument(
        name='avoid_point_cloud_padding', default_value='False',
        description='Avoid alignment padding in the generated point cloud.'
                    'This reduces bandwidth requirements, as the point cloud size is halved.'
                    'Using point clouds without alignment padding might degrade performance '
                    'for some algorithms.'
    )
    use_color_launch_arg = DeclareLaunchArgument(
        name='use_color', default_value='False',
        description='Generate point cloud with rgb data.'
    )
    use_system_default_qos_launch_arg = DeclareLaunchArgument(
        name='use_system_default_qos', default_value='False',
        description='Use the RMW QoS settings for the image and camera info subscriptions.'
    )
    launch_image_proc_launch_arg = DeclareLaunchArgument(
        name='launch_image_proc', default_value='True',
        description='Whether to launch debayer and rectify nodes from image_proc.'
    )
    left_namespace_launch_arg = DeclareLaunchArgument(
        name='left_namespace', default_value='left_image',
        description='Namespace for the left camera'
    )
    right_namespace_launch_arg = DeclareLaunchArgument(
        name='right_namespace', default_value='right_image',
        description='Namespace for the right camera'
    )
    container_launch_arg = DeclareLaunchArgument(
        name='container', default_value='',
        description=(
            'Name of an existing node container to load launched nodes into. '
            'If unset, a new container will be created.'
        )
    )
    # Stereo algorithm parameters
    stereo_algorithm_launch_arg = DeclareLaunchArgument(
        name='stereo_algorithm', default_value='0',
        description='Stereo algorithm: Block Matching (0) or Semi-Global Block Matching (1)'
    )
    prefilter_size_launch_arg = DeclareLaunchArgument(
        name='prefilter_size', default_value='9',
        description='Normalization window size in pixels (must be odd)'
    )
    prefilter_cap_launch_arg = DeclareLaunchArgument(
        name='prefilter_cap', default_value='31',
        description='Bound on normalized pixel values'
    )
    correlation_window_size_launch_arg = DeclareLaunchArgument(
        name='correlation_window_size', default_value='15',
        description='SAD correlation window width in pixels (must be odd)'
    )
    min_disparity_launch_arg = DeclareLaunchArgument(
        name='min_disparity', default_value='0',
        description='Disparity to begin search at in pixels'
    )
    disparity_range_launch_arg = DeclareLaunchArgument(
        name='disparity_range', default_value='64',
        description='Number of disparities to search in pixels (must be a multiple of 16)'
    )
    texture_threshold_launch_arg = DeclareLaunchArgument(
        name='texture_threshold', default_value='10',
        description='Filter out if SAD window response does not exceed texture threshold'
    )
    speckle_size_launch_arg = DeclareLaunchArgument(
        name='speckle_size', default_value='100',
        description='Reject regions smaller than this size in pixels'
    )
    speckle_range_launch_arg = DeclareLaunchArgument(
        name='speckle_range', default_value='4',
        description='Maximum allowed difference between detected disparities'
    )
    disp12_max_diff_launch_arg = DeclareLaunchArgument(
        name='disp12_max_diff', default_value='0',
        description='Maximum allowed difference in the left-right disparity check in pixels '
                    '(Semi-Global Block Matching only)'
    )
    uniqueness_ratio_launch_arg = DeclareLaunchArgument(
        name='uniqueness_ratio', default_value='15.0',
        description='Filter out if best match does not sufficiently exceed the next-best match'
    )
    P1_launch_arg = DeclareLaunchArgument(
        name='P1', default_value='200.0',
        description='The first parameter ccontrolling the disparity smoothness '
                    '(Semi-Global Block Matching only)'
    )
    P2_launch_arg = DeclareLaunchArgument(
        name='P2', default_value='400.0',
        description='The second parameter ccontrolling the disparity smoothness '
                    '(Semi-Global Block Matching only)'
    )
    full_dp_launch_arg = DeclareLaunchArgument(
        name='full_dp', default_value='False',
        description='Run the full variant of the algorithm (Semi-Global Block Matching only)'
    )
    #endregion

    image_proc_container = ComposableNodeContainer(
        name=['image_proc_', sensor_id],
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='left_rectify_mono_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image', [camera_name, '/', left_namespace, '/image_raw']),
                        ('camera_info', [camera_name, '/', left_namespace, '/camera_info']),
                        ('image_rect', [camera_name, '/', left_namespace, '/image_rect'])
                    ]),
                ComposableNode(
                    package='image_proc',
                    plugin='image_proc::RectifyNode',
                    name='right_rectify_mono_node',
                    # Remap subscribers and publishers
                    remappings=[
                        ('image', [camera_name, '/', right_namespace, '/image_raw']),
                        ('camera_info', [camera_name, '/', right_namespace, '/camera_info']),
                        ('image_rect', [camera_name, '/', right_namespace, '/image_rect'])
                    ])
        ],
        output=stdout,
        condition=IfCondition(launch_image_proc),
    )

    """Generate launch description with multiple components."""
    disparity_container = ComposableNodeContainer(
        name=['disparity_', sensor_id],
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::DisparityNode',
                    name=['disparity_node_', sensor_id],
                    parameters=[{
                        'approximate_sync': approximate_sync,
                        'use_system_default_qos': use_system_default_qos,
                        'stereo_algorithm': stereo_algorithm,
                        'prefilter_size': prefilter_size,
                        'prefilter_cap': prefilter_cap,
                        'correlation_window_size': correlation_window_size,
                        'min_disparity': min_disparity,
                        'disparity_range': disparity_range,
                        'texture_threshold': texture_threshold,
                        'speckle_size': speckle_size,
                        'speckle_range': speckle_range,
                        'disp12_max_diff': disp12_max_diff,
                        'uniqueness_ratio': uniqueness_ratio,
                        'P1': P1_conf,
                        'P2': P2_conf,
                        'full_dp': full_dp
                    }],
                    remappings=[
                        ('left/image_rect', [camera_name, '/', left_namespace, '/image_rect']),
                        ('left/camera_info', [camera_name, '/', left_namespace, '/camera_info']),
                        ('right/image_rect', [camera_name, '/', right_namespace, '/image_rect']),
                        ('right/camera_info', [camera_name, '/', right_namespace, '/camera_info'])
                    ]),
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    name=['pointcloud_node_', sensor_id],
                    parameters=[{
                        'approximate_sync': approximate_sync,
                        'avoid_point_cloud_padding': avoid_point_cloud_padding,
                        'use_color': use_color,
                        'use_system_default_qos': use_system_default_qos
                    }],
                    remappings=[
                        ('left/camera_info', [camera_name, '/', left_namespace, '/camera_info']),
                        ('right/camera_info', [camera_name, '/', right_namespace, '/camera_info']),
                        ('left/image_rect_color', [camera_name, '/', left_namespace, '/image_rect'])
                    ])
        ],
        output=stdout
    )

    opencv_node = Node(
        package='opencv_apps',
        executable='edge_detection',
        name=['opencv_', sensor_id],
        namespace=camera_name,
        remappings=[
            ('image', [left_namespace, '/image_rect'])
        ]
    )

    v2c_disp_masker_node = Node(
        package='calibration',
        executable='v2c_disp_masker',
        name=['v2c_disp_masker_', sensor_id],
        namespace=camera_name,
        remappings=[
            ('image', 'disparity'),
            ('mask', 'edge_detection/image'),
            ('output', 'edges_disparity')
        ],
        parameters=[{
            'edges_threshold': 128
        }]
    )

    """Generate launch description with multiple components."""
    pointclouder_edges_container = ComposableNodeContainer(
        name=['pointclouder_edges'],
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='stereo_image_proc',
                    plugin='stereo_image_proc::PointCloudNode',
                    parameters=[{
                        'approximate_sync': approximate_sync,
                        'avoid_point_cloud_padding': LaunchConfiguration('avoid_point_cloud_padding'),
                        'use_color': LaunchConfiguration('use_color'),
                        'use_system_default_qos': LaunchConfiguration('use_system_default_qos')
                    }],
                    remappings=[
                        ('left/camera_info', ['stereo_camera_info', '/left_info']),
                        ('right/camera_info', ['stereo_camera_info', '/right_info']),
                        ('left/image_rect_color', [left_namespace, '/image_rect_color'])
                    ])
        ],
        output=stdout
    )

    """Generate launch description with multiple components."""
    stereo_pcl_container = ComposableNodeContainer(
        name=['stereo_pcl_', sensor_id],
        namespace=camera_name,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name=['edges_pass_through_z'],
                    remappings=[
                        ('input', 'edge_points2'),
                        ('output', 'edge_z_filtered_cloud')
                    ],
                    parameters=[{
                        'filter_field_name': 'z',
                        'filter_limit_min': 0.8,
                        'filter_limit_max': 3,
                        'filter_limit_negative': False,
                        'max_queue_size': 1,
                        'keep_organized': False
                    }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name=['full_pass_through_z'],
                    remappings=[
                        ('input', 'points2'),
                        ('output', 'z_filtered_cloud')
                    ],
                    parameters=[{
                        
                        'filter_field_name': 'z',
                        'filter_limit_min': 0.8,
                        'filter_limit_max': 3,
                        'filter_limit_negative': False,
                        'max_queue_size': 1,
                        'keep_organized': False
                    }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::ExtractIndices',
                    name=['extract_plane_indices'],
                    remappings=[
                        ('input', 'z_filtered_cloud'),
                        ('indices', '/planar_segmentation/inliers')
                    ],
                    parameters=[{
                        'negative': False
                    }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::ExtractIndices',
                    name=['extract_circle_indices'],
                    remappings=[
                        ('input', '/laser2cam_calibration/z_filtered_cloud'),
                        ('indices', '/laser2cam_calibration/inliers')
                    ],
                    parameters=[{
                        'negative': False
                    }])
        ],
        output=stdout
    )

    v2c_plane_segmentation_node = Node(
        package='calibration',
        executable='v2c_plane_segmentation',
        name=['planar_segmentation_', sensor_id],
        namespace=camera_name,
        remappings=[
            ('input', 'z_filtered_cloud')
        ],
        parameters=[{
            'segmentation_type': 1,
            'axis': [0.0, 1.0, 0.0],
            'threshold': 0.1,
            'eps_angle': 0.55
        }]
    )

    stereo_pattern_node = Node(
        package='calibration',
        executable='stereo_pattern',
        name=['stereo_pattern_', sensor_id],
        namespace=camera_name,
        remappings=[
            ('cloud2', 'edge_z_filtered_cloud'),
            ('cam_plane_coeffs', '/planar_segmentation/model')
        ]
    )

    tf2_node = Node(
        package='tf2',
        executable='static_transform_publisher',
        name='stereo_ros_tf',
        arguments=['0', '0', '0', '-1.57079632679', '0', '-1.57079632679', 'rotated_', frame_name, frame_name, '100']
    )

    return LaunchDescription([
        stdout_launch_arg,
        camera_name_launch_arg,
        frame_name_launch_arg,
        sensor_id_launch_arg,
        #region Configureable
        approximate_sync_launch_arg,
        avoid_point_cloud_padding_launch_arg,
        use_color_launch_arg,
        use_system_default_qos_launch_arg,
        launch_image_proc_launch_arg,
        left_namespace_launch_arg,
        right_namespace_launch_arg,
        container_launch_arg,
        stereo_algorithm_launch_arg,
        prefilter_size_launch_arg,
        prefilter_cap_launch_arg,
        correlation_window_size_launch_arg,
        min_disparity_launch_arg,
        disparity_range_launch_arg,
        texture_threshold_launch_arg,
        speckle_size_launch_arg,
        speckle_range_launch_arg,
        disp12_max_diff_launch_arg,
        uniqueness_ratio_launch_arg,
        P1_launch_arg,
        P2_launch_arg,
        full_dp_launch_arg,
        #endregion
        image_proc_container,
        disparity_container,
        # opencv_node,
        # v2c_disp_masker_node,
        # pointclouder_edges_container,
        # stereo_pcl_container,
        # v2c_plane_segmentation_node,
        # stereo_pattern_node,
        # tf2_node
    ])