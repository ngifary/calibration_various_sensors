from multiprocessing import Value
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    stdout = LaunchConfiguration('stdout')
    cloud_topic = LaunchConfiguration('cloud_topic')
    sensor_id = LaunchConfiguration('sensor_id')

    # args that can be set from the command line or a default will be used
    stdout_launch_arg = DeclareLaunchArgument(
        'stdout', default_value='screen'
    )
    cloud_topic_launch_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='/sick_mrs6124/sick_points'
    )
    sensor_id_launch_arg = DeclareLaunchArgument(
        "sensor_id", default_value='0'
    )

    """Generate launch description with multiple components."""
    container = ComposableNodeContainer(
        name=['pcl_manager_', sensor_id],
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name=['pass_through_z_laser_', sensor_id],
                    remappings=[
                        ('input', [cloud_topic]),
                        ('output', ['/lidar_pattern_',
                         sensor_id, '/z_filtered'])
                    ],
                    parameters=[{
                        'filter_field_name': 'z',
                        'filter_limit_min': -2.0,
                        'filter_limit_max': 1.0,
                        'filter_limit_negative': False,
                        'max_queue_size': 1,
                    }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name=['pass_through_y_laser_', sensor_id],
                    remappings=[
                        ('input', ['/lidar_pattern_',
                         sensor_id, '/z_filtered']),
                        ('output', ['/lidar_pattern_',
                         sensor_id, '/zy_filtered'])
                    ],
                    parameters=[{
                        'filter_field_name': 'y',
                        'filter_limit_min': -0.25,
                        'filter_limit_max': 0.3,
                        'filter_limit_negative': False,
                        'max_queue_size': 1,
                    }]),
                ComposableNode(
                    package='pcl_ros',
                    plugin='pcl_ros::PassThrough',
                    name=['pass_through_x_laser_', sensor_id],
                    remappings=[
                        ('input', ['/lidar_pattern_',
                         sensor_id, '/zy_filtered']),
                        ('output', ['/lidar_pattern_',
                         sensor_id, '/zyx_filtered'])
                    ],
                    parameters=[{
                        'filter_field_name': 'x',
                        'filter_limit_min': 0.0,
                        'filter_limit_max': 7.0,
                        'filter_limit_negative': False,
                        'max_queue_size': 1,
                    }]),
        ],
        output=stdout,
    )

    lidar_pattern_node = Node(
        package='calibration',
        executable='lidar_pattern',
        name=['lidar_pattern_', sensor_id],
        remappings=[
            ('cloud1', ['/lidar_pattern_', sensor_id, '/zyx_filtered']),
            ('centers_msg', ['/lidar_pattern_', sensor_id, '/centers_msg'])
        ],
        parameters=[{
            'passthrough_radius_min': 1.0,
            'passthrough_radius_max': 4.0,
            'plane_threshold': 0.1,
            'angle_threshold': 0.55,
            'gap_threshold': 0.01,
            'line_thresholdl': 0.003,
            'circle_threshold': 0.005,
            'circle_radius': 0.03,
            'target_radius_tolerance': 0.002,
            'delta_width_circles': 0.14,
            'delta_height_circles': 0.09,
        }],
        output=stdout
    )

    return LaunchDescription([
        stdout_launch_arg,
        cloud_topic_launch_arg,
        sensor_id_launch_arg,
        container,
        lidar_pattern_node
    ])
