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
    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')
    camera_topic = LaunchConfiguration('camera_topic')
    frame_name = LaunchConfiguration('frame_name')
    sensor_id = LaunchConfiguration('sensor_id')

    # args that can be set from the command line or a default will be used
    stdout_launch_arg = DeclareLaunchArgument(
        'stdout', default_value='screen'
    )
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value='basler_camera'
    )
    image_topic_launch_arg = DeclareLaunchArgument(
        'image_topic', default_value='image_raw'
    )
    camera_topic_launch_arg = DeclareLaunchArgument(
        'camera_topic', default_value='camera_info'
    )
    frame_name_launch_arg = DeclareLaunchArgument(
        "frame_name", default_value='camera_link'
    )
    sensor_id_launch_arg = DeclareLaunchArgument(
        'sensor_id', default_value='0'
    )

    marker_size = 0.04

    mono_qr_pattern_node = Node(
        package='calibration',
        executable='mono_qr_pattern',
        name=['mono_qr_pattern_', sensor_id],
        namespace=camera_name,
        remappings=[
            ('centers_msg', ['mono_qr_pattern_',
             sensor_id, '/centers_msg'])
        ],
        parameters=[{
            'image_topic': image_topic,
            'cinfo_topic': camera_topic,
            'marker_size': marker_size,
            'delta_width_qr_center': 0.155,
            'delta_height_qr_center': 0.080,
            'delta_width_circles': 0.14,
            'delta_height_circles': 0.09
        }],
        output=stdout
    )

    tf2_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=['camera_ros_tf_', sensor_id],
        arguments=['0', '0', '0', '-1.57079632679', '0',
                   '-1.57079632679', ['rotated_', frame_name], frame_name]
    )

    return LaunchDescription([
        stdout_launch_arg,
        camera_name_launch_arg,
        image_topic_launch_arg,
        camera_topic_launch_arg,
        frame_name_launch_arg,
        sensor_id_launch_arg,
        mono_qr_pattern_node,
        tf2_node
    ])
