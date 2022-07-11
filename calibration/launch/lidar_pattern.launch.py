from multiprocessing import Value
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
    # args that can be set from the command line or a default will be used
    stdout_launch_arg = DeclareLaunchArgument(
        "stdout", default_value=TextSubstitution(text="screen")
    )
    camera_name_launch_arg = DeclareLaunchArgument(
        "camera_name", default_value=TextSubstitution(text="/mono_camera")
    )
    image_topic_launch_arg = DeclareLaunchArgument(
        "image_topic", default_value=TextSubstitution(text="image_color")
    )
    frame_name_launch_arg = DeclareLaunchArgument(
        "frame_name", default_value=TextSubstitution(text="mono_camera")
    )
    sensor_id_launch_arg = DeclareLaunchArgument(
        "sensor_id", default_value=TextSubstitution(text="0")
    )
    marker_size_launch_arg = DeclareLaunchArgument(
        "marker_size", default_value=TextSubstitution(text="0.2")
    )

    mono_qr_pattern_with_parameters = Node(
            package='calibration',
            executable='mono_qr_pattern',
            name='mono_pattern_'+sensor_id_launch_arg.choices(),
            parameters=[{
                "background_r": LaunchConfiguration('background_r'),
                "background_g": LaunchConfiguration('background_g'),
                "background_b": LaunchConfiguration('background_b'),
            }]
        )

    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])