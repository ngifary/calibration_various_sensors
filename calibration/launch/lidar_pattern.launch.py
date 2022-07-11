from multiprocessing import Value
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cloud_topic = LaunchConfiguration('cloud_topic')
    sensor_id = LaunchConfiguration('sensor_id')

    # args that can be set from the command line or a default will be used
    cloud_topic_launch_arg = DeclareLaunchArgument(
        'cloud_topic', default_value='sick_points'
    )
    sensor_id_launch_arg = DeclareLaunchArgument(
        "sensor_id", default_value='0'
    )

    # pcl_manager_component = ExecuteProcess(
    #     cmd=[[
    #         'ros2 component load ',
    #         turtlesim_ns,
    #         '/sim background_r ',
    #         '120'
    #     ]],
    #     shell=True
    # )

    lidar_pattern_node = Node(
        package='calibration',
        executable='lidar_pattern',
        name=['lidar_pattern_',sensor_id],
        remappings=[
                ('cloud1', [cloud_topic])
        ],
        parameters=[{
                'passthrough_radius_min': '1.0',
                'passthrough_radius_max': '6.0'
        }]
    )


    return LaunchDescription([
        cloud_topic,
        sensor_id,
        cloud_topic_launch_arg,
        sensor_id_launch_arg,
        lidar_pattern_node
    ])