from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    sensor1_type = LaunchConfiguration('sensor1_type')
    sensor2_type = LaunchConfiguration('sensor2_type')
    sensor1_id = LaunchConfiguration('sensor1_id')
    sensor2_id = LaunchConfiguration('sensor2_id')

    # args that can be set from the command line or a default will be used
    sensor1_type_launch_arg = DeclareLaunchArgument(
        'sensor1_type', default_value='lidar'
    )
    sensor2_type_launch_arg = DeclareLaunchArgument(
        'sensor2_type', default_value='mono'
    )
    sensor1_id_launch_arg = DeclareLaunchArgument(
        "sensor1_id", default_value='0'
    )
    sensor2_id_launch_arg = DeclareLaunchArgument(
        "sensor2_id", default_value='0'
    )
    
    is_sensor1_cam = PythonExpression([
                    '"',
                    sensor1_type,
                    '" == "mono"',
                    ' or ',
                    '"',
                    sensor1_type,
                    '" == "stereo"'])
    is_sensor2_cam = PythonExpression([
                    '"',
                    sensor2_type,
                    '" == "mono"',
                    ' or ',
                    '"',
                    sensor2_type,
                    '" == "stereo"'])

    laser2cam_calibration_node = Node(
        package='calibration',
        executable='laser2cam_calibration',
        name='laser2cam_calibration',
        remappings=[
                ('cloud1', ['/', sensor1_type, '_pattern_',
                 sensor1_id, '/centers_cloud']),
                ('cloud2', ['/', sensor2_type, '_pattern_',
                 sensor2_id, '/centers_cloud'])
        ],
        parameters=[{
                'is_sensor1_cam': is_sensor1_cam,
                'is_sensor2_cam': is_sensor2_cam
        }]
    )


    return LaunchDescription([
        sensor1_type_launch_arg,
        sensor2_type_launch_arg,
        sensor1_id_launch_arg,
        sensor2_id_launch_arg,
        laser2cam_calibration_node
    ])
