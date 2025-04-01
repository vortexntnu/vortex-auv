import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    flir_camera_arg = DeclareLaunchArgument(
        'use_flir_camera',
        default_value='true',
        description='Whether to include the FLIR camera in the robot description',
    )

    use_flir_camera = LaunchConfiguration('use_flir_camera')

    xacro_path = os.path.join(
        get_package_share_directory('auv_setup'), 'description', 'orca.urdf.xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': Command(
                    [
                        'xacro',
                        ' ',
                        str(xacro_path),
                        ' ',
                        'use_flir_camera:=',
                        use_flir_camera,
                    ]
                )
            }
        ],
    )

    return LaunchDescription(
        [
            flir_camera_arg,
            robot_state_publisher_node,
        ]
    )
