import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    flir_camera_arg = DeclareLaunchArgument(
        'include_flir',
        default_value='true',
        description='Whether to include the FLIR camera in the robot description',
    )

    gripper_arg = DeclareLaunchArgument(
        'include_gripper',
        default_value='false',
        description='Whether to include the gripper in the robot description',
    )


    include_flir = LaunchConfiguration('include_flir')

    include_gripper = LaunchConfiguration('include_gripper')

    xacro_path = os.path.join(
        get_package_share_directory('auv_setup'), 'description', 'drone.urdf.xacro'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': ParameterValue(
                    Command(
                        [
                            'xacro',
                            ' ',
                            str(xacro_path),
                            ' ',
                            'include_flir:=',
                            include_flir,
                            ' ',
                            'include_gripper:=',
                            include_gripper,
                            ' '
                        ]
                    ),
                    value_type=str,
                )
            }
        ],
    )

    return LaunchDescription(
        [
            flir_camera_arg,
            gripper_arg,
            robot_state_publisher_node,
        ]
    )
