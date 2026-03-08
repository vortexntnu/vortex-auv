import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    drone_name_arg = DeclareLaunchArgument(
        'drone',
        default_value='drone',
        description='Name of the drone (used as frame prefix)',
    )

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

    static_shoulder_arg = DeclareLaunchArgument(
        'static_shoulder',
        default_value='true',
        description='Whether the gripper shoulder joint is fixed (true) or revolute (false)',
    )

    drone_name = LaunchConfiguration('drone')

    include_flir = LaunchConfiguration('include_flir')

    include_gripper = LaunchConfiguration('include_gripper')

    static_shoulder = LaunchConfiguration('static_shoulder')

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
                            ' ',
                            'static_shoulder:=',
                            static_shoulder,
                            ' ',
                        ]
                    ),
                    value_type=str,
                ),
                'frame_prefix': [drone_name, '/'],
            }
        ],
    )

    return LaunchDescription(
        [
            drone_name_arg,
            flir_camera_arg,
            gripper_arg,
            static_shoulder_arg,
            robot_state_publisher_node,
        ]
    )
