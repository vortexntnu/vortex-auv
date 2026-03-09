import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    drone_name = LaunchConfiguration("drone").perform(context)

    xacro_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "description",
        f"{drone_name}.urdf.xacro",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro", " ", xacro_path]),
                    value_type=str,
                ),
                "frame_prefix": f"{drone_name}/",
            }
        ],
    )

    return [robot_state_publisher_node]


def generate_launch_description():
    drone_name_arg = DeclareLaunchArgument(
        "drone",
        default_value="orca",
        description="Name of the drone (used as frame prefix)",
    )

    return LaunchDescription(
        [
            drone_name_arg,
            OpaqueFunction(function=launch_setup),
        ]
    )
