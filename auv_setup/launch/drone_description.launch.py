import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    xacro_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "description",
        f"{drone}.urdf.xacro",
    )

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=namespace,
            output="screen",
            remappings=[("joint_states", "servo_state")],
            parameters=[
                {
                    "robot_description": ParameterValue(
                        Command(["xacro", " ", xacro_path]),
                        value_type=str,
                    ),
                    "frame_prefix": f"{namespace}/",
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
