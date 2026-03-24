import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    guidance_config = os.path.join(
        get_package_share_directory("los_guidance"),
        "config",
        "guidance_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="los_guidance",
            executable="los_guidance_node",
            name="los_guidance_node",
            namespace=namespace,
            parameters=[
                drone_params,
                {
                    "los_config_file": guidance_config,
                    "time_step": 0.1,
                },
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
