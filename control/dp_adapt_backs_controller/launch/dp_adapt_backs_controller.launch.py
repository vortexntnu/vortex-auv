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

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller"),
        "config",
        "adapt_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="dp_adapt_backs_controller",
            executable="dp_adapt_backs_controller_node",
            name="dp_adapt_backs_controller_node",
            namespace=namespace,
            parameters=[adapt_params, drone_params],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
