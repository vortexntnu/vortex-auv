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

    config = os.path.join(
        get_package_share_directory("pid_controller_dp_euler"),
        "config",
        "pid_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="pid_controller_dp_euler",
            executable="pid_controller_node",
            name="pid_controller_euler_node",
            namespace=namespace,
            parameters=[config, drone_params],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
