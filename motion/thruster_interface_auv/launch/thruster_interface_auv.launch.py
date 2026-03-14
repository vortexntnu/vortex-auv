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

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    thruster_config = os.path.join(
        get_package_share_directory("thruster_interface_auv"),
        "config",
        "thruster_interface_auv_config.yaml",
    )

    return [
        Node(
            package="thruster_interface_auv",
            executable="thruster_interface_auv_node",
            name="thruster_interface_auv_node",
            namespace=namespace,
            output="screen",
            parameters=[drone_params, thruster_config],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
