import os
from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)

eskf_params = path.join(
    get_package_share_directory("eskf"), "config", "eskf_params.yaml"
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )
    drone_env_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "environments",
        "trondheim_saltwater.yaml",
    )
    eskf_node = Node(
        package="eskf",
        executable="eskf_node",
        name="eskf_node",
        namespace=namespace,
        parameters=[eskf_params, drone_params, drone_env_params],
        output="screen",
    )

    return [eskf_node]


def generate_launch_description():
    # This function defines WHAT to do, but doesn't execute the logic yet
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
