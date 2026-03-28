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

odom_transformer_params = path.join(
    get_package_share_directory("odom_transformer"),
    "config",
    "odom_transformer_params.yaml",
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    node = Node(
        package="odom_transformer",
        executable="odom_transformer_node",
        name="odom_transformer_node",
        namespace=namespace,
        parameters=[
            odom_transformer_params,
            drone_params,
            {"frame_prefix": namespace},
        ],
        output="screen",
    )

    return [node]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
