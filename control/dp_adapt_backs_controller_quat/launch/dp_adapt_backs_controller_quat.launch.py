import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_config_type_arg,
    declare_drone_and_namespace_args,
    resolve_config_type,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    config_type = resolve_config_type(context)

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller_quat"),
        "config",
        f"adapt_params_{drone}_{config_type}.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="dp_adapt_backs_controller_quat",
            executable="dp_adapt_backs_controller_quat_node",
            name="dp_adapt_backs_controller_node",
            namespace=namespace,
            parameters=[adapt_params, drone_params],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [declare_config_type_arg(), OpaqueFunction(function=launch_setup)]
    )
