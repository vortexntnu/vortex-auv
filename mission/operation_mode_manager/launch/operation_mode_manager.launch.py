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
    """Set up the operation_mode_manager node with drone-specific config."""
    drone, namespace = resolve_drone_and_namespace(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    operation_mode_params = os.path.join(
        get_package_share_directory("operation_mode_manager"),
        "config",
        "operation_mode_manager.yaml",
    )

    return [
        Node(
            package="operation_mode_manager",
            executable="operation_mode_manager_cpp",
            name="operation_mode_manager",
            namespace=namespace,
            output="screen",
            parameters=[drone_params, operation_mode_params],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the operation_mode_manager node.

    This function creates a ROS 2 launch description that includes the
    operation_mode_manager node.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        operation_mode_manager node.

    """
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
