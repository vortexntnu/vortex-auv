import launch.actions
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    """Set up the yasmin viewer node."""
    _drone, namespace = resolve_drone_and_namespace(context)

    return [
        Node(
            package="yasmin_viewer",
            executable="yasmin_viewer_node",
            name="yasmin_viewer",
            namespace=namespace,
            on_exit=launch.actions.LogInfo(msg="Yasmin_viewer exited"),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch the yasmin viewer."""
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
