from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)
    return [
        Node(
            package="landmark_server",
            executable="competition_map_gui.py",
            name="competition_map_gui",
            namespace=namespace,
            parameters=[{"frame_prefix": namespace}],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
