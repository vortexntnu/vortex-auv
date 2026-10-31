import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)
    sim = LaunchConfiguration("env").perform(context) == "sim"
    routes = os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "routes_sim.yaml" if sim else "routes.yaml",
    )
    return [
        Node(
            package="landmark_server",
            executable="competition_map_gui.py",
            name="competition_map_gui",
            namespace=namespace,
            parameters=[{"frame_prefix": namespace, "routes_file": routes}],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "env",
                default_value="pool",
                description="pool, or sim (routes_sim.yaml)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
