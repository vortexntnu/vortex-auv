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
    drone, namespace = resolve_drone_and_namespace(context)

    config_dir = os.path.join(get_package_share_directory("landmark_server"), "config")
    landmark_config = os.path.join(config_dir, "landmark_server_config.yaml")
    env = LaunchConfiguration("env").perform(context)
    if env not in ("sim", "pool"):
        raise RuntimeError(f"env must be sim or pool, not '{env}'")
    # Loaded after the common file: its values win.
    env_config = os.path.join(config_dir, f"{env}.yaml")

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="landmark_server",
            executable="landmark_server_node",
            name="landmark_server_node",
            namespace=namespace,
            parameters=[
                landmark_config,
                env_config,
                drone_params,
                {
                    "use_sim_time": False,
                },  # If testing with rosbags sim_time might be preferred if bag is looped
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "env",
                default_value="sim",
                description="sim (simulator values) or pool (values measured in the pool)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
