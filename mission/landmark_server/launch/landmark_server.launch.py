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

CONFIG_DIR = os.path.join(get_package_share_directory("landmark_server"), "config")


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    env = LaunchConfiguration("env").perform(context)
    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )
    premap_file = LaunchConfiguration("premap_file").perform(context) or os.path.join(
        CONFIG_DIR, "premap_sim.yaml" if env == "sim" else "premap.yaml"
    )
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    return [
        Node(
            package="landmark_server",
            executable="landmark_server_node",
            name="landmark_server_node",
            namespace=namespace,
            parameters=[
                LaunchConfiguration("config_file").perform(context),
                # The simulator's measured detection noise on top.
                *([os.path.join(CONFIG_DIR, "sim.yaml")] if env == "sim" else []),
                drone_params,
                {"premap_file": premap_file, "frame_prefix": namespace},
                *([{"topics.odom": odom_topic}] if odom_topic else []),
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "config_file",
                default_value=os.path.join(CONFIG_DIR, "landmark_server.yaml"),
            ),
            DeclareLaunchArgument(
                "env",
                default_value="pool",
                description="pool, or sim (sim.yaml on top, premap_sim.yaml)",
            ),
            DeclareLaunchArgument(
                "premap_file",
                default_value="",
                description="Prior map (default: config/premap.yaml, "
                "premap_sim.yaml with env:=sim). set_premap writes it: with "
                "--symlink-install that is the source file",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="Odometry topic (default: the robot file's, odom)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
