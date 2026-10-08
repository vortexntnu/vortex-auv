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

CONFIG_DIR = os.path.join(get_package_share_directory("landmark_slam"), "config")


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )
    return [
        Node(
            package="landmark_slam",
            executable="landmark_slam_node",
            name="landmark_slam_node",
            namespace=namespace,
            parameters=[
                LaunchConfiguration("params_file").perform(context),
                # Simulator noise values on top (env:=sim).
                *(
                    [os.path.join(CONFIG_DIR, "params_sim.yaml")]
                    if LaunchConfiguration("env").perform(context) == "sim"
                    else []
                ),
                drone_params,
                {
                    "classes_file": LaunchConfiguration("classes_file").perform(
                        context
                    ),
                    "prior_map_file": LaunchConfiguration("prior_map_file").perform(
                        context
                    )
                    or os.path.join(
                        CONFIG_DIR,
                        "prior_map_sim.yaml"
                        if LaunchConfiguration("env").perform(context) == "sim"
                        else "prior_map.yaml",
                    ),
                    "frame_prefix": namespace,
                },
                # The simulator's odometry comes through sim_odom_relay_node.
                *(
                    [
                        {
                            "topics.odom": LaunchConfiguration("odom_topic").perform(
                                context
                            )
                        }
                    ]
                    if LaunchConfiguration("odom_topic").perform(context)
                    else []
                ),
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "params_file", default_value=os.path.join(CONFIG_DIR, "params.yaml")
            ),
            DeclareLaunchArgument(
                "classes_file",
                default_value=os.path.join(CONFIG_DIR, "landmark_classes.yaml"),
            ),
            DeclareLaunchArgument(
                "prior_map_file",
                default_value="",
                description="Default: prior_map.yaml, prior_map_sim.yaml with env:=sim",
            ),
            DeclareLaunchArgument(
                "env",
                default_value="pool",
                description="pool, or sim (params_sim.yaml on top, prior_map_sim.yaml)",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="Odometry topic (default: the robot file's, odom)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
