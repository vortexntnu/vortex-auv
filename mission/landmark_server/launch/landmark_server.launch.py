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
    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )
    competition = LaunchConfiguration("competition").perform(context)
    course = LaunchConfiguration("course").perform(context)
    premap_file = LaunchConfiguration("premap_file").perform(context) or os.path.join(
        CONFIG_DIR, "course", course, "premap.yaml"
    )
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    landmarks_topic = LaunchConfiguration("landmarks_topic").perform(context)
    return [
        Node(
            package="landmark_server",
            executable="landmark_server_node",
            name="landmark_server_node",
            namespace=namespace,
            parameters=[
                LaunchConfiguration("config_file").perform(context),
                os.path.join(CONFIG_DIR, competition, "classes.yaml"),
                drone_params,
                {
                    "premap_file": premap_file,
                    "frame_prefix": namespace,
                    "debug": LaunchConfiguration("debug").perform(context) == "true",
                },
                *([{"topics.odom": odom_topic}] if odom_topic else []),
                *([{"topics.landmarks": landmarks_topic}] if landmarks_topic else []),
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
                "competition",
                default_value="robosub",
                description="config/<competition>/classes.yaml: what is on the course",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="pool",
                description="config/course/<course>/premap.yaml: where it is",
            ),
            DeclareLaunchArgument(
                "premap_file",
                default_value="",
                description="Another prior map. set_premap writes it: with "
                "--symlink-install that is the source file",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="",
                description="Odometry topic (default: the robot file's, odom)",
            ),
            DeclareLaunchArgument(
                "landmarks_topic",
                default_value="",
                description="Detections (default: the robot file's, landmarks)",
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="Also publish markers, raw detections and NIS",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
