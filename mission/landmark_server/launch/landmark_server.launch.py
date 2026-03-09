import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone = LaunchConfiguration("drone").perform(context)

    landmark_config = os.path.join(
        get_package_share_directory("landmark_server"),
        "config",
        "landmark_server_config.yaml",
    )

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
            namespace=drone,
            parameters=[
                landmark_config,
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
        [
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="Drone name / namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
