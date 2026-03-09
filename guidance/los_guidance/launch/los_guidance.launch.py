import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone = LaunchConfiguration("drone").perform(context)

    adapt_params = os.path.join(
        get_package_share_directory("los_guidance"),
        "config",
        "guidance_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="los_guidance",
            executable="los_guidance_node",
            name="los_guidance_node",
            namespace=drone,
            parameters=[drone_params, adapt_params],
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
