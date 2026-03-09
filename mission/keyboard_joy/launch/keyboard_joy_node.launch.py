import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    drone = LaunchConfiguration("drone").perform(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="keyboard_joy",
            executable="keyboard_joy_node",
            name="keyboard_joy",
            namespace=drone,
            output="screen",
            parameters=[
                {"config": LaunchConfiguration("config").perform(context)},
                drone_params,
            ],
        )
    ]


def generate_launch_description():
    keyboard_config = os.path.join(
        get_package_share_directory("keyboard_joy"),
        "config",
        "key_mappings.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=keyboard_config,
                description="Path to key mappings YAML file",
            ),
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="Drone name / namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
