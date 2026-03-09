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

    thruster_config = os.path.join(
        get_package_share_directory("thruster_interface_auv"),
        "config",
        "thruster_interface_auv_config.yaml",
    )

    return [
        Node(
            package="thruster_interface_auv",
            executable="thruster_interface_auv_node",
            name="thruster_interface_auv_node",
            namespace=drone,
            output="screen",
            parameters=[drone_params, thruster_config],
        )
    ]


def generate_launch_description() -> LaunchDescription:
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
