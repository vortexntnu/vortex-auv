from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    config = [
        path.join(
            get_package_share_directory(package_name="auv_setup"),
            "config",
            "robots",
            "orca.yaml",
        ),
        path.join(
            get_package_share_directory(package_name="thruster_interface_auv"),
            "config",
            "thruster_interface_auv_config.yaml",
        ),
    ]

    thruster_interface_auv_node = Node(
        package="thruster_interface_auv",
        executable="thruster_interface_auv_node",
        name="thruster_interface_auv_node",
        namespace="orca",
        output="screen",
        parameters=config,
    )

    return LaunchDescription([thruster_interface_auv_node])
