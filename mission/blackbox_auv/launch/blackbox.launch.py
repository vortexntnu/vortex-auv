import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the blackbox node.

    This function constructs the path to the YAML configuration file for the
    blackbox node and returns a LaunchDescription object that includes the
    blackbox node with the specified parameters.

    Returns:
        LaunchDescription: A launch description object containing the blackbox node.

    """
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        "orca.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="blackbox_auv",
                executable="blackbox_node.py",
                name="blackbox_node",
                namespace="orca",
                output="screen",
                parameters=[yaml_file_path],
            ),
        ]
    )
