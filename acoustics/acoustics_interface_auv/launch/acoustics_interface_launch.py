import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the acoustics_interface node.

    This function constructs the path to the YAML configuration file for the
    acoustics_interface node and returns a LaunchDescription object that
    includes the node with the specified parameters.

    Returns:
        LaunchDescription: A launch description object that includes the
        acoustics_interface node with the specified parameters.

    """
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        "orca.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="acoustics_interface_auv",
                executable="acoustics_interface_node.py",
                name="acoustics_interface_auv_node",
                output="screen",
                parameters=[yaml_file_path],
            ),
        ]
    )
