import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the acoustics_data_record node.

    This function constructs the path to the YAML configuration file for the
    acoustics_interface package and returns a LaunchDescription object that
    includes a Node for the acoustics_data_record package.

    Returns:
        LaunchDescription: A launch description containing the node configuration
        for acoustics_data_record.

    """
    yaml_file = os.path.join(
        get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
    )
    return LaunchDescription(
        [
            Node(
                package="acoustics_data_record_auv",
                executable="acoustics_data_record_node.py",
                name="acoustics_data_record_auv",
                namespace="orca",
                output="screen",
                parameters=[yaml_file],
            ),
        ]
    )
