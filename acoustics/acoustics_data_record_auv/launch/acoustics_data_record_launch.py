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
    return LaunchDescription(
        [
            Node(
                package="acoustics_data_record_auv",
                executable="acoustics_data_record_node.py",
                name="acoustics_data_record_auv_node",
                output="screen",
                parameters=[],
            ),
        ]
    )
