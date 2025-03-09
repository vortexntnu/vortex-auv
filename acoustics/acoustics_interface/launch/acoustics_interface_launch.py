import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        'orca.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='acoustics_interface',
                namespace='acoustics_interface',
                executable='acoustics_interface_node.py',
                name='acoustics_interface_node',
                output='screen',
                parameters=[yaml_file_path],
            ),
        ]
    )
