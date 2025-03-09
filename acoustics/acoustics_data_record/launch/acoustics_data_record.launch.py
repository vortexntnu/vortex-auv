import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


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
                package='acoustics_data_record',
                namespace='acoustics_data_record',
                executable='acoustics_data_record_node.py',
                name='acoustics_data_record_node',
                output='screen',
                parameters=[yaml_file_path],
            ),
        ]
    )
