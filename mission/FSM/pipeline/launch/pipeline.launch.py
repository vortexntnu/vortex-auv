import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pipeline_config = os.path.join(
        get_package_share_directory('pipeline'),
        'config',
        'pipeline_config.yaml',
    )

    pipeline_node = Node(
        package='pipeline',
        executable='pipeline',
        namespace='orca',
        parameters=[pipeline_config],
        output='screen',
    )

    return LaunchDescription([pipeline_node])
