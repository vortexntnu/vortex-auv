import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("pool_exploration"),
        "config",
        "pool_exploration.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package='pool_exploration',
                executable='pool_exploration_node',
                name='pool_exploration',
                output='screen',
                parameters=[
                    config,
                    {'use_sim_time': True},
                ],
            ),
        ]
    )
