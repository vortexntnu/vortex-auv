import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pose_filtering'),
        'config',
        'pose_filtering.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='pose_filtering',
                executable='pose_filtering_node',
                name='pose_filtering',
                output='screen',
                parameters=[
                    config,
                    {
                        'use_sim_time': False,
                    },  # If testing with rosbags sim_time might be preferred if bag is looped
                ],
            ),
        ]
    )
