import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ipda_pose_filtering'),
        'config',
        'ipda_pose_filtering.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='ipda_pose_filtering',
                executable='ipda_pose_filtering_node',
                name='ipda_pose_filtering',
                output='screen',
                parameters=[config, {'use_sim_time': False}],
            ),
        ]
    )
