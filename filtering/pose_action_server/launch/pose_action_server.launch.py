import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pose_action_server_node = Node(
        package='pose_action_server',
        executable='pose_action_server_node',
        name='pose_action_server_node',
        parameters=[
            os.path.join(
                get_package_share_directory('pose_action_server'),
                'config',
                'pose_action_server_params.yaml',
            )
        ],
        output='screen',
    )
    return LaunchDescription([pose_action_server_node])
