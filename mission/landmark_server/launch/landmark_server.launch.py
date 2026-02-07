import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

landmark_config = os.path.join(
    get_package_share_directory('landmark_server'),
    'config',
    'landmark_server_config.yaml',
)

orca_config = os.path.join(
    get_package_share_directory('auv_setup'),
    'config',
    'robots',
    'orca.yaml',
)


def generate_launch_description():
    landmark_node = Node(
        package='landmark_server',
        executable='landmark_server_node',
        name='landmark_server_node',
        namespace='orca',
        parameters=[landmark_config, orca_config],
        output='screen',
    )
    return LaunchDescription([landmark_node])
