import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sp_ukf_python_node = Node(
        package='sp_ukf_python',
        executable='sp_ukf_python_node.py',
        name='sp_ukf_python_node',
        parameters=[
            os.path.join(
                get_package_share_directory('sp_ukf_python'),
                'config',
                'sp_ukf_python.yaml',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([sp_ukf_python_node])
