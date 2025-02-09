import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    eskf_python_node = Node(
        package='eskf_python',
        executable='eskf_python_node.py',
        name='eskf_python_node',
        parameters=[
            os.path.join(
                get_package_share_directory('eskf_python'),
                'config',
                'eskf_python.yaml',
            ),
        ],
        output='screen',
    )
    return LaunchDescription([eskf_python_node])
