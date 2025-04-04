from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_file = path.join(
        get_package_share_directory('waypoint_manager'),
        'config',
        'waypoint_manager_params.yaml',
    )

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        namespace='orca',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([waypoint_manager_node])
