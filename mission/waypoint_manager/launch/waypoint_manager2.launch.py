import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    orca_config = os.path.join(
        get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
    )

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager2',
        name='waypoint_manager_node',
        namespace='orca',
        parameters=[orca_config],
        output='screen',
    )

    return LaunchDescription([waypoint_manager_node])
