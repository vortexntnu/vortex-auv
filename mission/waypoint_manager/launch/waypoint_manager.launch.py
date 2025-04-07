import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_manager_pkg_dir = get_package_share_directory('waypoint_manager')

    waypoint_manager_config = os.path.join(
        waypoint_manager_pkg_dir, 'config', 'orca.yaml'
    )

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        namespace='orca',
        parameters=[waypoint_manager_config],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true',
            ),
            waypoint_manager_node,
        ]
    )
