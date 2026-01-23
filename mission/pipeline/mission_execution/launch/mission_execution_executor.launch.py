from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mission_execution'),
        'config',
        'mission_execution_executor.yaml'
    )

    return LaunchDescription([
        Node(
            package='mission_execution',
            executable='mission_execution_executor',
            name='mission_execution_executor',
            output='screen',
            parameters=[config_file]
        ),
        Node(
            package='waypoint_manager',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            namespace='orca',
            output='screen',
        )
    ])

