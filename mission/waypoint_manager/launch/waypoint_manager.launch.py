from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        namespace='orca',
        parameters=[],
        output='screen',
    )
    return LaunchDescription([waypoint_manager_node])
