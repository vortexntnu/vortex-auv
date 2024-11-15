from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to launch all nodes necessary for docking fsm.
    """
    return LaunchDescription(
        initial_entities=[
            Node(package='dock_action_servers', executable='go_to_dock_server.py', name='go_to_dock_server'),
            Node(package='dock_action_servers', executable='dock_server.py', name='dock_server'),
            Node(package='dock_action_servers', executable='go_down_server.py', name='go_down_server'),
            Node(package='dock_action_servers', executable='find_dock_server.py', name='find_dock'),
            Node(package='dock_action_servers', executable='go_over_dock_server.py', name='go_over_dock_server'),
            Node(package='dock_action_servers', executable='return_home_server.py', name='return_home_server'),
            Node(package='yasmin_viewer', executable='yasmin_viewer_node', name='yasmin_viewer'),
            Node(package='docking_cpp', executable='docking', name='docking'),
        ]
    )
