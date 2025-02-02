import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for docking fsm."""
    config = os.path.join(
        get_package_share_directory('dock_action_servers'), 'config', 'params.yaml'
    )

    return LaunchDescription(
        initial_entities=[
            # Node(package='dock_action_servers', executable='go_to_dock_server.py', name='go_to_dock_server', on_exit=launch.actions.LogInfo(msg="Go_to_dock_server exited")),
            # Node(package='dock_action_servers', executable='dock_server.py', name='dock_server', on_exit=launch.actions.LogInfo(msg="Dock_server exited")),
            # Node(package='dock_action_servers', executable='go_down_server.py', name='go_down_server', on_exit=launch.actions.LogInfo(msg="Go_down_server exited")),
            # Node(package='dock_action_servers', executable='find_dock_server.py', name='find_dock', on_exit=launch.actions.LogInfo(msg="Find_dock_server exited")),
            # Node(package='dock_action_servers', executable='go_over_dock_server.py', name='go_over_dock_server', on_exit=launch.actions.LogInfo(msg="Go_over_dock_server exited")),
            # Node(package='dock_action_servers', executable='return_home_server.py', name='return_home_server', on_exit=launch.actions.LogInfo(msg="Return_home_server exited")),
            Node(
                package='yasmin_viewer',
                executable='yasmin_viewer_node',
                name='yasmin_viewer',
                on_exit=launch.actions.LogInfo(msg="Yasmin_viewer exited"),
            ),
            Node(
                package='docking_cpp',
                executable='docking',
                name='docking',
                parameters=[config],
                on_exit=launch.actions.LogInfo(msg="Docking exited"),
            ),
            Node(
                package='dock_action_servers',
                executable='fsm_state_node.py',
                name='fsm_state_node',
                on_exit=launch.actions.LogInfo(msg="fsm_state_node exited"),
            ),
        ]
    )
