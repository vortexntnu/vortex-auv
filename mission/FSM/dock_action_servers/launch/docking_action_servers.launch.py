import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for docking fsm."""
    orca_config = os.path.join(
        get_package_share_directory(package_name='auv_setup'),
        'config',
        'robots',
        'orca.yaml',
    )

    yasmin_viewer_node = Node(
        package='yasmin_viewer',
        executable='yasmin_viewer_node',
        name='yasmin_viewer',
        namespace="orca",
        on_exit=launch.actions.LogInfo(msg="Yasmin_viewer exited"),
    )

    docking_launch = Node(
        package='docking_cpp',
        executable='docking',
        namespace="orca",
        parameters=[orca_config],
        on_exit=launch.actions.LogInfo(msg="Docking exited"),
    )

    state_publisher_node = Node(
        package='dock_action_servers',
        executable='fsm_state_node.py',
        name='fsm_state_node',
        namespace="orca",
        parameters=[orca_config],
        on_exit=launch.actions.LogInfo(msg="fsm_state_node exited"),
    )

    return LaunchDescription(
        initial_entities=[
            yasmin_viewer_node,
            docking_launch,
            state_publisher_node,
        ],
    )
