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
    pose_config = os.path.join(
        get_package_share_directory(package_name='pose_action_server'),
        'config',
        'pose_action_server_params.yaml',
    )

    docking_launch = Node(
        package='docking',
        executable='docking',
        namespace="orca",
        parameters=[orca_config, pose_config],
        on_exit=launch.actions.LogInfo(msg="Docking exited"),
    )

    state_publisher_node = Node(
        package='publish_docking_state',
        executable='publish_docking_state',
        name='publish_docking_state',
        namespace="orca",
        parameters=[orca_config],
        on_exit=launch.actions.LogInfo(msg="Publish docking state node exited"),
    )

    return LaunchDescription(
        initial_entities=[
            docking_launch,
            state_publisher_node,
        ],
    )
