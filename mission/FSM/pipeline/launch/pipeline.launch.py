import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for pipeline fsm."""
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

    pipeline_launch = Node(
        package='pipeline',
        executable='pipeline',
        namespace="orca",
        parameters=[orca_config, pose_config],
        on_exit=launch.actions.LogInfo(msg="Pipeline exited"),
        output='screen',
    )

    state_publisher_node = Node(
        package='publish_pipeline_state',
        executable='publish_pipeline_state',
        name='publish_pipeline_state',
        namespace="orca",
        parameters=[orca_config],
        on_exit=launch.actions.LogInfo(msg="Publish pipeline state node exited"),
        output='screen',
    )

    return LaunchDescription(
        initial_entities=[
            pipeline_launch,
            state_publisher_node,
        ],
    )
