import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for docking fsm."""
    config_dock = os.path.join(
        get_package_share_directory(package_name='dock_action_servers'),
        'config',
        'docking.yaml',
    )

    config_dock_state = os.path.join(
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
        parameters=[config_dock, config_dock_state],
        on_exit=launch.actions.LogInfo(msg="Docking exited"),
    )

    state_publisher_node = Node(
        package='dock_action_servers',
        executable='fsm_state_node.py',
        name='fsm_state_node',
        namespace="orca",
        parameters=[config_dock_state],
        on_exit=launch.actions.LogInfo(msg="fsm_state_node exited"),
    )

    reference_filter_dp_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('reference_filter_dp'),
                'launch',
                'reference_filter.launch.py',
            )
        )
    )

    dp_adapt_backs_controller_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('dp_adapt_backs_controller'),
                'launch',
                'dp_adapt_backs_controller.launch.py',
            )
        )
    )

    pose_action_server_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('pose_action_server'),
                'launch',
                'pose_action_server.launch.py',
            )
        )
    )

    return LaunchDescription(
        initial_entities=[
            yasmin_viewer_node,
            docking_launch,
            state_publisher_node,
            reference_filter_dp_launch,
            dp_adapt_backs_controller_launch,
            pose_action_server_launch,
        ],
    )
