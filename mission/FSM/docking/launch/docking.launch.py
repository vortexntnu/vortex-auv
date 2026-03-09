import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Launch all nodes necessary for docking fsm."""
    drone = LaunchConfiguration("drone").perform(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    pose_config = os.path.join(
        get_package_share_directory("pose_action_server"),
        "config",
        "pose_action_server_params.yaml",
    )

    docking_launch = Node(
        package="docking",
        executable="docking",
        namespace=drone,
        parameters=[drone_params, pose_config],
        on_exit=launch.actions.LogInfo(msg="Docking exited"),
        output="screen",
    )

    state_publisher_node = Node(
        package="publish_docking_state",
        executable="publish_docking_state",
        name="publish_docking_state",
        namespace=drone,
        parameters=[drone_params],
        on_exit=launch.actions.LogInfo(msg="Publish docking state node exited"),
        output="screen",
    )

    return [docking_launch, state_publisher_node]


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for docking fsm."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="Drone name / namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
