import os

import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    """Launch all nodes necessary for docking fsm."""
    drone, namespace = resolve_drone_and_namespace(context)

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
        namespace=namespace,
        parameters=[drone_params, pose_config],
        on_exit=launch.actions.LogInfo(msg="Docking exited"),
        output="screen",
    )

    state_publisher_node = Node(
        package="publish_docking_state",
        executable="publish_docking_state",
        name="publish_docking_state",
        namespace=namespace,
        parameters=[drone_params],
        on_exit=launch.actions.LogInfo(msg="Publish docking state node exited"),
        output="screen",
    )

    return [docking_launch, state_publisher_node]


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch all nodes necessary for docking fsm."""
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
