import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    velocity_lqr_config = os.path.join(
        get_package_share_directory("velocity_controller_lqr"),
        "config",
        "param_velocity_controller_lqr.yaml",
    )

    los_config = os.path.join(
        get_package_share_directory("los_guidance"),
        "config",
        "guidance_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    los_node = Node(
        package="los_guidance",
        executable="los_guidance_node",
        name="los_guidance_node",
        namespace=namespace,
        parameters=[drone_params, los_config],
        output="screen",
    )

    lqr_node = Node(
        package="velocity_controller_lqr",
        executable="velocity_controller_lqr_node.py",
        name="velocity_controller_lqr_node",
        namespace=namespace,
        output="screen",
        parameters=[drone_params, velocity_lqr_config],
    )

    return [los_node, lqr_node]


def generate_launch_description():
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
