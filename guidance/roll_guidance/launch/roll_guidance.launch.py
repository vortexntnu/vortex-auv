"""
@file roll_guidance.launch.py

@brief ROS 2 launch configuration for the ROLL guidance node.

Loads the ROLL guidance parameters and robot-specific
configuration, then starts the ROLL guidance executable
inside the configured ROS namespace.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    """
    @brief Configures the ROLL guidance node for launch.

    Resolves the selected AUV and namespace, retrieves the
    configuration files, and creates the ROS 2 node action.

    @param context Current ROS 2 launch context.
    @param args Additional positional arguments.
    @param kwargs Additional keyword arguments.
    @return List of ROS 2 launch actions.
    """
    drone, namespace = resolve_drone_and_namespace(context)

    roll_params = os.path.join(
        get_package_share_directory("roll_guidance"),
        "config",
        "roll_guidance_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="roll_guidance",
            executable="roll_guidance_node",
            name="roll_guidance_node",
            namespace=namespace,
            parameters=[
                roll_params,
                drone_params,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    """
    @brief Generates the launch description for ROLL guidance.

    Declares the AUV and namespace arguments and registers
    the function responsible for creating the ROLL guidance node.

    @return LaunchDescription containing the required launch actions.
    """
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )