import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    """Set up the topside nodes with drone-specific namespace."""
    drone, namespace = resolve_drone_and_namespace(context)

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver",
        namespace=namespace,
        output="screen",
        parameters=[
            {"deadzone": 0.15},
            {"autorepeat_rate": 100.0},
        ],
    )

    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("joystick_interface_auv"),
                "launch",
                "joystick_interface_auv.launch.py",
            )
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
        }.items(),
    )

    return [joy_node, joystick_interface_launch]


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the topside AUV setup.

    This function sets up the environment variable for ROS console formatting,
    initializes the joystick node with specific parameters and remappings, and
    includes the joystick interface launch description.

    Returns:
        LaunchDescription: The launch description containing the environment
        variable setting, joystick node, and joystick interface launch.

    """
    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT", value="[${severity}] [${time}] [${node}]: ${message}"
    )

    return LaunchDescription(
        [set_env_var]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
