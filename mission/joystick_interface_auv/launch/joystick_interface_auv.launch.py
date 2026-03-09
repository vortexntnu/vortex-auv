import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Set up the joystick_interface_auv node with drone-specific config."""
    drone = LaunchConfiguration("drone").perform(context)

    joystick_params = os.path.join(
        get_package_share_directory("joystick_interface_auv"),
        "config",
        "param_joystick_interface_auv.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="joystick_interface_auv",
            executable="joystick_interface_auv_node.py",
            name="joystick_interface_auv",
            namespace=drone,
            output="screen",
            parameters=[joystick_params, drone_params],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the joystick_interface_auv node.

    This function creates a ROS 2 launch description that includes the
    joystick_interface_auv node. The node is configured to use the
    parameters specified in the 'param_joystick_interface_auv.yaml' file.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        joystick_interface_auv node.

    """
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
