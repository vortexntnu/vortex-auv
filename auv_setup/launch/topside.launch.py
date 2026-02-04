import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


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

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick_driver",
        namespace="orca",
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
                "launch/joystick_interface_auv.launch.py",
            )
        )
    )

    return LaunchDescription(
        [
            set_env_var,
            joy_node,
            joystick_interface_launch,
        ]
    )
