import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    joystick_interface_node = Node(
        package="joystick_interface_auv",
        executable="joystick_interface_auv.py",
        name="joystick_interface_auv",
        output="screen",
        parameters=[
            os.path.join(
                get_package_share_directory("joystick_interface_auv"),
                "config/param_joystick_interface_auv.yaml",
            )
        ],
    )

    return LaunchDescription([joystick_interface_node])
