from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to the YAML file
    config = path.join(
        get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
    )

    thruster_interface_auv_node = Node(
        package="thruster_interface_auv_cpp",
        executable="thruster_interface_auv_cpp_node",
        name="thruster_interface_auv_cpp_node",
        output="screen",
        parameters=[
            path.join(
                get_package_share_directory("auv_setup"),
                "config",
                "robots",
                "orca.yaml",
            ),
            path.join(
                get_package_share_directory("thruster_interface_auv_cpp"),
                "config",
                "coeffs.yaml",
            ),
        ],
    )

    return LaunchDescription([thruster_interface_auv_node])
