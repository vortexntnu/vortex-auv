from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

pid_params = path.join(
    get_package_share_directory("pid_controller_dp"), "config", "pid_params.yaml"
)

orca_params = path.join(
    get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
)


def generate_launch_description():
    pid_controller_node = Node(
        package="pid_controller_dp",
        executable="pid_controller_node",
        name="pid_controller_node",
        namespace="orca",
        parameters=[
            pid_params,
            orca_params,
        ],
        output="screen",
    )
    return LaunchDescription([pid_controller_node])
