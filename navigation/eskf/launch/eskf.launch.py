from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

eskf_params = path.join(
    get_package_share_directory("eskf"), "config", "eskf_params.yaml"
)


def generate_launch_description():
    eskf_node = Node(
        package="eskf",
        executable="eskf_node",
        name="eskf_node",
        namespace="orca",
        parameters=[
            eskf_params,
        ],
        output="screen",
    )
    return LaunchDescription([eskf_node])
