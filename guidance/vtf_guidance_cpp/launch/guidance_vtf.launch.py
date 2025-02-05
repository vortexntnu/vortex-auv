from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

vtf_params = path.join(
    get_package_share_directory("guidance_vtf"), "config", "vtf_params.yaml"
)


def generate_launch_description():
    guidance_vtf_node = Node(
        package="guidance_vtf",
        executable="guidance_vtf_node",
        name="guidance_vtf_node",
        parameters=[
            vtf_params,
        ],
        output="screen",
    )
    return LaunchDescription([guidance_vtf_node])
