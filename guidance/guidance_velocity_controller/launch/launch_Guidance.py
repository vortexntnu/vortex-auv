from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

adapt_params = path.join(
    get_package_share_directory("guidance_velocity_controller"),
    "config",
    "guidanceParamsa.yaml",
)
auv_params = path.join(
    get_package_share_directory("auv_setup"),
    "config",
    "robots",
    "orca.yaml",
)


def generate_launch_description():
    guidance_node = Node(
        package="guidance_velocity_controller",
        executable="guidance_node",
        name="guidance_node",
        namespace="orca",
        parameters=[
            auv_params,
            adapt_params,
        ],
        output="screen",
    )
    return LaunchDescription([guidance_node])