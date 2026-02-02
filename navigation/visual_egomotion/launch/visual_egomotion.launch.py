from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

visual_egomotion_params = path.join(
    get_package_share_directory("visual_egomotion"),
    "config",
    "visual_egomotion.yaml",
)

def generate_launch_description():
    visual_egomotion_node = Node(
        package="visual_egomotion",
        executable="visual_egomotion_node",
        name="visual_egomotion",
        parameters=[visual_egomotion_params],
        output="screen",
    )

    base_to_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_cam",
        arguments=[
            "0.2", "0", "0",
            "0", "0", "0",
            "base_link", "Orca/camera_front",
        ],
        output="screen",
    )

    return LaunchDescription([
        visual_egomotion_node,
        base_to_cam_tf,
    ])