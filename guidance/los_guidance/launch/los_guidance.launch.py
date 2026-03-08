from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

los_params = path.join(
    get_package_share_directory("los_guidance"),
    "config",
    "guidance_params.yaml",
)
orca_params = path.join(
    get_package_share_directory("auv_setup"),
    "config",
    "robots",
    "orca.yaml",
)


def generate_launch_description():
    los_guidance_node = Node(
        package="los_guidance",
        executable="los_guidance_node",
        name="los_guidance_node",
        namespace="orca",
        parameters=[
            orca_params,
            {"los_config_file": los_params},
            {"time_step": 0.01},
        ],
        output="screen",
    )
    return LaunchDescription([los_guidance_node])
