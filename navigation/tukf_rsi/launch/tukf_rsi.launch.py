from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

tukf_rsi_params = path.join(
    get_package_share_directory("tukf_rsi"), "config", "tukf_rsi_params.yaml"
)


def generate_launch_description():
    tukf_rsi_node = Node(
        package="tukf_rsi",
        executable="tukf_rsi_node",
        name="tukf_rsi_node",
        parameters=[
            tukf_rsi_params,
        ],
        output="screen",
    )
    return LaunchDescription([tukf_rsi_node])
