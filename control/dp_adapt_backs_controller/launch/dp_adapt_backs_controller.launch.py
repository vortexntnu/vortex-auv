from os import path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

adapt_params = path.join(
    get_package_share_directory("dp_adapt_backs_controller"),
    "config",
    "adapt_params.yaml",
)
orca_params = path.join(
    get_package_share_directory("auv_setup"),
    "config",
    "robots",
    "orca.yaml",
)


def generate_launch_description():
    dp_adapt_backs_controller_node = LifecycleNode(
        package="dp_adapt_backs_controller",
        executable="dp_adapt_backs_controller_node",
        name="dp_adapt_backs_controller_node",
        namespace="",
        parameters=[
            adapt_params,
            orca_params,
        ],
        output="screen",
    )
    return LaunchDescription([dp_adapt_backs_controller_node])
