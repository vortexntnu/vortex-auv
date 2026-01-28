import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

orca_params = os.path.join(
    get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
)

operation_mode_params = os.path.join(
    get_package_share_directory("operation_mode_manager"),
    "config",
    "operation_mode_manager.yaml",
)


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the operation_mode_manager node.

    This function creates a ROS 2 launch description that includes the
    operation_mode_manager node.

    Returns:
        LaunchDescription: A ROS 2 launch description containing the
        operation_mode_manager node.

    """
    operation_mode_manager_node = Node(
        package='operation_mode_manager',
        executable='operation_mode_manager_cpp',
        name='operation_mode_manager',
        namespace='orca',
        output="screen",
        parameters=[orca_params, operation_mode_params],
    )

    return LaunchDescription([operation_mode_manager_node])
