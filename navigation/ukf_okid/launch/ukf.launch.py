
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ukf_node = Node(
        package="ukf_python",
        executable="ukf_ros.py",
        name="ukf_node",
    )

    return LaunchDescription([ukf_node])
