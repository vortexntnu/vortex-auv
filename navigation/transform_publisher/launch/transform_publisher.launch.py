from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    transform_publisher_node = Node(
        package='transform_publisher',
        executable='transform_publisher_node',
        name='transform_publisher_node',
        output='screen',
    )
    return LaunchDescription([transform_publisher_node])
