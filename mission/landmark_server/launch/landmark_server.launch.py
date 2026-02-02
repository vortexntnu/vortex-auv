from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    landmark_node = Node(
        package='landmark_server',
        executable='landmark_server_node',
        name='landmark_server_node',
        namespace='orca',
        parameters=[],
        output='screen',
    )
    return LaunchDescription([landmark_node])
