from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='guidance_los',
                executable='los_guidance_action_server',
                name='guidance_action_server',
                output='screen',
            )
        ]
    )
