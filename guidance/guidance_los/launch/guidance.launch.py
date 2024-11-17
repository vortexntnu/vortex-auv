from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for the guidance system."""
    return LaunchDescription(
        [
            # Launch the guidance action server
            Node(
                package='guidance_los',
                executable='los_guidance_action_server',  # This matches the entry point name
                name='guidance_action_server',
                output='screen',
            ),
            # Keep odom_formatter if still needed
            # Node(
            #     package='guidance_los',
            #     executable='odom_formatter',
            #     name='odom_formatter_node',
            #     output='screen'
            # )
        ]
    )
