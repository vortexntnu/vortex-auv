from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch file for the guidance system."""
    return LaunchDescription([
        Node(
            package='guidance_los',
            executable='guidance',
            name='guidance_los_node',
            output='screen'
        ),
        #Node(
        #    package='guidance_los',
        #    executable='odom_formatter',
        #    name='odom_formatter_node',
        #    output='screen'
        #)
    ])