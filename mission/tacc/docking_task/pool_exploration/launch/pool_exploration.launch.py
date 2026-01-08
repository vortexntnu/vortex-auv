from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='pool_exploration',
                executable='pool_exploration_node',
                name='pool_exploration',
                output='screen',
                parameters=[{'use_sim_time': False}],
            ),
        ]
    )
