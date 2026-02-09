from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the mission execution controller node with yasmin_viewer."""
    
    mission_controller_node = Node(
        package='pipeline_fsm',
        executable='mission_controller_node',
        name='mission_controller',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    
    yasmin_viewer_node = Node(
        package='yasmin_viewer',
        executable='yasmin_viewer_node',
        name='yasmin_viewer',
        output='screen',
    )
    
    return LaunchDescription([
        mission_controller_node,
        yasmin_viewer_node,
    ])
