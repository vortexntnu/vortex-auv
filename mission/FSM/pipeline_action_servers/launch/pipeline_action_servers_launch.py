from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    Launch file to launch all nodes necessary for docking fsm.
    """
    return LaunchDescription(
        initial_entities=[
            Node(package='pipeline_action_servers', executable='go_to_pipeline_server', name='go_to_pipeline_server'),
            Node(package='pipeline_action_servers', executable='find_pipeline_start_server', name='find_pipeline_start_server'),
            Node(package='pipeline_action_servers', executable='return_home_server', name='return_home_server'),
            Node(package='yasmin_viewer', executable='yasmin_viewer_node', name='yasmin_viewer'),
            Node(package='pipeline_cpp', executable='pipeline_demo_cpp', name='pipeline_demo_cpp'),
        ]
    )
