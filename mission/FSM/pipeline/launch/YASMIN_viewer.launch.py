import launch.actions
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch the yasmin viewer."""
    yasmin_viewer_node = Node(
        package='yasmin_viewer',
        executable='yasmin_viewer_node',
        name='yasmin_viewer',
        namespace="orca",
        on_exit=launch.actions.LogInfo(msg="Yasmin_viewer exited"),
    )

    return LaunchDescription(
        initial_entities=[
            yasmin_viewer_node,
        ],
    )
