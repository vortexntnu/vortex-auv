import launch.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Set up the yasmin viewer node."""
    drone = LaunchConfiguration("drone").perform(context)

    return [
        Node(
            package="yasmin_viewer",
            executable="yasmin_viewer_node",
            name="yasmin_viewer",
            namespace=drone,
            on_exit=launch.actions.LogInfo(msg="Yasmin_viewer exited"),
        )
    ]


def generate_launch_description() -> LaunchDescription:
    """Launch file to launch the yasmin viewer."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="Drone name / namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
