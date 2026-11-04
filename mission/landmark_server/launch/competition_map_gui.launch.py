from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)
    return [
        Node(
            package="landmark_server",
            executable="competition_map_gui.py",
            name="competition_map_gui",
            namespace=namespace,
            parameters=[
                {
                    "frame_prefix": namespace,
                    "competition": LaunchConfiguration("competition").perform(context),
                }
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "competition",
                default_value="robosub",
                description="config/<competition>/classes.yaml has the tasks",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
