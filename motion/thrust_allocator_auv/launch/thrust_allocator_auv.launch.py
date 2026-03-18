import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    solver_type = LaunchConfiguration("solver_type").perform(context)

    config_file = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="thrust_allocator_auv",
            executable="thrust_allocator_auv_node",
            name="thrust_allocator_auv_node",
            namespace=namespace,
            parameters=[
                config_file,
                {"propulsion.solver_type": solver_type},
            ],
            output="screen",
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "solver_type",
                default_value="qp",
                description="Thrust allocator solver type (available: pseudoinverse, qp)",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
