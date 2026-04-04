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
    rpy_publish = LaunchConfiguration("rpy_publish").perform(context) == "true"

    config_file_path = os.path.join(
        get_package_share_directory("reference_filter_dp_quat"),
        "config",
        "reference_filter_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    extra_params = {}
    if rpy_publish:
        extra_params = {"publish_rpy_debug": True}

    return [
        Node(
            package="reference_filter_dp_quat",
            executable="reference_filter_dp_quat_node",
            name="reference_filter_node",
            namespace=namespace,
            parameters=[config_file_path, drone_params, extra_params],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "rpy_publish",
                default_value="false",
                description="When true, publish RPY ReferenceFilter on guidance/dp "
                "for compatibility with controllers expecting RPY messages.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
