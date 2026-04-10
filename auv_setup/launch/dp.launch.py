import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    filter_file_path = os.path.join(
        get_package_share_directory("reference_filter_dp"),
        "config",
        "reference_filter_params.yaml",
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller"),
        "config",
        f"adapt_params_{drone}.yaml",
    )

    container = ComposableNodeContainer(
        name="dp_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="reference_filter_dp",
                plugin="ReferenceFilterNode",
                name="reference_filter_node",
                namespace=namespace,
                parameters=[filter_file_path, drone_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="dp_adapt_backs_controller",
                plugin="DPAdaptBacksControllerNode",
                name="dp_adapt_backs_controller_node",
                namespace=namespace,
                parameters=[adapt_params, drone_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
