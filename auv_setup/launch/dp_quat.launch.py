import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from auv_setup.launch_arg_common import (
    declare_config_type_arg,
    declare_drone_and_namespace_args,
    resolve_config_type,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    config_type = resolve_config_type(context)

    filter_config = os.path.join(
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

    adapt_params = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller_quat"),
        "config",
        f"adapt_params_{drone}_{config_type}.yaml",
    )

    container = ComposableNodeContainer(
        name="dp_quat_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="reference_filter_dp_quat",
                plugin="ReferenceFilterNode",
                name="reference_filter_node",
                namespace=namespace,
                parameters=[filter_config, drone_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="dp_adapt_backs_controller_quat",
                plugin="DPAdaptBacksControllerNode",
                name="dp_adapt_backs_controller_node",
                namespace=namespace,
                parameters=[adapt_params, drone_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return [container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            declare_config_type_arg(),
            OpaqueFunction(function=launch_setup),
        ]
    )
