import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    controller_type = LaunchConfiguration("controller_type").perform(context).lower()

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )
    filter_file_path = os.path.join(
        get_package_share_directory("reference_filter_dp"),
        "config",
        "reference_filter_params.yaml",
    )

    nodes = []

    if controller_type == "adaptive":
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
        nodes.append(container)

    elif controller_type == "pid":
        pid_params = os.path.join(
            get_package_share_directory("pid_controller_dp"),
            "config",
            "pid_params.yaml",
        )
        filter_quat_file_path = os.path.join(
            get_package_share_directory("reference_filter_dp_quat"),
            "config",
            "reference_filter_params.yaml",
        )
        container = ComposableNodeContainer(
            name="dp_container",
            namespace=namespace,
            package="rclcpp_components",
            executable="component_container_mt",
            composable_node_descriptions=[
                ComposableNode(
                    package="reference_filter_dp_quat",
                    plugin="ReferenceFilterNode",
                    name="reference_filter_node",
                    namespace=namespace,
                    parameters=[filter_quat_file_path, drone_params],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                ComposableNode(
                    package="pid_controller_dp",
                    plugin="PIDControllerNode",
                    name="pid_controller_node",
                    namespace=namespace,
                    parameters=[pid_params, drone_params],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        )
        nodes.append(container)
    else:
        raise ValueError(
            f"Unknown controller_type '{controller_type}'. "
            f"Expected 'adaptive' or 'pid'."
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "controller_type",
                default_value="adaptive",
                description="Controller to use: 'adaptive' or 'pid'",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
