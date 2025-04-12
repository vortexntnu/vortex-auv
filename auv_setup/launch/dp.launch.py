import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

filter_file_path = os.path.join(
    get_package_share_directory('reference_filter_dp'),
    'config',
    'reference_filter_params.yaml',
)

orca_config = os.path.join(
    get_package_share_directory('auv_setup'),
    'config',
    'robots',
    'orca.yaml',
)

adapt_params = os.path.join(
    get_package_share_directory("dp_adapt_backs_controller"),
    "config",
    "adapt_params.yaml",
)


def generate_launch_description():
    container = ComposableNodeContainer(
        name="dp_container",
        namespace="orca",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="reference_filter_dp",
                plugin="ReferenceFilterNode",
                name="reference_filter_node",
                namespace="orca",
                parameters=[filter_file_path, orca_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="dp_adapt_backs_controller",
                plugin="DPAdaptBacksControllerNode",
                name="dp_adapt_backs_controller_node",
                namespace="orca",
                parameters=[adapt_params, orca_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return LaunchDescription([container])
