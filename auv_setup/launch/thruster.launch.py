import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import ComposableNodeContainer
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ComposableNode


from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    solver_type = LaunchConfiguration("solver_type").perform(context)


    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    thruster_interface_config = os.path.join(
        get_package_share_directory("thruster_interface_auv"),
        "config",
        "thruster_interface_auv_config.yaml",
    )

    container = ComposableNodeContainer(
        name="thruster_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="thrust_allocator_auv",
                plugin="ThrustAllocator",
                name="thrust_allocator_auv_node",
                namespace=namespace,
                parameters=[
                drone_params,
                {"propulsion.solver_type": solver_type},
            ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="thruster_interface_auv",
                plugin="ThrusterInterfaceAUVNode",
                name="thruster_interface_auv_node",
                namespace=namespace,
                parameters=[thruster_interface_config, drone_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [container]


def generate_launch_description() -> LaunchDescription:
    """Generates a launch description for the ORCA AUV setup.

    This function sets up the environment variable for ROS console formatting
    and includes the launch descriptions for the thrust allocator and thruster
    interface components of the AUV.

    Returns:
        LaunchDescription: A launch description containing the environment variable
        setting and the included launch descriptions for the thrust allocator and
        thruster interface.

    """
    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT", value="[${severity}] [${time}] [${node}]: ${message}"
    )

    return LaunchDescription(
        [set_env_var]
        + declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "solver_type",
                default_value="qp",
                description="Thrust allocator solver type (available: pseudoinverse, qp)",
            ),
            OpaqueFunction(function=launch_setup)
            ]
    )
