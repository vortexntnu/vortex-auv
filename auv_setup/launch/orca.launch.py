import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

orca_config = os.path.join(
    get_package_share_directory('auv_setup'),
    'config',
    'robots',
    'orca.yaml',
)

thruster_interface_config = os.path.join(
    get_package_share_directory("thruster_interface_auv"),
    "config",
    "thruster_interface_auv_config.yaml",
)

operation_mode_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("operation_mode_manager"),
                "launch",
                "operation_mode_manager.launch.py",
            )
        )
    )

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

    container = ComposableNodeContainer(
        name="thruster_container",
        namespace="orca",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="thrust_allocator_auv",
                plugin="ThrustAllocator",
                name="thrust_allocator_auv_node",
                namespace="orca",
                parameters=[orca_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="thruster_interface_auv",
                plugin="ThrusterInterfaceAUVNode",
                name="thruster_interface_auv_node",
                namespace="orca",
                parameters=[thruster_interface_config, orca_config],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",killswitch
        arguments=["--ros-args", "--log-level", "error"],
        
        
    )

    return LaunchDescription([set_env_var, container, operation_mode_manager_launch])
