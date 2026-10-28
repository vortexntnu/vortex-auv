import os

# from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

# from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    pkg_share = get_package_share_directory('velocity_controller')
    global_share = get_package_share_directory('auv_setup')
    config_path_local = os.path.join(pkg_share, 'config', f'{drone}_params.yaml')
    config_path_global = os.path.join(global_share, 'config', 'robots', f"{drone}.yaml")

    #return [
    #    Node(
    #       package='velocity_controller',
    #      executable='velocity_node',
    #      name="velocity_controller_node",
    #      namespace=namespace,
    #    parameters=[config_path_local, config_path_global],
    # )
    #]
    return [
    ComposableNodeContainer(
        name='velocity_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velocity_controller',
                plugin='Velocity_node',   # must match the class name registered
                name='velocity_controller_node',
                namespace=namespace,
                parameters=[config_path_local, config_path_global],
            ),
        ],
        output='screen',
    )
]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
