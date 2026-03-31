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

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    use_sim = LaunchConfiguration('use_sim').perform(context).lower() == 'true'

    if use_sim:
        param_file_name = "eskf_params.yaml"
    else:
        param_file_name = "eskf_params_real_world.yaml"

    eskf_params = os.path.join(
        get_package_share_directory("eskf"), "config", param_file_name
    )

    eskf_node = Node(
        package="eskf",
        executable="eskf_node",
        name="eskf_node",
        namespace=namespace,
        parameters=[
            eskf_params,
            drone_params,
            {"frame_prefix": namespace},
        ],
        output="screen",
    )

    return [eskf_node]


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to "false" to load real-world hardware parameters.',
    )
    return LaunchDescription(
        [sim_arg]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
