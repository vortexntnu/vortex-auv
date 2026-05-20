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
    debug_output = (
        LaunchConfiguration('debug_output').perform(context).lower() == 'true'
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    use_sim = LaunchConfiguration('use_sim').perform(context).lower() == 'true'

    param_file_name = "eskf_params.yaml" if use_sim else "eskf_params_real_world.yaml"
    eskf_params = os.path.join(
        get_package_share_directory("eskf"), "config", param_file_name
    )

    environment = (
        'stonefish_sim'
        if use_sim
        else LaunchConfiguration('environment').perform(context)
    )
    env_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "environments",
        f"{environment}.yaml",
    )

    eskf_node = Node(
        package="eskf",
        executable="eskf_node",
        name="eskf_node",
        namespace=namespace,
        parameters=[
            eskf_params,
            env_params,
            drone_params,
            {"frame_prefix": namespace},
            {"publish_debug": debug_output},
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
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='trondheim_freshwater',
        description=(
            'Environment config to load from auv_setup/config/environments/. '
            'If use_sim is true env config is set to stonefish_sim'
        ),
        choices=[
            'longbeach',
            'stonefish_sim',
            'trondheim_freshwater',
            'trondheim_saltwater',
        ],
    )
    debug_output_arg = DeclareLaunchArgument(
        'debug_output',
        default_value='true',
        description='If true, publish ESKF outputs on debug/private topics and disable TF publishing.',
    )
    return LaunchDescription(
        [sim_arg, environment_arg, debug_output_arg]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
