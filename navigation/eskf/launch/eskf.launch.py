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
    act_as_odom_source = (
        LaunchConfiguration('act_as_odom_source').perform(context).lower() == 'true'
    )
    use_sim = LaunchConfiguration('use_sim').perform(context).lower() == 'true'
    environment = (
        'stonefish_sim'
        if use_sim
        else LaunchConfiguration('environment').perform(context)
    )

    drone_params = os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', f'{drone}.yaml'
    )
    eskf_params = os.path.join(
        get_package_share_directory('eskf'),
        'config',
        'eskf_params.yaml' if use_sim else 'eskf_params_real_world.yaml',
    )
    env_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'environments',
        f'{environment}.yaml',
    )

    params = [eskf_params, env_params, drone_params, {"frame_prefix": namespace}]
    remappings = []
    if not act_as_odom_source:
        params.append({"publish_tf": False})
        remappings = [
            ("odom", "eskf/odom"),
            ("pose", "eskf/pose"),
            ("twist", "eskf/twist"),
        ]

    return [
        Node(
            package="eskf",
            executable="eskf_node",
            name="eskf_node",
            namespace=namespace,
            parameters=params,
            remappings=remappings,
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'act_as_odom_source',
                default_value='true',
                description='If true, publish on standard topics and enable TF. If false, remap to eskf/* topics and disable TF.',
            ),
            DeclareLaunchArgument(
                'use_sim',
                default_value='false',
                description='Set to "false" to load real-world hardware parameters.',
            ),
            DeclareLaunchArgument(
                'environment',
                default_value='trondheim_freshwater',
                description='Environment config to load from auv_setup/config/environments/.',
                choices=[
                    'longbeach',
                    'stonefish_sim',
                    'trondheim_freshwater',
                    'trondheim_saltwater',
                ],
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
