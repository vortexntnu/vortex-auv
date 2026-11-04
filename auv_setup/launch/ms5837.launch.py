"""MS5837 pressure/depth sensor driver."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def _launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    environment = LaunchConfiguration('environment').perform(context)

    env_params_path = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'environments',
        f'{environment}.yaml',
    )

    with open(env_params_path) as f:
        env = yaml.safe_load(f)['/**']['ros__parameters']

    fluid_density = env['water']['density']
    atmospheric_pressure = env['atmosphere']['pressure']
    gravity = env['gravity']['acceleration']

    return [
        Node(
            package='ms5837_driver',
            executable='ms5837_node',
            name='ms5837_driver_node',
            namespace=namespace,
            parameters=[
                {
                    'frame_id': f'{namespace}/pressure_sensor_link',
                    'fluid_density': fluid_density,
                    'atmospheric_pressure': atmospheric_pressure,
                    'gravity': gravity,
                    'publish_depth': True,
                    'publish_altitude': False,
                }
            ],
            output='screen',
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
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
        + [OpaqueFunction(function=_launch_setup)]
    )
