import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace()
    local_pkg_config = os.path.join(
        get_package_share_directory('landmark_drift_correction'),
        'config',
        'landmark_drift_correction.yaml',
    )

    drone_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        f'{drone}.yaml',
    )

    return [
        Node(
            package='landmark_drift_correction',
            executable='landmark_drift_correction_node',
            name='landmark_drift_correction',
            namespace=namespace,
            output='screen',
            parameters=[
                local_pkg_config,
                drone_params,
                {
                    'use_sim_time': False,
                },
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
