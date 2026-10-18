from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    _, namespace = resolve_drone_and_namespace(context)
    return [
        Node(
            package='ukf',
            executable='nees_node.py',
            name=f'{estimator}_nees',
            namespace=namespace,
            parameters=[
                {
                    'estimate_topic': f'{estimator}/odom',
                    'truth_topic': 'odom',
                    'output_prefix': estimator,
                    'use_sim_time': False,
                }
            ],
            output='screen',
        )
        for estimator in ('ukf',)
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
