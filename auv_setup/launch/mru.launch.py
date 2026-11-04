"""Kongsberg MRU — IMU driver.

Two modes:

  standalone=true  (default)
    Launches a regular Node.

  standalone=false
    Uses LoadComposableNodes to attach to an already-running container
    identified by `container_name`.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def _launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    standalone = LaunchConfiguration('standalone').perform(context).lower() == 'true'
    container_name = LaunchConfiguration('container_name').perform(context)
    host_ip = LaunchConfiguration('host_ip').perform(context)

    drone_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        f'{drone}.yaml',
    )

    with open(drone_params) as f:
        robot_topics = yaml.safe_load(f)['/**']['ros__parameters']['topics']

    parameters = [
        {
            "imu_pub_topic": robot_topics['imu'],
            'frame_id': f'/{namespace}/imu_link',
            'connection_params.remote_ip': '10.0.0.20',
            'connection_params.data_remote_port': 7550,
            'connection_params.data_local_port': 7551,
            'connection_params.control_local_port': 7552,
            'mru_settings.channel': 'UDP1',
            'mru_settings.port': 7551,
            'mru_settings.ip_addr': host_ip,
            'mru_settings.format': 'MRUBIN',
            'mru_settings.interval': 5,
            'mru_settings.token': 21,
        }
    ]

    if standalone:
        return [
            Node(
                package='mru_ros_interface',
                executable='mru_ros_interface_node',
                name='mru_ros_interface_node',
                namespace=namespace,
                parameters=parameters,
                output='screen',
            )
        ]
    else:
        return [
            LoadComposableNodes(
                target_container=container_name,
                composable_node_descriptions=[
                    ComposableNode(
                        package='mru_ros_interface',
                        plugin='MruRosInterface',
                        name='mru_ros_interface_node',
                        namespace=namespace,
                        parameters=parameters,
                        extra_arguments=[{'use_intra_process_comms': True}],
                    )
                ],
            )
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'standalone',
                default_value='true',
                description=(
                    'true = launch as a regular node; '
                    'false = attach to an existing container named by container_name'
                ),
            ),
            DeclareLaunchArgument(
                'container_name',
                default_value='',
                description='Container to attach to when standalone=false',
            ),
            DeclareLaunchArgument(
                'host_ip',
                default_value='10.0.0.67',
                description='IP address of this host machine, sent to the MRU so it knows where to stream data.',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=_launch_setup)]
    )
