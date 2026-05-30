"""Nortek Nucleus 1000 — DVL/INS/pressure driver.

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
            'frame_id': f'/{namespace}/dvl_link',
            'qos': 'best_effort',
            'connection_params.remote_ip': '10.0.0.42',
            'connection_params.data_remote_port': 9000,
            'connection_params.password': '',
            'enable_imu': False,
            'enable_ins_odom': True,
            'enable_dvl': True,
            'enable_pressure': True,
            'enable_altimeter': True,
            'enable_magnetometer': True,
            'enable_ins_twist': False,
            'enable_ins_position': False,
            'enable_ins_pose': False,
            "imu_data_raw_pub_topic": f"/{namespace}/nucleus/imu/data_raw",
            "imu_data_pub_topic": f"/{namespace}/nucleus/imu/data",
            "ins_pub_topic": f"/{namespace}/nucleus/odom",
            "dvl_pub_topic": robot_topics['dvl_twist'],
            "altimeter_pub_topic": robot_topics['dvl_altitude'],
            "pressure_pub_topic": f"/{namespace}/nucleus/pressure", # Not used atm
            "magnetometer_pub_topic": robot_topics['magnetometer'],
            "ins_twist_pub_topic": f"/{namespace}/nucleus/ins/twist",
            "ins_position_pub_topic": f"/{namespace}/nucleus/ins/position",
            "ins_pose_pub_topic": f"/{namespace}/nucleus/ins/pose",
            'imu_settings.freq': 125,
            'ahrs_settings.freq': 10,
            'ahrs_settings.mode': 0,
            'bottom_track_settings.mode': 2,
            'bottom_track_settings.velocity_range': 5,
            'bottom_track_settings.enable_watertrack': False,
            'fast_pressure_settings.enable': True,
            'fast_pressure_settings.sampling_rate': 16,
            'magnetometer_settings.freq': 75,
            'magnetometer_settings.mode': 0,
            'instrument_settings.rotxy': 0.0,
            'instrument_settings.rotyz': 0.0,
            'instrument_settings.rotxz': 0.0,
        }
    ]

    if standalone:
        return [
            Node(
                package='nortek_nucleus_ros_interface',
                executable='nortek_nucleus_ros_interface_node',
                name='nortek_nucleus_ros_interface_node',
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
                        package='nortek_nucleus_ros_interface',
                        plugin='NortekNucleusRosInterface',
                        name='nortek_nucleus_ros_interface_node',
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
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=_launch_setup)]
    )
