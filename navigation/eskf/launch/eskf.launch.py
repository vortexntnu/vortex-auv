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
    nodes = [
        Node(
            package='eskf',
            executable='eskf_node',
            name='eskf_node',
            namespace=namespace,
            parameters=[
                eskf_params,
                env_params,
                drone_params,
                {'frame_prefix': namespace},
                {'publish_debug': debug_output},
            ],
            output='screen',
        ),
    ]

    if (
        LaunchConfiguration('include_odom_transformer').perform(context).lower()
        == 'true'
    ):
        nodes.append(
            Node(
                package='odom_transformer',
                executable='odom_transformer_node',
                name='odom_transformer_node',
                namespace=namespace,
                parameters=[
                    drone_params,
                    {
                        'frame_prefix': namespace,
                        'sensor_frame': 'dvl_link',
                        'publish_tf': False,
                        'publish_pose': False,
                        'publish_twist': False,
                        'topics.input': 'nucleus/odom',
                        'topics.output': 'nucleus/odom_relative',
                        'topics.pose': 'pose',
                        'topics.twist': 'twist',
                    },
                ],
                output='screen',
            )
        )

    if debug_output:
        nodes.append(
            Node(
                package='vortex_utility_nodes',
                executable='rpy_publisher_node',
                name='rpy_publisher_node',
                namespace=namespace,
                parameters=[
                    {
                        'input_topics': ['nucleus/odom_relative', 'eskf/odom'],
                        'output_topics': ['nucleus/odom_relative/rpy', 'eskf/odom/rpy'],
                        'input_types': ['odometry', 'odometry'],
                    }
                ],
                output='screen',
            )
        )

    if (
        LaunchConfiguration('include_pressure_to_depth').perform(context).lower()
        == 'true'
    ):
        nodes.append(
            Node(
                package='vortex_utility_nodes',
                executable='pressure_to_depth_node',
                name='pressure_to_depth',
                namespace=namespace,
                parameters=[
                    env_params,
                    {
                        'atmospheric_pressure': 101500.0,
                        'transform_to_base_link': LaunchConfiguration(
                            'pressure_transform_to_base_link'
                        ).perform(context),
                    },
                ],
                output='screen',
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
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
            DeclareLaunchArgument(
                'debug_output',
                default_value='true',
                description='If true, publish ESKF outputs on debug/private topics and disable TF publishing.',
            ),
            DeclareLaunchArgument(
                'include_odom_transformer',
                default_value='true',
                description='If true, launch the odom_transformer node alongside the ESKF.',
            ),
            DeclareLaunchArgument(
                'include_pressure_to_depth',
                default_value='true',
                description='If true, launch the pressure_to_depth node alongside the ESKF.',
            ),
            DeclareLaunchArgument(
                'pressure_transform_to_base_link',
                default_value='true',
                description='If true, shift pressure depth from pressure_sensor_link to base_link frame via TF.',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
