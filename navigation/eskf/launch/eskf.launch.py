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


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    debug_output = (
        LaunchConfiguration('debug_output').perform(context).lower() == 'true'
    )
    use_sim = LaunchConfiguration('use_sim').perform(context).lower() == 'true'
    environment = LaunchConfiguration('environment').perform(context)

    drone_params = os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', f'{drone}.yaml'
    )
    eskf_params = os.path.join(
        get_package_share_directory('eskf'),
        'config',
        'eskf_params.yaml' if use_sim else 'eskf_params_real_world.yaml',
    )
    env_params = {}
    if not use_sim:
        env_file = os.path.join(
            get_package_share_directory('auv_setup'),
            'config',
            'environments',
            f'{environment}.yaml',
        )
        with open(env_file) as f:
            env_params = {
                group: {name: float(value) for name, value in values.items()}
                for group, values in yaml.safe_load(f).items()
            }
    overrides = {
        'frame_prefix': namespace,
        'publish_debug': debug_output,
        # stonefish does not publish /clock, it stamps messages with the wall clock,
        # so a node on sim time would never get a time and drop every message
        'use_sim_time': False,
    }
    defaults = {
        'imu_topic': f'/{namespace}/imu/data_raw' if use_sim else '',
        'pressure_topic': f'/{namespace}/pressure_sensor' if use_sim else '',
        'dvl_topic': '',
        'magnetometer_topic': '',
    }
    for argument, parameter in [
        ('imu_topic', 'topics.imu'),
        ('pressure_topic', 'topics.pressure_sensor'),
        ('dvl_topic', 'topics.dvl_twist'),
        ('magnetometer_topic', 'topics.magnetometer'),
    ]:
        topic = LaunchConfiguration(argument).perform(context) or defaults[argument]
        if topic:
            overrides[parameter] = topic
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
                overrides,
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

    if (
        debug_output
        and LaunchConfiguration('include_rpy').perform(context).lower() == 'true'
    ):
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

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('imu_topic', default_value=''),
            DeclareLaunchArgument('dvl_topic', default_value=''),
            DeclareLaunchArgument('magnetometer_topic', default_value=''),
            DeclareLaunchArgument('pressure_topic', default_value=''),
            DeclareLaunchArgument('include_rpy', default_value='false'),
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
                default_value='false',
                description='If true, launch the odom_transformer node alongside the ESKF.',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
