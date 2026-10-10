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
    ukf_params = os.path.join(
        get_package_share_directory('ukf'),
        'config',
        'ukf_params.yaml' if use_sim else 'ukf_params_real_world.yaml',
    )
    env_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'environments',
        f'{environment}.yaml',
    )
    overrides = {
        'frame_prefix': namespace,
        'publish_debug': debug_output,
        'use_sim_time': use_sim,
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
            package='ukf',
            executable='ukf_node',
            name='ukf_node',
            namespace=namespace,
            parameters=[
                ukf_params,
                env_params,
                drone_params,
                overrides,
            ],
            output='screen',
        ),
    ]

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument('imu_topic', default_value=''),
            DeclareLaunchArgument('dvl_topic', default_value=''),
            DeclareLaunchArgument('pressure_topic', default_value=''),
            DeclareLaunchArgument('magnetometer_topic', default_value=''),
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
                description='If true, publish UKF outputs on debug/private topics and disable TF publishing.',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
