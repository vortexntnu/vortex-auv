"""Full state estimation bringup.

Runs the ESKF, Nortek Nucleus, and MRU all inside a single shared
ComposableNodeContainer for intra-process comms.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer

from auv_setup.launch_arg_common import declare_drone_and_namespace_args


def generate_launch_description():
    auv_setup_dir = get_package_share_directory('auv_setup')
    eskf_dir = get_package_share_directory('eskf')

    # Mirrors resolve_drone_and_namespace: falls back to drone name if namespace is empty.
    namespace = PythonExpression([
        '"', LaunchConfiguration('namespace'), '" or "', LaunchConfiguration('drone'), '"'
    ])

    # /{namespace}/{container_name}
    fqn_container = PythonExpression([
        '"/" + ("', LaunchConfiguration('namespace'), '" or "', LaunchConfiguration('drone'),
        '") + "/" + "', LaunchConfiguration('container_name'), '"'
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'environment',
                default_value='trondheim_freshwater',
                description='Environment config to load from auv_setup/config/environments/.',
                choices=['longbeach', 'stonefish_sim', 'trondheim_freshwater', 'trondheim_saltwater'],
            ),
            DeclareLaunchArgument(
                'container_name',
                default_value='eskf_container',
                description='Name of the shared component container.',
            ),
            DeclareLaunchArgument(
                'host_ip',
                default_value='10.0.0.67',
                description='IP address of this host machine, sent to the MRU so it knows where to stream data.',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [
            ComposableNodeContainer(
                name=LaunchConfiguration('container_name'),
                namespace=namespace,
                package='rclcpp_components',
                executable='component_container_mt',
                composable_node_descriptions=[],
                output='screen',
                arguments=['--ros-args', '--log-level', 'error'],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(auv_setup_dir, 'launch', 'drone_description.launch.py')
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'namespace': namespace,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(eskf_dir, 'launch', 'eskf.launch.py')
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'namespace': namespace,
                    'standalone': 'false',
                    'container_name': fqn_container,
                    'environment': LaunchConfiguration('environment'),
                    'act_as_odom_source': 'true',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(auv_setup_dir, 'launch', 'nucleus.launch.py')
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'namespace': namespace,
                    'standalone': 'false',
                    'container_name': fqn_container,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(auv_setup_dir, 'launch', 'mru.launch.py')
                ),
                launch_arguments={
                    'drone': LaunchConfiguration('drone'),
                    'namespace': namespace,
                    'standalone': 'false',
                    'container_name': fqn_container,
                    'host_ip': LaunchConfiguration('host_ip'),
                }.items(),
            ),
        ]
    )
