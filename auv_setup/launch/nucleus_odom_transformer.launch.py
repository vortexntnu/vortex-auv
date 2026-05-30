"""Launch the Nucleus driver and odom_transformer together.

Forwards Nucleus INS odometry through odom_transformer to produce the AUV's
odometry and sets up the transform tree.

When launch_nucleus=true (default), both the Nucleus driver and odom_transformer
run inside a shared ComposableNodeContainer for intra-process comms.

When launch_nucleus=false, only the odom_transformer is launched as a plain node,
assuming the Nucleus driver is already running elsewhere.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    launch_nucleus = LaunchConfiguration('launch_nucleus').perform(context).lower() == 'true'
    container_name = LaunchConfiguration('container_name').perform(context)

    auv_setup_dir = get_package_share_directory('auv_setup')
    drone_params = os.path.join(auv_setup_dir, 'config', 'robots', f'{drone}.yaml')

    drone_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(auv_setup_dir, 'launch', 'drone_description.launch.py')
        ),
        launch_arguments={'drone': drone, 'namespace': namespace}.items(),
    )

    odom_transformer_params = [
        {
            'sensor_frame': 'dvl_link',
            'publish_tf': True,
            'publish_pose': True,
            'publish_twist': True,
            'topics.input': f'/{namespace}/nucleus/odom',
            'topics.output': f'/{namespace}/odom',
            'topics.pose': f'/{namespace}/pose',
            'topics.twist': f'/{namespace}/twist',
        },
        drone_params,
        {'frame_prefix': namespace},
    ]

    if launch_nucleus:
        nucleus_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(auv_setup_dir, 'launch', 'nucleus.launch.py')
            ),
            launch_arguments={
                'drone': drone,
                'namespace': namespace,
                'standalone': 'false',
                'container_name': f'/{namespace}/{container_name}',
            }.items(),
        )

        container = ComposableNodeContainer(
            name=container_name,
            namespace=namespace,
            package='rclcpp_components',
            executable='component_container_mt',
            composable_node_descriptions=[
                ComposableNode(
                    package='odom_transformer',
                    plugin='OdomTransformer',
                    name='odom_transformer_node',
                    namespace=namespace,
                    parameters=odom_transformer_params,
                    extra_arguments=[{'use_intra_process_comms': True}],
                )
            ],
            output='screen',
        )

        return [drone_description_launch, container, nucleus_launch]

    else:
        return [
            drone_description_launch,
            Node(
                package='odom_transformer',
                executable='odom_transformer_node',
                name='odom_transformer_node',
                namespace=namespace,
                parameters=odom_transformer_params,
                output='screen',
            ),
        ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'launch_nucleus',
                default_value='true',
                description='Set to false if the Nucleus driver is already running.',
            ),
            DeclareLaunchArgument(
                'container_name',
                default_value='nucleus_odom_container',
                description='Name of the shared component container (used when launch_nucleus=true).',
            ),
        ]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
