import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    drone_params = os.path.join(
        get_package_share_directory('auv_setup'),
        'config',
        'robots',
        f'{drone}.yaml',
    )
    velocity_control_params = os.path.join(
        get_package_share_directory('velocity_controller'),
        'config',
        f'{drone}_params.yaml',
    )
    los_config = os.path.join(
        get_package_share_directory('los_guidance'),
        'config',
        'guidance_params.yaml',
    )
    container=ComposableNodeContainer(
        name='autopilot_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='velocity_controller',
                plugin='velocity_node',
                name='velocity_controller_node',
                namespace=namespace,
                parameters=[velocity_control_params,drone_params],
                extra_arguments=[{"use_intra_process_comms":True}],
            ),
            ComposableNode(
                package='los_guidance',
                plugin='vortex::guidance::los::LosGuidanceNode',
                name='los_guidance_node',
                namespace=namespace,
                parameters=[drone_params,{"los_config_file":los_config,"time_step":0.1},],
                extra_arguments=[{"use_intra_process_comms":True}],
            ),

        ],
        output='screen',
        arguments=['--ros-args','--log-level','error'],
    )
    return [container]
    


def generate_launch_description():
def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )
