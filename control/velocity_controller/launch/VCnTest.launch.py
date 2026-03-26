import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    global_share = get_package_share_directory('auv_setup')
    config_path_global = os.path.join(global_share, 'config', 'robots', f'{drone}.yaml')
    common_launch_args = {
        "drone": drone,
        "namespace": namespace,
    }.items()

    stonefish_dir = get_package_share_directory('stonefish_sim')

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'rendering_quality': 'low', 'rendering': 'true'}.items(),
    )
    orca_sim = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stonefish_dir, 'launch', 'drone_sim.launch.py')
                )
            )
        ],
    )
    operation_mode_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("operation_mode_manager"),
                "launch",
                "operation_mode_manager.launch.py",
            )
        ),
        launch_arguments=common_launch_args,
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name_1',
        default_value='test_VC_node',
        description='Name of the test VC node',
    )
    test_VC_name = LaunchConfiguration('node_name_1')

    return [
        stonefish_sim,
        orca_sim,
        node_name_arg,
        # operation_mode_manager_launch,
        Node(
            package='velocity_controller',
            executable='test_VC_node',
            name=test_VC_name,
            namespace=namespace,
            output='screen',
            parameters=[config_path_global],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args() + [OpaqueFunction(function=launch_setup)]
    )
