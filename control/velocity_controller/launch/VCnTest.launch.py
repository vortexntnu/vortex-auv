import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('velocity_controller')
    config_path = os.path.join(pkg_share, 'config', 'parameters.yaml')

    stonefish_dir = get_package_share_directory('stonefish_sim')

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'rendering_quality': 'low', 'rendering': 'false'}.items(),
    )
    orca_sim = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stonefish_dir, 'launch', 'orca_sim.launch.py')
                )
            )
        ],
    )

    node_name_arg = DeclareLaunchArgument(
        'node_name',
        default_value='velocity_controller_node',
        description='Name of the velocity controller node',
    )

    node_name_arg2 = DeclareLaunchArgument(
        'node_name_1',
        default_value='test_VC_node',
        description='Name of the test VC node',
    )

    velocity_controller_name = LaunchConfiguration('node_name')
    test_VC_name = LaunchConfiguration('node_name_1')

    return LaunchDescription(
        [
            stonefish_sim,
            orca_sim,
            node_name_arg,
            node_name_arg2,
            Node(
                package='velocity_controller',
                executable='velocity_controller_node',
                name=velocity_controller_name,
                output='screen',
                parameters=[config_path],
                # arguments=['--ros-args','--log-level','debug']
            ),
            Node(
                package='velocity_controller',
                executable='test_VC_node',
                name=test_VC_name,
                output='screen',
                parameters=[config_path],
            ),
        ]
    )
