import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    waypoint_manager_pkg_dir = get_package_share_directory('waypoint_manager')
    reference_filter_pkg_dir = get_package_share_directory('reference_filter_dp')

    reference_filter_config = os.path.join(
        reference_filter_pkg_dir, 'config', 'reference_filter_params.yaml'
    )

    waypoint_manager_config = os.path.join(
        waypoint_manager_pkg_dir, 'config', 'waypoint_manager_params.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    reference_filter_node = Node(
        package='reference_filter_dp',
        executable='reference_filter_dp_node',
        name='reference_filter_node',
        namespace='orca',
        parameters=[reference_filter_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    waypoint_manager_node = Node(
        package='waypoint_manager',
        executable='waypoint_manager_node',
        name='waypoint_manager_node',
        namespace='orca',
        parameters=[waypoint_manager_config, {'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='false',
                description='Use simulation (Gazebo) clock if true',
            ),
            reference_filter_node,
            waypoint_manager_node,
        ]
    )
