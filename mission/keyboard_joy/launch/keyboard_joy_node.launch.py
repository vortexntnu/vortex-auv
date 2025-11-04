import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    keyboard_config = os.path.join(
        get_package_share_directory('keyboard_joy'), 'config', 'key_mappings.yaml'
    )

    orca_params = os.path.join(
        get_package_share_directory('auv_setup'), 'config', 'robots', 'orca.yaml'
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config',
                default_value=keyboard_config,
                description='Path to key mappings YAML file',
            ),
            Node(
                package='keyboard_joy',
                executable='keyboard_joy_node.py',
                name='keyboard_joy',
                namespace='orca',
                output='screen',
                parameters=[{'config': LaunchConfiguration('config')}, orca_params],
            ),
        ]
    )
