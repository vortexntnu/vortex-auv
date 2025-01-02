import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('guidance_los')
    config_file = os.path.join(pkg_share, 'config', 'los_guidance_params.yaml')

    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found at: {config_file}")

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=config_file,
        description='Path to the LOS guidance config file',
    )

    guidance_node = Node(
        package='guidance_los',
        executable='los_guidance_action_server',
        name='guidance_action_server',
        output='screen',
        emulate_tty=True,
        parameters=[config_file],
        respawn=False,
    )

    return LaunchDescription([config_path_arg, guidance_node])
