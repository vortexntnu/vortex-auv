import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the package share directory
    pkg_share = get_package_share_directory('guidance_los')

    # Build the config file path
    config_file = os.path.join(pkg_share, 'config', 'los_guidance_params.yaml')

    # Verify config file exists
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"Config file not found at: {config_file}")

    # Create the launch configuration variables
    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=config_file,
        description='/home/badawi/Desktop/auto-pilot/src/vortex-auv/guidance/guidance_los/config/los_guidance_params.yaml',
    )

    # Create the node
    guidance_node = Node(
        package='guidance_los',
        executable='los_guidance_action_server',
        name='guidance_action_server',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'config_path': LaunchConfiguration('config_path'), 'debug_mode': False}
        ],
        respawn=False,
    )

    # Return the launch description
    return LaunchDescription([config_path_arg, guidance_node])
