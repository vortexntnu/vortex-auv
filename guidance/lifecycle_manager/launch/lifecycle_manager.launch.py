import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode

config_file_path = os.path.join(
    get_package_share_directory('lifecycle_manager'),
    'config',
    'lifecycle_manager_params.yaml',
)


def generate_launch_description():
    lifecycle_manager_node = LifecycleNode(
        package='lifecycle_manager',
        executable='lifecycle_manager_node',
        name='lifecycle_manager_node',
        namespace='orca',
        parameters=[
            config_file_path,
        ],
        output='screen',
    )
    return LaunchDescription([lifecycle_manager_node])
