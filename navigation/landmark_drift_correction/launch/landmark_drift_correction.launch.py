import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('landmark_drift_correction'),
        'config',
        'landmark_drift_correction.yaml',
    )

    return LaunchDescription(
        [
            Node(
                package='landmark_drift_correction',
                executable='landmark_drift_correction_node',
                name='landmark_drift_correction',
                output='screen',
                parameters=[
                    config,
                    {
                        'use_sim_time': True
                    },
                ],
            ),
        ]
    )
