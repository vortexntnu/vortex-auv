import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to your xacro file
    xacro_file = os.path.join(
        get_package_share_directory('auv_setup'), 'description', 'orca.urdf.xacro'
    )

    # Run xacro to convert to URDF
    robot_description = subprocess.check_output(['xacro', xacro_file]).decode()

    return LaunchDescription(
        [
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'robot_description': robot_description}],
            )
        ]
    )
