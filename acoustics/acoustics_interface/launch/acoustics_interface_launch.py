from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("blackbox"),
        "../../../../",  # Go to the workspace
        "src/vortex-asv/asv_setup/config/robots/", # Go inside where yamal files are located at
        'freya.yaml'
    )

    return LaunchDescription([
        Node(
            package='acoustics_interface',
            namespace='acoustics_interface',
            executable='acoustics_interface_node.py',
            name='acoustics_interface_node',
            output='screen',
            parameters=[yaml_file_path],
        ),
    ])