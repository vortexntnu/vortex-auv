from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the YAML file
    yaml_file_path = os.path.join(
        get_package_share_directory("internal_status_auv"),
        "../../../../",  # Navigate to the workspace root
        "src/vortex-auv/auv_setup/config/robots/",  # Directory containing YAML files
        'orca.yaml'  # Configuration file for the orca robot
    )

    # Power Sense Module Node
    power_sense_module_node = Node(
        package='internal_status_auv',
        executable='power_sense_module_node.py',
        output='screen',
        parameters=[yaml_file_path],
    )

    # Pressure Sensor Node
    pressure_sensor_node = Node(
        package='internal_status_auv',
        executable='pressure_sensor_node.py',
        output='screen',
        parameters=[yaml_file_path],
    )

    # Temperature Sensor Node
    temperature_sensor_node = Node(
        package='internal_status_auv',
        executable='temperature_sensor_node.py',
        output='screen',
        parameters=[yaml_file_path],
    )

    # Launch Nodes
    return LaunchDescription([
        power_sense_module_node,
        pressure_sensor_node,
        temperature_sensor_node,
    ])
