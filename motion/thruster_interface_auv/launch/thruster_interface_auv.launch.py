from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path


def generate_launch_description():
    # TODO: read parameters from yaml file
    return LaunchDescription([
        Node(package='thruster_interface_auv',
             executable='thruster_interface_auv_node.py',
             name='thruster_interface_auv_node',
             output='screen',
             emulate_tty=True)
    ])
