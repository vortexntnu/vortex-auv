from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='thruster_interface_auv',
            executable='thruster_interface_auv_node',
            name='thruster_interface_auv_node'
        )
    ])