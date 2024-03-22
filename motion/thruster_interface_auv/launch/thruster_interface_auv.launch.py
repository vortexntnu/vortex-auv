from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from os import path

def generate_launch_description():
    thruster_interface_auv_node = Node(
        package='thruster_interface_auv',
        executable='thruster_interface_auv_node.py',
        name='thruster_interface_auv_node',
        output='screen',
        emulate_tty=True,
        parameters=[path.join(get_package_share_directory('auv_setup'), 'config',
                    'robots', 'orca.yaml')]
    )
    return LaunchDescription([thruster_interface_auv_node])