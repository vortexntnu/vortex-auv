from os import path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    thrust_allocator_auv_node = Node(
        package='thrust_allocator_auv',
        executable='thrust_allocator_auv_node',
        name='thrust_allocator_auv_node',
        parameters=[
            path.join(get_package_share_directory('auv_setup'), 'config',
                      'robots', 'orca.yaml')
        ],
        output='screen',
    )
    return LaunchDescription([thrust_allocator_auv_node])
