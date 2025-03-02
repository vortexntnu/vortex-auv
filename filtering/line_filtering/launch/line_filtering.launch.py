import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    line_filtering_node = Node(
        package='line_filtering',
        executable='line_filtering_node',
        name='line_filtering_node',
        parameters=[
            os.path.join(
                get_package_share_directory('line_filtering'),
                'config',
                'line_filtering_params.yaml',
            ),
            {"use_sim_time": True}  # Enable simulated time
        ],
        output='screen',
    )
    return LaunchDescription([line_filtering_node])
