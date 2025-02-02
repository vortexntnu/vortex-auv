import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ekf_pose_filtering_node = Node(
            package='ekf_pose_filtering',
            executable='ekf_pose_filtering_node',
            name='ekf_pose_filtering_node',
            parameters=[os.path.join(get_package_share_directory('ekf_pose_filtering'),'config','ekf_pose_filtering_params.yaml')],
            output='screen',
        )
    return LaunchDescription([
        ekf_pose_filtering_node
    ])