from launch import LaunchDescription
from launch_ros.actions import Node
import os
#from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
#from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('velocity_controller')
    config_path = os.path.join(pkg_share, 'config', 'parameters.yaml')

    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='velocity_controller_node',
        description='Name of the velocity controller node'
    )

    node_name_arg2 = DeclareLaunchArgument(
        'node_name', default_value='test_VC_node',
        description='Name of the test VC node'
    )

    velocity_controller_name = LaunchConfiguration('node_name')
    test_VC_name = LaunchConfiguration('node_name')

    return LaunchDescription([
        node_name_arg,
        node_name_arg2,
        Node(package='velocity_controller',
             executable='velocity_controller_node',
             name=velocity_controller_name,
             output='screen',
             parameters=[config_path]),
        Node(package='velocity_controller',
             executable='test_VC_node',
             name=test_VC_name,
             output='screen',
             parameters=[config_path])
    ])