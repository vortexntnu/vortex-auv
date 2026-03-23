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
    global_share = get_package_share_directory('auv_setup')
    config_path_local = os.path.join(pkg_share, 'config', 'parameters.yaml')
    config_path_global = os.path.join(global_share,'config','robots','orca.yaml')

    node_name_arg = DeclareLaunchArgument(
        'node_name', default_value='velocity_controller_node',
        description='Name of the velocity controller node'
    )

    velocity_controller_name = LaunchConfiguration('node_name')


    return LaunchDescription([
        node_name_arg,
        Node(package='velocity_controller',
             executable='velocity_controller_node',
             name=velocity_controller_name,
             namespace='orca',
             output='screen',
             parameters=[config_path_local,config_path_global])
    ])