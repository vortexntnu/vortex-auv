from os import path
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchContext
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='1',
        description='Number of drones to control'
    )

    set_env_var = SetEnvironmentVariable(
        name='ROSCONSOLE_FORMAT',
        value='[${severity}] [${time}] [${node}]: ${message}')
    
    set_warn_color = SetEnvironmentVariable(
        name='RCUTILS_COLORIZED_OUTPUT',
        value='1'
    )

    thrust_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('thrust_allocator_auv'),
                      'launch', 'thrust_allocator_auv.launch.py')))

    joystick_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('joystick_interface_auv'),
                      'launch', 'joystick_interface_auv.launch.py')))

    vortex_sim_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            path.join(get_package_share_directory('vortex_sim_interface'),
                      'launch', 'vortex_sim_interface.launch.py')))

    def include_joy_node(context: LaunchContext):
        num_drones = int(LaunchConfiguration('num_drones').perform(context))

        if num_drones != 2:
            return [Node(
                package='joy',
                executable='joy_node',
                name='joystick_driver',
                output='screen',
                parameters=[{
                    'deadzone': 0.15,
                    'autorepeat_rate': 100.0,
                }],
                remappings=[
                    ('/joy', '/joystick/joy'),
                ],
            )]
        
        elif num_drones == 2:
            return [Node(
                package='joy',
                executable='joy_node',
                name='joystick_driver1',
                output='screen',
                parameters=[{
                    'deadzone': 0.15,
                    'autorepeat_rate': 100.0,
                    'device_name': 'Xbox Series X Controller',
                }],
                remappings=[
                    ('/joy', '/joystick/joy'),
                ],
            )]

    return LaunchDescription([
        num_drones_arg,
        set_env_var,
        set_warn_color,
        thrust_allocator_launch,
        OpaqueFunction(function=include_joy_node),
        joystick_interface_launch,
        vortex_sim_interface_launch,
    ])
