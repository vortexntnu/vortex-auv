from yaml import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction,  ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    stonefish_dir = get_package_share_directory('stonefish_sim')
    vortex_sim_interface_dir = get_package_share_directory('vortex_sim_interface')
    los_guidance_dir = get_package_share_directory('los_guidance')
    velocity_controller_dir = get_package_share_directory('velocity_controller_lqr')

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'scenario': 'tacc',
            'rendering': 'false',
        }.items(),
    )

    vortex_sim_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vortex_sim_interface_dir, 'launch', 'vortex_sim_interface.launch.py')
        )
    )

    los_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(los_guidance_dir, 'launch', 'los_guidance.launch.py')
        )
    )

    velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(velocity_controller_dir, 'launch', 'velocity_controller_lqr.launch.py')
        )
    )

    orca_sim = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stonefish_dir, 'launch', 'orca_sim.launch.py')
                )
            )
        ]
    )

    set_autonomy = TimerAction(
        period=12.0, 
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-lc",
                    "ros2 topic pub --once /orca/killswitch std_msgs/msg/Bool \"{data: false}\"; "
                    "ros2 topic pub --once /orca/operation_mode std_msgs/msg/String \"{data: 'autonomous mode'}\"; "
                ],
                output="screen",
            ),
        ],
    )

    square_test = TimerAction(
        period=14.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash", "-lc",
                    f"python3 {os.path.join(los_guidance_dir, 'scripts', 'square_test.py')}"
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([
        stonefish_sim,
        vortex_sim_interface,
        los_guidance_launch,
        velocity_controller_launch,
        orca_sim,
        set_autonomy,
        square_test,
    ])