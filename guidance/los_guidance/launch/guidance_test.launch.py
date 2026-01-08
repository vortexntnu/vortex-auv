from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    stonefish_dir = get_package_share_directory('stonefish_sim')
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

    return LaunchDescription([
        stonefish_sim,
        los_guidance_launch,
        velocity_controller_launch,
        orca_sim,
    ])