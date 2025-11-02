from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stonefish_dir = get_package_share_directory('stonefish_sim')
    ref_filter_dir = get_package_share_directory('reference_filter_dp')
    dp_controller_dir = get_package_share_directory('dp_adapt_backs_controller')

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={'scenario': 'docking'}.items(),
    )

    reference_filter = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(ref_filter_dir, 'launch', 'reference_filter_dp.launch.py')
                )
            )
        ]
    )


    dp_controller = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(dp_controller_dir, 'launch', 'dp_adapt_backs_controller.launch.py')
                )
            )
        ]
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
        reference_filter,
        dp_controller,
        orca_sim,
    ])
