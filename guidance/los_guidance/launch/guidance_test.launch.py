import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource


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
            os.path.join(
                vortex_sim_interface_dir, 'launch', 'vortex_sim_interface.launch.py'
            )
        )
    )

    operation_mode_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("operation_mode_manager"),
            "launch",
            "operation_mode_manager.launch.py",
        )
    )
)

    los_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(los_guidance_dir, 'launch', 'los_guidance.launch.py')
        )
    )

    velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                velocity_controller_dir, 'launch', 'velocity_controller_lqr.launch.py'
            )
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
        ],
    )

    set_autonomy = TimerAction(
        period=12.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        "ros2 service call /orca/set_killswitch "
                        "vortex_msgs/srv/SetKillswitch "
                        "\"{killswitch_on: false}\" "
                        "&& "
                        "ros2 service call /orca/set_operation_mode "
                        "vortex_msgs/srv/SetOperationMode "
                        "\"{requested_operation_mode: {operation_mode: 1}}\""
                    ),
                ],
                output="screen",
            ),
        ],
    )

    square_test = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    f"python3 {os.path.join(los_guidance_dir, 'scripts', 'square_test.py')}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            stonefish_sim,
            vortex_sim_interface,
            operation_mode_launch,
            los_guidance_launch,
            velocity_controller_launch,
            orca_sim,
            set_autonomy,
            square_test,
        ]
    )
