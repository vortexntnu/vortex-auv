import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    test_scenario = LaunchConfiguration("test_scenario").perform(context)

    stonefish_dir = get_package_share_directory("stonefish_sim")
    los_guidance_dir = get_package_share_directory("los_guidance")
    velocity_controller_dir = get_package_share_directory('velocity_controller')

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "scenario": "default",
            "rendering": "true",
        }.items(),
    )

    los_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(los_guidance_dir, "launch", "los_guidance.launch.py")
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
        }.items(),
    )

    velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                velocity_controller_dir, 'launch', 'velocity_controller.launch.py'
            )
        ),
            launch_arguments={
            "drone": drone,
            "namespace": namespace,
        }.items(),

    )

    orca_sim = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stonefish_dir, "launch", "drone_sim.launch.py")
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
                        f"ros2 service call /{namespace}/set_killswitch "
                        "vortex_msgs/srv/SetKillswitch "
                        "\"{killswitch_on: false}\" "
                        "&& "
                        f"ros2 service call /{namespace}/set_operation_mode "
                        "vortex_msgs/srv/SetOperationMode "
                        "\"{requested_operation_mode: {operation_mode: 1}}\""
                    ),
                ],
                output="screen",
            ),
        ],
    )

    run_test_scenario = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        f"python3 {os.path.join(los_guidance_dir, 'scripts', 'test_scenarios.py')} "
                        f"--ros-args -p drone:={drone} -p test_scenario:={test_scenario}"
                    ),
                ],
                output="screen",
            )
        ],
    )

    return [
        stonefish_sim,
        los_guidance_launch,
        velocity_controller_launch,
        orca_sim,
        set_autonomy,
        run_test_scenario,
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args(default_drone="orca")
        + [
            DeclareLaunchArgument(
                "test_scenario",
                default_value="square",
                description="Scenario to run: square, circle, test_pitch, opposite_point",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
