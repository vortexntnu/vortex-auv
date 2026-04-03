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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    test_scenario = LaunchConfiguration("test_scenario").perform(context)
    use_keyboard_joy = LaunchConfiguration("use_keyboard_joy")

    stonefish_dir = get_package_share_directory("stonefish_sim")
    los_guidance_dir = get_package_share_directory("los_guidance")
    keyboard_joy_dir = get_package_share_directory("keyboard_joy")
    velocity_controller_dir = get_package_share_directory("velocity_controller_lqr")
    utility_dir = get_package_share_directory("vortex_utility_nodes")

    stonefish_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "drone": drone,
            "scenario": "nautilus_no_gpu",
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

    keyboard_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(keyboard_joy_dir, "launch", "keyboard_joy_node.launch.py")
        ),
        condition=IfCondition(use_keyboard_joy),
    )

    velocity_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                velocity_controller_dir, "launch", "velocity_controller_lqr.launch.py"
            )
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
        }.items(),
    )

    drone_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stonefish_dir, "launch", "drone_sim.launch.py")
        ),
        launch_arguments={
            "drone": drone,
        }.items(),
    )

    utility_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(utility_dir, "launch", "message_publisher.launch.py")
        )
    )

    run_test_scenario = TimerAction(
        period=20.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    os.path.join(los_guidance_dir, "scripts", "test_scenarios.py"),
                    "--ros-args",
                    "-p",
                    f"drone:={drone}",
                    "-p",
                    f"test_scenario:={test_scenario}",
                ],
                output="screen",
            )
        ],
    )

    return [
        stonefish_sim,
        keyboard_joy,
        los_guidance_launch,
        velocity_controller_launch,
        drone_sim,
        utility_node,
        run_test_scenario,
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args(default_drone="nautilus")
        + [
            DeclareLaunchArgument(
                "test_scenario",
                default_value="4_corner",
                description="Scenario to run: 4_corner, circle, test_pitch, opposite_point",
            ),
            DeclareLaunchArgument(
                "use_keyboard_joy",
                default_value="true",
                description="Launch keyboard joy node",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
