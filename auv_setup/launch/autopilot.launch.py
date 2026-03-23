import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    drone = LaunchConfiguration("drone").perform(context)

    los_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("los_guidance"),
                "launch",
                "los_guidance.launch.py",
            )
        ),
        launch_arguments={"drone": drone}.items(),
    )

    autopilot_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velocity_controller"),
                "launch",
                "velocity_controller.launch.py",
            )
        ),
        launch_arguments={"drone": drone}.items(),
    )

    return [los_guidance_launch, autopilot_controller_launch]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "drone",
                default_value="orca",
                description="Drone name / namespace",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
