import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    los_guidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("los_guidance"),
                "launch/los_guidance.launch.py",
            )
        )
    )

    autopilot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("velocity_controller_lqr"),
                "launch/velocity_controller_lqr.launch.py",
            )
        )
    )

    return LaunchDescription([los_guidance_launch, autopilot_controller])
