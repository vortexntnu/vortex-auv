import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Set environment variable
    set_env_var = SetEnvironmentVariable(
        name="ROSCONSOLE_FORMAT", value="[${severity}] [${time}] [${node}]: ${message}"
    )

    # Thruster Allocator launch
    thrust_allocator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thrust_allocator_auv"),
                "launch",
                "thrust_allocator_auv.launch.py",
            )
        )
    )

    # Thruster Interface launch
    thruster_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("thruster_interface_auv"),
                "launch",
                "thruster_interface_auv.launch.py",
            )
        )
    )

    # Return launch description
    return LaunchDescription(
        [set_env_var, thrust_allocator_launch, thruster_interface_launch]
    )
