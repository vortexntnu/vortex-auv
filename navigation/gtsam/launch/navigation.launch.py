"""Run navigation against the existing Nautilus robot description and sensor topics."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create an isolated navigation launch without changing controller setup."""
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="nautilus"),
            DeclareLaunchArgument("start_description", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("publish_tf", default_value="false"),
            DeclareLaunchArgument(
                "imu_profile", default_value="stim300_10g_provisional"
            ),
            DeclareLaunchArgument("odom_topic", default_value="gtsam/odom"),
            DeclareLaunchArgument("imu_topic", default_value="imu/data_raw"),
            DeclareLaunchArgument("dvl_topic", default_value="dvl/twist"),
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    get_package_share_directory("gtsam_navigation"),
                    "config",
                    "navigation.yaml",
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("auv_setup"),
                        "launch",
                        "drone_description.launch.py",
                    )
                ),
                launch_arguments={"drone": "nautilus", "namespace": namespace}.items(),
                condition=IfCondition(LaunchConfiguration("start_description")),
            ),
            Node(
                package="gtsam_navigation",
                executable="gtsam_navigation_node",
                namespace=namespace,
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "frame_prefix": namespace,
                        "imu_profile": LaunchConfiguration("imu_profile"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "imu_topic": LaunchConfiguration("imu_topic"),
                        "dvl_topic": LaunchConfiguration("dvl_topic"),
                        "use_sim_time": ParameterValue(
                            LaunchConfiguration("use_sim_time"), value_type=bool
                        ),
                        "publish_tf": ParameterValue(
                            LaunchConfiguration("publish_tf"), value_type=bool
                        ),
                    },
                ],
                output="screen",
            ),
        ]
    )
