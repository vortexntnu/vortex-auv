"""Standalone sensor simulation using the existing Nautilus mounting assumptions."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Launch the estimator and sensor generator, without a vehicle/controller dependency."""
    config = os.path.join(
        get_package_share_directory("gtsam_navigation"), "config", "navigation.yaml"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trajectory",
                default_value="turn",
                choices=[
                    "stationary",
                    "straight",
                    "turn",
                    "rotate",
                    "barrel_roll",
                    "square_barrel_roll",
                ],
                description="square_barrel_roll returns around a square, aligns, rolls once and stops",
            ),
            DeclareLaunchArgument(
                "duration",
                default_value=PythonExpression(
                    [
                        "'100.0' if '",
                        LaunchConfiguration("trajectory"),
                        "' == 'square_barrel_roll' else '60.0'",
                    ]
                ),
            ),
            DeclareLaunchArgument("imu_rate", default_value="1000.0"),
            DeclareLaunchArgument("dvl_rate", default_value="8.0"),
            DeclareLaunchArgument("publish_rate", default_value="125.0"),
            DeclareLaunchArgument("noise", default_value="true"),
            DeclareLaunchArgument(
                "imu_profile", default_value="stim300_10g_provisional"
            ),
            DeclareLaunchArgument("seed", default_value="42"),
            DeclareLaunchArgument("stress_scale", default_value="1.0"),
            DeclareLaunchArgument(
                "dropout_start",
                default_value=PythonExpression(
                    [
                        "'-1.0' if '",
                        LaunchConfiguration("trajectory"),
                        "' == 'square_barrel_roll' else '20.0'",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "dropout_end",
                default_value=PythonExpression(
                    [
                        "'-1.0' if '",
                        LaunchConfiguration("trajectory"),
                        "' == 'square_barrel_roll' else '25.0'",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "dvl_max_tilt_deg",
                default_value="30.0",
                description="Assumed DVL bottom-lock tilt limit, not a sensor specification",
            ),
            Node(
                package="gtsam_navigation",
                executable="gtsam_navigation_node",
                namespace="nautilus",
                output="screen",
                parameters=[
                    config,
                    {
                        "use_sim_time": True,
                        "imu_profile": LaunchConfiguration("imu_profile"),
                        "publish_rate": ParameterValue(
                            LaunchConfiguration("publish_rate"), value_type=float
                        ),
                    },
