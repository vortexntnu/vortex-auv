import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def declare_drone_and_namespace_args(default_drone="nautilus"):
    return [
        DeclareLaunchArgument(
            "drone",
            default_value=default_drone,
            description="Drone model / config name",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="ROS namespace. If empty, uses drone name.",
        ),
    ]


def resolve_drone_and_namespace(context):
    drone = LaunchConfiguration("drone").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    if namespace == "":
        namespace = drone
    return drone, namespace


def declare_config_type_arg(default_config_type="physical"):
    return DeclareLaunchArgument(
        "config_type",
        default_value=default_config_type,
        description="Controller config variant: 'sim' or 'physical'",
    )


def resolve_config_type(context):
    return LaunchConfiguration("config_type").perform(context)


def drone_params(auv_setup_dir, drone, config_type):
    """robots/<drone>.yaml, then robots/<drone>_sim.yaml on top in the sim."""
    robots = os.path.join(auv_setup_dir, "config", "robots")
    files = [os.path.join(robots, f"{drone}.yaml")]
    sim = os.path.join(robots, f"{drone}_sim.yaml")
    if config_type == "sim" and os.path.exists(sim):
        files.append(sim)
    return files
