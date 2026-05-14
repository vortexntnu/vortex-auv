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


def declare_config_type_arg(default_config_type="sim"):
    return DeclareLaunchArgument(
        "config_type",
        default_value=default_config_type,
        description="Controller config variant: 'sim' or 'physical'",
    )


def resolve_config_type(context):
    return LaunchConfiguration("config_type").perform(context)
