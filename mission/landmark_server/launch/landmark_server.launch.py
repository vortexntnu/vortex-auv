import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    config_dir = os.path.join(get_package_share_directory("landmark_server"), "config")
    landmark_config = os.path.join(config_dir, "landmark_server_config.yaml")
    env = LaunchConfiguration("env").perform(context)
    if env not in ("sim", "pool"):
        raise RuntimeError(f"env must be sim or pool, not '{env}'")
    # Loaded after the common file: its values win.
    env_config = os.path.join(config_dir, f"{env}.yaml")
    markers_config = os.path.join(config_dir, "markers.yaml")
    # The props (the same everywhere), then where the tasks are:
    # config/course/<env>.yaml unless course:= names another file (a path,
    # or a name in config/course).
    templates_config = os.path.join(config_dir, "course", "templates.yaml")
    # Layouts are read from (and set_course saves them to) the package
    # source when the workspace is built with --symlink-install: the
    # installed files link there, so a layout saved since the build is found.
    course_dir = os.path.dirname(os.path.realpath(templates_config))
    course = LaunchConfiguration("course").perform(context) or env
    course_config = (
        course if os.path.isabs(course) else os.path.join(course_dir, f"{course}.yaml")
    )
    if not os.path.isfile(course_config):
        raise RuntimeError(f"no course layout file {course_config}")

    # Optional files, last so they win: the debug output, and the drift
    # calibration session (README, "Measuring a pool").
    optional = [
        os.path.join(config_dir, f"{name}.yaml")
        for name in ("debug", "calibration")
        if LaunchConfiguration(name).perform(context).lower() == "true"
    ]

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    return [
        Node(
            package="landmark_server",
            executable="landmark_server_node",
            name="landmark_server_node",
            namespace=namespace,
            parameters=[
                landmark_config,
                markers_config,
                env_config,
                templates_config,
                course_config,
                {"course_file": course_config},
                drone_params,
                *optional,
                {"use_sim_time": False},
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            DeclareLaunchArgument(
                "env",
                default_value="sim",
                description="sim (simulator values) or pool (values measured in the pool)",
            ),
            DeclareLaunchArgument(
                "debug",
                default_value="false",
                description="true: publish the debug topics and the map view "
                "(config/debug.yaml); both can also be switched while it runs",
            ),
            DeclareLaunchArgument(
                "calibration",
                default_value="false",
                description="true: drift calibration with an ArUco board (raw odometry, "
                "no graph, no course; config/calibration.yaml)",
            ),
            DeclareLaunchArgument(
                "course",
                default_value="",
                description="Course layout: a file in config/course (without .yaml) "
                "or an absolute path; empty = the one of env",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
