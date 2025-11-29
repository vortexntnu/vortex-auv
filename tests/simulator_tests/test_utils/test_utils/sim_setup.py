import os
import yaml
import tempfile
import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import launch_testing.actions


def _write_temp_config(config: dict) -> str:
    tmp_file = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
    yaml.safe_dump(config, tmp_file)
    tmp_file.close()
    return tmp_file.name


def _load_config(scenario: str) -> dict:
    config_path = os.path.join(
        get_package_share_directory("stonefish_sim"), "config", f"{scenario}_config.yaml"
    )

    if not os.path.exists(config_path):
        print(f"Warning: Scenario config not found for scenario '{scenario}'. Using default config.")
        config_path = os.path.join(
            get_package_share_directory("stonefish_sim"), "config", "default_config.yaml"
        )

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    return config


def _prepare_bag_recording():
    """
    Creates a unique per-run rosbag directory inside the build folder of the test package.
    """
    # The test always runs inside <ws>/build/<pkg>, so cwd is that dir.
    build_dir = os.getcwd()
    rosbag_base = os.path.join(build_dir, "rosbags")

    os.makedirs(rosbag_base, exist_ok=True)

    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    bag_dir = os.path.join(rosbag_base, f"bag_{timestamp}")

    print(f"[SIM_SETUP] Recording rosbag to: {bag_dir}")

    return ExecuteProcess(
        cmd=["ros2", "bag", "record", "-a", "-o", bag_dir, "-s", "mcap"],
        output="screen",
    )


def default_launch_setup(context, modify_config_fn=None, extra_nodes=None, record_bag=False):
    """Creates the simulation launch setup with optional overrides and extra nodes."""
    scenario = context.launch_configurations.get("scenario", "docking")

    config = _load_config(scenario)
    if modify_config_fn:
        config = modify_config_fn(config)

    tmp_path = _write_temp_config(config)

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("stonefish_sim"), "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "scenario": scenario,
            "scenario_config_override": tmp_path,
        }.items(),
    )

    sim_interface = Node(
        package="vortex_sim_interface",
        executable="vortex_sim_interface",
        name="vortex_sim_interface",
        output="screen",
        emulate_tty=True,
    )

    nodes = [sim_launch, sim_interface]

    if extra_nodes:
        nodes.extend(extra_nodes)

    if record_bag:
        nodes.append(_prepare_bag_recording())

    return nodes


def generate_sim_test_description(
    modify_config_fn=None,
    delay=5.0,
    scenario_value="docking",
    extra_nodes=None,
    record_bag=False,
):
    """Reusable test description generator for simulation-based tests."""
    return (
        LaunchDescription([
            DeclareLaunchArgument("scenario", default_value=scenario_value),
            OpaqueFunction(function=lambda ctx, *a, **k:
                default_launch_setup(
                    ctx,
                    modify_config_fn=modify_config_fn,
                    extra_nodes=extra_nodes,
                    record_bag=record_bag,
                )
            ),
            TimerAction(period=delay, actions=[launch_testing.actions.ReadyToTest()]),
        ]),
        {},
    )
