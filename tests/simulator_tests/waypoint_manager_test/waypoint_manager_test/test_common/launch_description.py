import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from vortex_stonefish_test_description import generate_sim_test_description

ORCA_NS = "orca"


def generate_wm_test_description(
    scenario="default", rendering=True, bag=True, delay=5.0
):
    """Shared launch description for all WM tests."""
    rf_cfg = os.path.join(
        get_package_share_directory("reference_filter_dp"),
        "config",
        "reference_filter_params.yaml",
    )
    adapt_cfg = os.path.join(
        get_package_share_directory("dp_adapt_backs_controller"),
        "config",
        "adapt_params.yaml",
    )
    orca_cfg = os.path.join(
        get_package_share_directory("auv_setup"), "config", "robots", "orca.yaml"
    )

    extra_nodes = [
        Node(
            package="waypoint_manager",
            executable="waypoint_manager_node",
            name="waypoint_manager_node",
            namespace=ORCA_NS,
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="reference_filter_dp",
            executable="reference_filter_dp_node",
            name="reference_filter_node",
            namespace=ORCA_NS,
            parameters=[rf_cfg, orca_cfg],
            output="screen",
        ),
        Node(
            package="dp_adapt_backs_controller",
            executable="dp_adapt_backs_controller_node",
            name="dp_adapt_backs_controller_node",
            namespace=ORCA_NS,
            parameters=[adapt_cfg, orca_cfg],
            output="screen",
        ),
        Node(
            package="thrust_allocator_auv",
            executable="thrust_allocator_auv_node",
            name="thrust_allocator_auv_node",
            namespace=ORCA_NS,
            parameters=[orca_cfg],
            output="screen",
        ),
    ]

    return generate_sim_test_description(
        scenario_value=scenario,
        rendering_enabled=str(rendering).lower(),
        extra_nodes=extra_nodes,
        delay=delay,
        record_bag=bag,
    )
