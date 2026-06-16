import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)
    act_as_odom_source = (
        LaunchConfiguration('act_as_odom_source').perform(context).lower() == 'true'
    )

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
    )

    use_sim = LaunchConfiguration('use_sim').perform(context).lower() == 'true'

    param_file_name = "eskf_params.yaml" if use_sim else "eskf_params_real_world.yaml"
    eskf_params = os.path.join(
        get_package_share_directory("eskf"), "config", param_file_name
    )

    environment = (
        'stonefish_sim'
        if use_sim
        else LaunchConfiguration('environment').perform(context)
    )
    env_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "environments",
        f"{environment}.yaml",
    )

    params = [eskf_params, env_params, drone_params, {"frame_prefix": namespace}]
    remappings = []
    if not act_as_odom_source:
        params.append({"publish_tf": False})
        remappings = [
            ("odom",  "eskf/odom"),
            ("pose",  "eskf/pose"),
            ("twist", "eskf/twist"),
        ]

    eskf_odom_topic = "odom" if act_as_odom_source else "eskf/odom"

    eskf_node = Node(
        package="eskf",
        executable="eskf_node",
        name="eskf_node",
        namespace=namespace,
        parameters=params,
        remappings=remappings,
        output="screen",
    )

    xacro_path = os.path.join(
        get_package_share_directory("auv_setup"),
        "description",
        f"{drone}.urdf.xacro",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        remappings=[("joint_states", "servo_state")],
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro", " ", xacro_path]),
                    value_type=str,
                ),
                "frame_prefix": f"{namespace}/",
            }
        ],
    )

    odom_transformer_node = Node(
        package="odom_transformer",
        executable="odom_transformer_node",
        name="odom_transformer_node",
        namespace=namespace,
        parameters=[
            {
                "sensor_frame": "dvl_link",
                "publish_tf": False,
                "publish_pose": False,
                "publish_twist": False,
                "topics.input": "nucleus/odom",
                "topics.output": "nucleus/odom_relative",
                "topics.pose": "pose",
                "topics.twist": "twist",
            },
            drone_params,
            {"frame_prefix": namespace},
        ],
        output="screen",
    )

    rpy_publisher_node = Node(
        package="vortex_utility_nodes",
        executable="rpy_publisher_node",
        name="rpy_publisher_node",
        namespace=namespace,
        parameters=[
            {
                "input_topics": ["nucleus/odom_relative", eskf_odom_topic],
                "output_topics": ["nucleus/odom_relative/rpy", eskf_odom_topic + "/rpy"],
                "input_types": ["odometry", "odometry"],
            }
        ],
        output="screen",
    )

    drone_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("auv_setup"),
                "launch",
                "drone_description.launch.py",
            )
        ),
        launch_arguments={
            "drone": drone,
            "namespace": namespace,
        }.items(),
    )


    return [drone_description_launch, eskf_node, robot_state_publisher_node, rpy_publisher_node, odom_transformer_node]


def generate_launch_description():
    sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Set to "false" to load real-world hardware parameters.',
    )
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='trondheim_freshwater',
        description=(
            'Environment config to load from auv_setup/config/environments/. '
            'If use_sim is true env config is set to stonefish_sim'
        ),
        choices=[
            'longbeach',
            'stonefish_sim',
            'trondheim_freshwater',
            'trondheim_saltwater',
        ],
    )
    act_as_odom_source_arg = DeclareLaunchArgument(
        'act_as_odom_source',
        default_value='true',
        description='If true, publish on standard topics and enable TF. If false, remap to eskf/* topics and disable TF.',
    )
    return LaunchDescription(
        [sim_arg, environment_arg, act_as_odom_source_arg]
        + declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )
