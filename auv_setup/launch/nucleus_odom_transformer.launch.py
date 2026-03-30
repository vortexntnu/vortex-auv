import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from auv_setup.launch_arg_common import (
    declare_drone_and_namespace_args,
    resolve_drone_and_namespace,
)


def launch_setup(context, *args, **kwargs):
    drone, namespace = resolve_drone_and_namespace(context)

    drone_params = os.path.join(
        get_package_share_directory("auv_setup"),
        "config",
        "robots",
        f"{drone}.yaml",
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

    nortek_nucleus_ros_interface_node = Node(
        package="nortek_nucleus_ros_interface",
        executable="nortek_nucleus_ros_interface_node",
        name="nortek_nucleus_ros_interface_node",
        namespace=namespace,
        parameters=[
            {
                "frame_id": f"{namespace}/nucleus_frame",
                "connection_params.remote_ip": "10.0.0.42",
                "connection_params.data_remote_port": 9000,
                "connection_params.password": "",
                "enable_imu": False,
                "enable_ins_odom": True,
                "enable_dvl": False,
                "enable_pressure": False,
                "enable_magnetometer": False,
                "enable_ins_twist": False,
                "enable_ins_position": False,
                "enable_ins_pose": False,
                "imu_data_raw_pub_topic": f"/{namespace}/imu/data_raw",
                "imu_data_pub_topic": f"/{namespace}/imu/data",
                "ins_pub_topic": f"/{namespace}/nucleus/odom",
                "dvl_pub_topic": f"/{namespace}/nucleus/dvl",
                "pressure_pub_topic": f"/{namespace}/nucleus/pressure",
                "magnetometer_pub_topic": f"/{namespace}/imu/mag",
                "ins_twist_pub_topic": f"/{namespace}/nucleus/ins/twist",
                "ins_position_pub_topic": f"/{namespace}/nucleus/ins/position",
                "ins_pose_pub_topic": f"/{namespace}/nucleus/ins/pose",
                "imu_settings.freq": 125,
                "ahrs_settings.freq": 10,
                "ahrs_settings.mode": 0,
                "bottom_track_settings.mode": 2,
                "bottom_track_settings.velocity_range": 5,
                "bottom_track_settings.enable_watertrack": False,
                "fast_pressure_settings.enable": True,
                "fast_pressure_settings.sampling_rate": 16,
                "magnetometer_settings.freq": 75,
                "magnetometer_settings.mode": 0,
                "instrument_settings.rotxy": 180.0,
                "instrument_settings.rotyz": 0.0,
                "instrument_settings.rotxz": 0.0,
            },
            drone_params,
        ],
        output="screen",
    )

    odom_transformer_node = Node(
        package="odom_transformer",
        executable="odom_transformer_node",
        name="odom_transformer_node",
        namespace=namespace,
        parameters=[
            {
                "sensor_frame": "dvl_link",
                "publish_tf": True,
                "publish_pose": True,
                "publish_twist": True,
                "topics.input": f"/{namespace}/nucleus/odom",
                "topics.output": f"/{namespace}/odom",
                "topics.pose": f"/{namespace}/pose",
                "topics.twist": f"/{namespace}/twist",
            },
            drone_params,
            {"frame_prefix": namespace},
        ],
        output="screen",
    )

    return [
        drone_description_launch,
        nortek_nucleus_ros_interface_node,
        odom_transformer_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [OpaqueFunction(function=launch_setup)]
    )