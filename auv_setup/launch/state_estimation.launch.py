import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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

    container = ComposableNodeContainer(
        name="eskf_container",
        namespace=namespace,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="eskf",
                plugin="ESKFNode",
                name="eskf_node",
                namespace=namespace,
                parameters=[
                    drone_params,
                    {
                        "diag_Q_std": [
                            0.05, 0.05, 0.1,
                            0.01, 0.01, 0.02,
                            0.001, 0.001, 0.001,
                            0.0001, 0.0001, 0.0001
                        ],

                        "diag_p_init": [
                            1.0, 1.0, 1.0,
                            0.5, 0.5, 0.5,
                            0.1, 0.1, 0.1,
                            0.001, 0.001, 0.001,
                            0.001, 0.001, 0.001
                        ],

                        "transform.imu_frame_r": [
                            -1.0, 0.0, 0.0,
                             0.0, 1.0, 0.0,
                             0.0, 0.0, -1.0
                        ],
                        "transform.imu_frame_t": [
                            0.0, 0.0, 0.0
                        ],

                        "transform.dvl_frame_r": [
                            0.0, -1.0, 0.0,
                            1.0,  0.0, 0.0,
                            0.0,  0.0, 1.0
                        ],
                        "transform.dvl_frame_t": [
                            0.4, 0.0, 0.2
                        ],

                        "transform.depth_frame_t": [
                            0.0, 0.0, 0.0
                        ],

                        "use_tf_transforms": True,
                        "publish_tf": True,
                        "publish_pose": True,
                        "publish_twist": True,
                        "publish_rate_ms": 5,
                        "add_gravity_to_imu": True,
                        "frame_prefix": namespace,
                        "initial_gyro_bias": [0.0, 0.0, 0.0],
                        "initial_accel_bias": [0.0, 0.0, -0.05]
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="mru_ros_interface",
                plugin="MruRosInterface",
                name="mru_ros_interface_node",
                namespace=namespace,
                parameters=[
                    {
                        "imu_pub_topic": f"/{namespace}/imu/data_raw",
                        "frame_id": f"/{namespace}/imu_link",
                        "connection_params.remote_ip": "10.0.1.20", # MRU IP
                        "connection_params.data_remote_port": 7550,
                        "connection_params.data_local_port": 7551,
                        "connection_params.control_local_port": 7552,
                        "mru_settings.channel": "UDP1",
                        "mru_settings.port": 7551,
                        "mru_settings.ip_addr": "10.0.0.69", # Host computer IP
                        "mru_settings.format": "MRUBIN",
                        "mru_settings.interval": 5,
                        "mru_settings.token": 21,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="nortek_nucleus_ros_interface",
                plugin="NortekNucleusRosInterface",
                name="nortek_nucleus_ros_interface_node",
                namespace=namespace,
                parameters=[
                    {
                        "frame_id": f"/{namespace}/nucleus_frame",
                        "connection_params.remote_ip": "10.0.0.42",
                        "connection_params.data_remote_port": 9000,
                        "connection_params.password": "",
                        "enable_imu": False,
                        "enable_ins_odom": False,
                        "enable_dvl": True,
                        "enable_pressure": True,
                        "enable_magnetometer": False,
                        "enable_ins_twist": False,
                        "enable_ins_position": False,
                        "enable_ins_pose": False,
                        "imu_data_raw_pub_topic": f"/{namespace}/nucleus/imu/data_raw",
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
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
    )

    return [drone_description_launch, container]


def generate_launch_description():
    return LaunchDescription(
        declare_drone_and_namespace_args()
        + [
            OpaqueFunction(function=launch_setup),
        ]
    )