from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('valve_topic', default_value='/aruco_detector/landmarks'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        Node(
            package='valve_egomotion',
            executable='valve_egomotion_node',
            name='valve_egomotion',
            parameters=[{
                'valve_topic': LaunchConfiguration('valve_topic'),
                'use_sim_time': use_sim_time,
                'ref_frame': 'vo_ref',
                'base_frame': 'base_link',
                'cam_frame': 'Orca/camera_front',
                'max_age_sec': 10.0,
                'window_size': 20,
                'publish_tf': False,
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_cam',
            arguments=['0.2', '0', '0', '0', '0', '0', 'base_link', 'Orca/camera_front'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
