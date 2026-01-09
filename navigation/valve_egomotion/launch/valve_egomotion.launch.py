from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_val = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('valve_topic', default_value='/aruco_detector/markers'),
        DeclareLaunchArgument('use_sim_time', default_value='false'), # change to true when using rosbags with --clock

        Node(
            package='valve_egomotion',
            executable='valve_egomotion',
            name='valve_egomotion',
            parameters=[{
                'valve_topic': LaunchConfiguration('valve_topic'),
                'use_sim_time': use_sim_time_val,
                'map_frame': 'map',
                'base_frame': 'base_link',
                'cam_frame': 'Orca/camera_front',
                'marker_frame': 'valve_marker_0',
                'max_age_sec': 10.0, 
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_cam',
            arguments=['0.2', '0', '0', '0', '0', '0', 'base_link', 'Orca/camera_front'],
            parameters=[{'use_sim_time': use_sim_time_val}]
        ),
    ])