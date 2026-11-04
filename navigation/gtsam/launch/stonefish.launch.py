"""Nautilus dynamics with estimate-driven DP; external repositories unchanged."""

import os

from ament_index_python.packages import get_package_share_directory as share
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    rendering = LaunchConfiguration('rendering').perform(context) == 'true'
    device = LaunchConfiguration('input').perform(context)
    robot = os.path.join(share('auv_setup'), 'config/robots/nautilus.yaml')
    package = share('gtsam_navigation')

    def node(pkg, executable, params=None, remaps=None, **kwargs):
        return Node(
            package=pkg,
            executable=executable,
            namespace='nautilus',
            parameters=[robot, *(params or []), {'use_sim_time': False}],
            remappings=remaps or [],
            output='screen',
            **kwargs,
        )

    def include(pkg, filename, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share(pkg), 'launch', filename)),
            launch_arguments=args.items(),
        )

    common = {'drone': 'nautilus', 'namespace': 'nautilus'}
    sim_args = [
        os.path.join(share('stonefish_sim'), 'data'),
        os.path.join(package, 'scenarios', 'nautilus_gtsam.scn'),
        '1000.0',
    ]
    if rendering:
        sim_args += ['1280', '720', 'medium']
    nodes = [
        Node(
            package='stonefish_ros2',
            executable='stonefish_simulator'
            if rendering
            else 'stonefish_simulator_nogpu',
            name='gtsam_stonefish',
            arguments=sim_args,
            parameters=[{'use_sim_time': False}],
            output='screen',
        ),
        include('auv_setup', 'drone_description.launch.py', common),
        node(
            'operation_mode_manager',
            'operation_mode_manager_cpp',
            [{'initial_mode': 1, 'initial_killswitch': True}],
            [('wrench_input', 'gtsam/command/mode')],
        ),
        node(
            'joystick_interface_auv',
            'joystick_interface_auv_node.py',
            [
                os.path.join(
                    share('joystick_interface_auv'),
                    'config/param_joystick_interface_auv.yaml',
                ),
                {'drone': 'nautilus', 'orientation_mode': 'quat'},
            ],
            [('wrench_input', 'gtsam/command/manual')],
        ),
        node(
            'dp_adapt_backs_controller_quat',
            'dp_adapt_backs_controller_quat_node',
            [
                os.path.join(
                    share('dp_adapt_backs_controller_quat'),
                    'config/adapt_params_nautilus_sim.yaml',
                )
            ],
            [('wrench_input', 'gtsam/command/dp')],
        ),
        node(
            'thrust_allocator_auv',
            'thrust_allocator_auv_node',
            [{'propulsion.solver_type': 'qp'}],
        ),
        node(
            'stonefish_sim_interface',
            'stonefish_sim_interface',
            [{'mock_odom': False, 'tf_name_prefix': 'nautilus', 'drone': 'nautilus'}],
            [
                ('odom', 'stonefish/unused_odom'),
                ('pose', 'stonefish/unused_pose'),
                ('twist', 'stonefish/unused_twist'),
                ('dvl/twist', 'stonefish/unused_dvl'),
            ],
        ),
        node(
            'gtsam_navigation',
            'stonefish_sensors.py',
            [
                {
                    'dvl_max_tilt_deg': float(
                        LaunchConfiguration('dvl_max_tilt_deg').perform(context)
                    )
                }
            ],
        ),
        node(
            'gtsam_navigation',
            'gtsam_navigation_node',
            [
                os.path.join(package, 'config/navigation.yaml'),
                {
                    'frame_prefix': 'nautilus',
                    'publish_tf': True,
                    'diagnostic_hardware_id': 'Stonefish: provisional STIM300 + generic four-beam DVL',
                },
            ],
        ),
        node('gtsam_navigation', 'stonefish_control.py'),
        node('gtsam_navigation', 'stonefish_evaluation.py'),
    ]
    if device == 'joystick':
        nodes.append(
            node(
                'joy',
                'joy_node',
                [{'deadzone': 0.15, 'autorepeat_rate': 100.0}],
                [('/joy', '/nautilus/joy')],
            )
        )
    elif device == 'keyboard':
        nodes.append(include('keyboard_joy', 'keyboard_joy_node.launch.py', common))
    if LaunchConfiguration('foxglove').perform(context) == 'true':
        nodes.append(
            Node(
                package='foxglove_bridge',
                executable='foxglove_bridge',
                parameters=[
                    {
                        'port': int(
                            LaunchConfiguration('foxglove_port').perform(context)
                        ),
                        'use_sim_time': False,
                    }
                ],
                output='screen',
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'rendering', default_value='true', choices=['true', 'false']
            ),
            DeclareLaunchArgument(
                'input',
                default_value='joystick',
                choices=['joystick', 'keyboard', 'none'],
            ),
            DeclareLaunchArgument(
                'foxglove', default_value='true', choices=['true', 'false']
            ),
            DeclareLaunchArgument('foxglove_port', default_value='8765'),
            DeclareLaunchArgument('dvl_max_tilt_deg', default_value='25.0'),
            OpaqueFunction(function=setup),
        ]
    )
