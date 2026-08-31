import unittest
import launch_testing
import launch_testing.actions
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import pytest
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import ExecuteProcess


@pytest.mark.launch_test
def generate_test_description():
    params_file = os.path.join(
        get_package_share_directory('velocity_controller'),
        'config', 'nautilus_params.yaml'
    )
    global_share = get_package_share_directory('auv_setup')
    params_file_2 = os.path.join(global_share, 'config', 'robots', "nautilus.yaml")

    velocity_container = ComposableNodeContainer(
        name='nautilus_velocity_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='velocity_controller',
                plugin='Velocity_node',
                name='velocity_controller_node',
                parameters=[params_file, params_file_2],
            )
        ],
        output='screen',
    )

    integration_test_node = ExecuteProcess(
    cmd=[os.path.join(
        get_package_prefix('velocity_controller'),
        'lib', 'velocity_controller', 'test_velocity_controller_integration'
    )],
    output='screen',
    )

    return LaunchDescription([
        velocity_container,
        integration_test_node,
        launch_testing.actions.ReadyToTest(),
    ]), {
        'container': velocity_container,
        'test_node': integration_test_node,
    }


class TestVelocityControllerLaunch(unittest.TestCase):
    def test_test_node_exits_successfully(self, proc_info, test_node):
        # Venter til integration_test_node-prosessen avslutter, og sjekker exit code.
        # Selve assertene ligger inni C++-testen (GTest), denne kun verifiserer
        # at HELE launch-oppsettet (container + node-lasting + testkjøring)
        # kjørte uten å krasje - akkurat den originale feilscenarioet fra, get_package_prefix
        # start av samtalen ville blitt fanget her.
        proc_info.assertWaitForShutdown(process=test_node, timeout=30)
        launch_testing.asserts.assertExitCodes(proc_info, process=test_node)


@launch_testing.post_shutdown_test()
class TestVelocityControllerLaunchAfterShutdown(unittest.TestCase):
    def test_container_exit_code(self, proc_info, container):
        # Container skal ikke krasje uventet selv om test-noden er ferdig
        launch_testing.asserts.assertExitCodes(
            proc_info, process=container,
            allowable_exit_codes=[0, -15]  # 0 = clean, -15 = SIGTERM ved launch-shutdown
        )