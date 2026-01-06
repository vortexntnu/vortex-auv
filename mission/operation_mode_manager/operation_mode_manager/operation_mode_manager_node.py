#!/usr/bin/env python3

import rclpy
from rclpy.node import Node, Parameter
from vortex_msgs.srv import OperationModeSRV
from std_msgs.msg import String, Bool
from vortex_msgs.msg import OperationMode
from vortex_utils.qos_profiles import reliable_profile, sensor_data_profile
from operation_mode_manager.mode_manager_utils import modes
from geometry_msgs.msg import WrenchStamped

# the Operation Mode Topic consists of:
# AUTONOMOUS = 1
# MANUAL = 2

# Whereas the killswitch is true or false

# This node takes requests:
# absolute (true/false)
# mode (KILLSWITCH = 0/AUTONOMOUS = 1/MANUAL = 2)
# Whereas if absolute is false switching modes will either toggle killswitch or switch modes depending on the request, keeping killswitch at its current state if so.
# If absolute is true then either the killswitch will be set to true if requested, or if any other mode is requested, killswitch will deactivate and switch to the requested mode

class OperationModeManager(Node):
    def __init__(self):
        super().__init__('operation_mode_manager')
        self.srv = self.create_service(OperationModeSRV, 'set_operation_mode', self.set_operation_mode_callback)

        self.get_parameters()
        self.setup_publishers()

        # Set initial mode to KILLSWITCH
        self._killswitch = True
        self._mode = OperationModeSRV.Request.MANUAL

        self.get_logger().info(f"Operation Mode Manager Node has been started. Current mode: {modes[self._mode]}")
    
    def get_parameters(self):

        topic_params = ['wrench_input', 'killswitch', 'operation_mode']

        for param in topic_params:
            self.declare_parameter(f'topics.{param}', Parameter.Type.STRING)
            setattr(
                self,
                param + '_topic',
                self.get_parameter(f'topics.{param}').value,
            )
        
    
    def setup_publishers(self):
        best_effort_qos = sensor_data_profile(1)
        reliable_qos = reliable_profile(2)

        self._wrench_publisher = self.create_publisher(
            WrenchStamped, self.wrench_input_topic, qos_profile=best_effort_qos
        )
        self._software_killswitch_signal_publisher = self.create_publisher(
            Bool, self.killswitch_topic, reliable_qos
        )
        self._software_killswitch_signal_publisher.publish(Bool(data=True))
        self._operational_mode_signal_publisher = self.create_publisher(
            OperationMode, self.operation_mode_topic, reliable_qos
        )

    def set_operation_mode_callback(self, request: OperationModeSRV.Request, response: OperationModeSRV.Response):
        if request.mode not in modes:
            # Invalid mode requested
            self.get_logger().error(f"Invalid operation mode requested: {request.mode}")
            response.mode = self._mode
            response.killswitch_status = self._killswitch
            return response
        elif request.absolute == True:
            # Handle absolute mode setting
            if request.mode == OperationModeSRV.Request.KILLSWITCH:
                # Activate killswitch
                self.get_logger().info("Activating KILLSWITCH")
                self._software_killswitch_signal_publisher.publish(Bool(data=True))
                empty_wrench_msg = WrenchStamped()
                empty_wrench_msg.header.stamp = self.get_clock().now().to_msg()
                empty_wrench_msg.header.frame_id = "base_link"
                self._wrench_publisher.publish(empty_wrench_msg)

                self._operational_mode_signal_publisher.publish(OperationMode(mode=self._mode))
                response.mode = self._mode
                response.killswitch_status = True
                self._killswitch = True
                return response
            else:
                # Deactivate killswitch and switch to requested mode
                self.get_logger().info(f"Deactivating KILLSWITCH and setting operation mode to: {modes[request.mode]}")
                self._software_killswitch_signal_publisher.publish(Bool(data=False))
                self._operational_mode_signal_publisher.publish(OperationMode(mode=request.mode))
                self._mode = request.mode
                self._killswitch = False
                response.mode = request.mode
                response.killswitch_status = False
                return response
        # Handle the service request and set the operation mode
        elif request.mode == self._mode:
            # Requested mode is the same as current mode
            self.get_logger().info(f"Operation mode is already set to: {modes[request.mode]}")
            response.mode = self._mode
            response.killswitch_status = self._killswitch
            return response
        elif request.mode == OperationModeSRV.Request.KILLSWITCH:
            # Handle killswitch
            if self._killswitch == True:
                self.get_logger().info("Toggling KILLSWITCH off")
                # Deactivate killswitch
                self._software_killswitch_signal_publisher.publish(Bool(data=False))
                self._operational_mode_signal_publisher.publish(OperationMode(mode=self._mode))
                self._killswitch = False
                response.mode = self._mode
                response.killswitch_status = False
                return response
            # Activate killswitch
            self.get_logger().info("Toggling KILLSWITCH on")
            self._software_killswitch_signal_publisher.publish(Bool(data=True))
            empty_wrench_msg = WrenchStamped()
            empty_wrench_msg.header.stamp = self.get_clock().now().to_msg()
            empty_wrench_msg.header.frame_id = "base_link"
            self._wrench_publisher.publish(empty_wrench_msg)

            self._operational_mode_signal_publisher.publish(OperationMode(mode=self._mode))
            response.mode = self._mode
            response.killswitch_status = True
            self._killswitch = True
            return response
        elif self._killswitch == True:
            # If killswitch is active, switch mode but keep killswitch active
            self.get_logger().info(f"Killswitch Active, setting operation mode to: {modes[request.mode]}")
            self._operational_mode_signal_publisher.publish(OperationMode(mode=request.mode))
            self._mode = request.mode
            response.mode = request.mode
            response.killswitch_status = self._killswitch
            return response
        # Switch mode
        self.get_logger().info(f"Setting operation mode to: {modes[request.mode]}")
        self._mode = request.mode
        self._operational_mode_signal_publisher.publish(OperationMode(mode=request.mode))
        response.mode = request.mode
        response.killswitch_status = self._killswitch
        return response

def main():
    rclpy.init()
    operation_mode_manager = OperationModeManager()
    rclpy.spin(operation_mode_manager)
    operation_mode_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()