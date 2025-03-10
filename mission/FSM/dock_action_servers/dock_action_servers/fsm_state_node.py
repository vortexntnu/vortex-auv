#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yasmin_msgs.msg import StateMachine


class FSMStateNode(Node):
    def __init__(self):
        super().__init__('fsm_state_node')
        self.subscription = self.create_subscription(
            StateMachine, '/fsm_viewer', self.listener_callback, 10
        )

        self.declare_parameter("topics.fsm.active_controller", "")
        publish_topic = (
            self.get_parameter("topics.fsm.active_controller")
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(f'Publishing to topic: {publish_topic}')

        self.publisher = self.create_publisher(String, publish_topic, 10)
        self.last_state_id = -1

    def listener_callback(self, fsm_msg: StateMachine):
        state_id = fsm_msg.states[0].current_state

        if self.last_state_id == state_id or state_id == -1:
            return

        if state_id != -1:
            current_state_name = fsm_msg.states[state_id].name
        else:
            current_state_name = "None"

        controller_message = self.get_controller_message(current_state_name)
        msg = String()
        msg.data = controller_message

        if msg.data != 'None':
            self.get_logger().info(f'Message to publish: {msg.data}')
            self.publisher.publish(msg)
            self.last_state_id = state_id

    def get_controller_message(self, current_state):
        """Returns the controller message based on the current state."""
        if current_state in ['APPROACH_DOCKING_STATION']:
            return 'LQR'
        elif current_state in [
            'GO_ABOVE_DOCKING_STATION',
            'CONVERGE_DOCKING_STATION',
            'RETURN_HOME',
        ]:
            return 'PID'
        else:
            return 'None'


def main(args=None):
    rclpy.init(args=args)
    fsm_state_node = FSMStateNode()
    try:
        rclpy.spin(fsm_state_node)
    except KeyboardInterrupt:
        pass
    finally:
        fsm_state_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
