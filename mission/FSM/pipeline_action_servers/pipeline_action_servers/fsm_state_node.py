#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from yasmin_msgs.msg import StateMachine
import yaml

class FSMStateNode(Node):
    """ 
    publishes the active controller based on the current state of the FSM"""
    def __init__(self):
        """
        Initialize the node and create the subscription and publisher."""
        super().__init__('fsm_state_node')
        self.subscription = self.create_subscription(
            StateMachine,
            '/fsm_viewer',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, '/fsm_active_controller', 10)
        #self.subscription  # prevent unused variable warning
        self.last_state_id = -1

    def listener_callback(self, fsm_msg: StateMachine):
        """
        Callback function for the subscriber to the FSM state topic.
        Matches the id of the current state to the name, and if the state is corresponding to a controller, publishes the controller name."""
        try:
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
            
        except Exception as e:
            self.get_logger().error(f'Failed to process message: {e}')

    def get_controller_message(self, current_state):
        """
        Returns the controller message based on the current state.
        """
        if current_state == 'GO_TO_START_OF_PIPELINE' or current_state == 'FOLLOW_PIPELINE' or current_state == 'RETURN_HOME':
            return 'LQR'
        else:
            return 'None'

def main(args=None):
    """
    Main function to run the FSMStateNode node."""
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
