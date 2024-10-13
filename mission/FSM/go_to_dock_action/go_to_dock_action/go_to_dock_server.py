import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from go_to_dock_action.action import GoToDock  # Import the action definition
import math

class GoToDockServer(Node):

    def __init__(self):
        super().__init__('go_to_dock_server')

        self._action_server = ActionServer(
            self,
            GoToDock,
            'go_to_dock',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal to dock at: {}'.format(goal_handle.request.docking_position))

        feedback_msg = GoToDock.Feedback()
        docking_position = goal_handle.request.docking_position  # [x, y, z]
        auv_position = [0.0, 0.0, 0.0]  # Let's say AUV starts at (0,0,0)
        rate = self.create_rate(1)  # Simulate 1Hz feedback

        while not self.is_at_dock(auv_position, docking_position):
            distance_to_dock = self.calculate_distance(auv_position, docking_position)
            feedback_msg.distance_to_dock = distance_to_dock
            self.get_logger().info('Distance to dock: {:.2f}'.format(distance_to_dock))

            goal_handle.publish_feedback(feedback_msg)

            # Simulate AUV moving towards the dock
            auv_position = self.move_towards_dock(auv_position, docking_position)

            rate.sleep()

        goal_handle.succeed()
        result = GoToDock.Result()
        result.success = True
        return result

    def is_at_dock(self, auv_position, docking_position):
        distance = self.calculate_distance(auv_position, docking_position)
        return distance < 0.5

    def calculate_distance(self, pos1, pos2):
        return math.sqrt(sum([(p1 - p2) ** 2 for p1, p2 in zip(pos1, pos2)]))

    def move_towards_dock(self, auv_position, docking_position):
        step_size = 0.5
        new_position = []
        for auv_coord, dock_coord in zip(auv_position, docking_position):
            if abs(dock_coord - auv_coord) > step_size:
                new_position.append(auv_coord + step_size if dock_coord > auv_coord else auv_coord - step_size)
            else:
                new_position.append(dock_coord)
        return new_position

def main(args=None):
    rclpy.init(args=args)
    node = GoToDockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
