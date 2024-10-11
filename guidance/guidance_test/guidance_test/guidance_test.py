import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('guidance_test')
        self.publisher_ = self.create_publisher(Float32MultiArray,
                                                'guidance/los', 10)
        self.timer = self.create_timer(1.0, self.publish_velocity)

    def publish_velocity(self):
        msg = Float32MultiArray()
        # Creating a data array to include surge velocity, pitch angle, and heading angle
        # Example values: [surge_velocity, pitch_angle, heading_angle]
        msg.data = [1.5, 0.1, 0.75, 5.0,
                    5.0]  # Replace with actual dynamic values as needed

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "{}"'.format(msg.data))


def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()

    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
