#!/usr/bin/python3

import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray

from acoustics_interface_auv.acoustics_interface_lib import TeensyCommunicationUDP


class AcousticsInterfaceNode(Node):
    """Publishes Acoustics data to ROS2.

    Methods:
        data_update() -> None:
            calls fetch_data() from acoustics_interface
        data_publisher(self) -> None:
            publishes data to ROS2 topics

    """

    def __init__(self) -> None:
        """Sets up acoustics logging and publishers, also sets up teensy communication."""
        super().__init__("acoustics_interface_auv")
        rclpy.logging.initialize()

        self.get_topics()

        self.create_publishers_and_subscribers()

        # Logs all the newest data
        self.declare_parameter(
            "acoustics.data_logging_rate", 1.0
        )  # Providing a default value 1.0 => 1 samplings per second, very slow
        data_logging_rate = (
            self.get_parameter("acoustics.data_logging_rate")
            .get_parameter_value()
            .double_value
        )
        timer_period = 1.0 / data_logging_rate

        self._timer_data_update = self.create_timer(0.001, self.data_update)
        self._timer_data_publisher = self.create_timer(
            timer_period, self.data_publisher
        )

        # Declare Frequency of interest parameters to send to Acoustics PCB to look out for
        # This list has to be exactly 10 entries long (20 elements (10 frequencies + 10 variances))
        # format [(FREQUENCY, FREQUENCY_VARIANCE), ...]
        self.declare_parameter("acoustics.frequencies_of_interest", [0] * 20)
        frequencies_of_interest_parameters = (
            self.get_parameter("acoustics.frequencies_of_interest")
            .get_parameter_value()
            .integer_array_value
        )

        frequencies_of_interest = []
        for i in range(0, len(frequencies_of_interest_parameters), 2):
            frequencies_of_interest += [
                (
                    frequencies_of_interest_parameters[i],
                    frequencies_of_interest_parameters[i + 1],
                )
            ]

        # Initialize communication with Acoustics PCB
        self.get_logger().info("Initializing communication with Acoustics")
        self.get_logger().info("Acoustics PCB MCU IP: 10.0.0.111")
        self.get_logger().info("Trying to connect...")

        TeensyCommunicationUDP.init_communication(frequencies_of_interest)

        self.get_logger().info("Successfully connected to Acoustics PCB MCU :D")

    def get_topics(self) -> None:
        orca_namespace = (
            self.declare_parameter("topics.namespace", "_")
            .get_parameter_value()
            .string_value
        )
        accoustics_namespace = (
            self.declare_parameter("topics.acoustics.namespace", "_")
            .get_parameter_value()
            .string_value
        )
        hydrophone_topics = (
            self.declare_parameter("topics.acoustics.hydrophones", ["_"])
            .get_parameter_value()
            .string_array_value
        )
        for topic in hydrophone_topics:
            setattr(
                self,
                topic + "_topic",
                orca_namespace + accoustics_namespace + "/" + topic,
            )

        topics = ["filter_response", "fft", "peaks", "tdoa", "position"]
        for topic in topics:
            self.declare_parameter(f"topics.acoustics.{topic}", "_")
            setattr(
                self,
                topic + "_topic",
                orca_namespace
                + accoustics_namespace
                + self.get_parameter(f"topics.acoustics.{topic}")
                .get_parameter_value()
                .string_value,
            )

    def create_publishers_and_subscribers(self) -> None:
        self._hydrophone_1_publisher = self.create_publisher(
            Int32MultiArray, self.hydrophone1_topic, 5
        )
        self._hydrophone_2_publisher = self.create_publisher(
            Int32MultiArray, self.hydrophone2_topic, 5
        )
        self._hydrophone_3_publisher = self.create_publisher(
            Int32MultiArray, self.hydrophone3_topic, 5
        )
        self._hydrophone_4_publisher = self.create_publisher(
            Int32MultiArray, self.hydrophone4_topic, 5
        )
        self._hydrophone_5_publisher = self.create_publisher(
            Int32MultiArray, self.hydrophone5_topic, 5
        )

        self._filter_response_publisher = self.create_publisher(
            Int32MultiArray, self.filter_response_topic, 5
        )
        self._fft_publisher = self.create_publisher(Int32MultiArray, self.fft_topic, 5)
        self._peak_publisher = self.create_publisher(
            Int32MultiArray, self.peaks_topic, 5
        )
        self._tdoa_publisher = self.create_publisher(
            Float32MultiArray, self.tdoa_topic, 5
        )
        self._position_publisher = self.create_publisher(
            Float32MultiArray, self.position_topic, 5
        )

    def data_update(self) -> None:
        """Fetches data using the TeensyCommunicationUDP class.

        This method calls the fetch_data method from the TeensyCommunicationUDP class
        to update the data.
        """
        TeensyCommunicationUDP.fetch_data()

    def data_publisher(self) -> None:
        """Publishes to topics."""
        self._hydrophone_1_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["HYDROPHONE_1"])
        )
        self._hydrophone_2_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["HYDROPHONE_2"])
        )
        self._hydrophone_3_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["HYDROPHONE_3"])
        )
        self._hydrophone_4_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["HYDROPHONE_4"])
        )
        self._hydrophone_5_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["HYDROPHONE_5"])
        )

        self._filter_response_publisher.publish(
            Int32MultiArray(
                data=TeensyCommunicationUDP.acoustics_data["SAMPLES_FILTERED"]
            )
        )
        self._fft_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["FFT"])
        )
        self._peak_publisher.publish(
            Int32MultiArray(data=TeensyCommunicationUDP.acoustics_data["PEAK"])
        )

        self._tdoa_publisher.publish(
            Float32MultiArray(data=TeensyCommunicationUDP.acoustics_data["TDOA"])
        )
        self._position_publisher.publish(
            Float32MultiArray(data=TeensyCommunicationUDP.acoustics_data["LOCATION"])
        )


def main(args: list = None) -> None:
    """Entry point for the acoustics interface node.

    This function initializes the ROS 2 Python client library, creates an instance
    of the AcousticsInterfaceNode, and starts spinning the node to process callbacks.
    Once the node is shut down, it destroys the node and shuts down the ROS 2 client library.

    Args:
        args (list, optional): Command line arguments passed to the ROS 2 client library. Defaults to None.

    """
    rclpy.init(args=args)

    node = AcousticsInterfaceNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
