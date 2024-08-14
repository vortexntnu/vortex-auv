#!/usr/bin/env python3

# ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
from ament_index_python.packages import get_package_share_directory

# Python libraries
import array

# Custom libraries
from acoustics_data_record_lib import AcousticsDataRecordLib



class AcousticsDataRecordNode(Node):
    def __init__(self):
        # Variables for seting upp loging correctly
        hydrophoneDataSize = (2**10) * 3 # 1 hydrophone buffer is 2^10 long, Each hydrophone data has 3 buffers full of this data
        DSPDataSize = 2**10 # DSP (Digital Signal Processing) has 2^10 long data
        TDOADataSize = 5 # TDOA (Time Difference Of Arrival) has 5 hydrophones it has times for
        positionDataSize = 3 # position only has X, Y, Z basicaly 3 elements

        # Initialize ROS2 node
        super().__init__('acoustics_data_record_node')

        # Initialize Subscribers ----------
        # Start listening to Hydrophone data
        self.subscriberHydrophone1 = self.create_subscription(
            Int32MultiArray,
            '/acoustics/hydrophone1',
            self.hydrophone1_callback,
            5)
        self.hydropone1Data = array.array("i", [0] * hydrophoneDataSize)

        self.subscriberHydrophone2 = self.create_subscription(
            Int32MultiArray,
            '/acoustics/hydrophone2',
            self.hydrophone2_callback,
            5)
        self.hydropone2Data = array.array("i", [0] * hydrophoneDataSize)

        self.subscriberHydrophone3 = self.create_subscription(
            Int32MultiArray,
            '/acoustics/hydrophone3',
            self.hydrophone3_callback,
            5)
        self.hydropone3Data = array.array("i", [0] * hydrophoneDataSize)

        self.subscriberHydrophone4 = self.create_subscription(
            Int32MultiArray,
            '/acoustics/hydrophone4',
            self.hydrophone4_callback,
            5)
        self.hydropone4Data = array.array("i", [0] * hydrophoneDataSize)

        self.subscriberHydrophone5 = self.create_subscription(
            Int32MultiArray,
            '/acoustics/hydrophone5',
            self.hydrophone5_callback,
            5)
        self.hydropone5Data = array.array("i", [0] * hydrophoneDataSize)

        # Start listening to DSP (Digital Signal Processing) data
        self.subscriberFilterResponse = self.create_subscription(
            Int32MultiArray,
            '/acoustics/filter_response',
            self.filter_response_callback,
            5)
        self.filterResponseData = array.array("i", [0] * DSPDataSize)

        self.subscriberFFT = self.create_subscription(
            Int32MultiArray,
            '/acoustics/fft',
            self.fft_callback,
            5)
        self.FFTData = array.array("i", [0] * DSPDataSize)

        self.subscriberPeaks = self.create_subscription(
            Int32MultiArray,
            '/acoustics/peaks',
            self.peaks_callback,
            5)
        self.peaksData = array.array("i", [0] * DSPDataSize)

        # Start listening to Multilateration data
        self.subscriberTDOAResponse = self.create_subscription(
            Float32MultiArray,
            '/acoustics/time_difference_of_arrival',
            self.tdoa_callback,
            5)
        self.TDOAData = array.array("f", [0.0] * TDOADataSize)

        self.subscriberPositionResponse = self.create_subscription(
            Float32MultiArray,
            '/acoustics/position',
            self.position_callback,
            5)
        self.positionData = array.array("f", [0.0] * positionDataSize)

        # Initialize logger ----------
        # Get package directory location
        ros2_package_directory_location = get_package_share_directory("acoustics_data_record")
        ros2_package_directory_location = ros2_package_directory_location + "/../../../../" # go back to workspace
        ros2_package_directory_location = ros2_package_directory_location + "src/vortex-asv/acoustics/acoustics_data_record/" # Navigate to this package
        
        # Make blackbox loging file
        self.acoustics_data_record = AcousticsDataRecordLib(
            ROS2_PACKAGE_DIRECTORY = ros2_package_directory_location
        )

        # Logs all the newest data 1 time(s) per second
        self.declare_parameter("acoustics.data_logging_rate", 1.0)  # Providing a default value 1.0 => 1 samplings per second, verry slow
        DATA_LOGING_RATE = self.get_parameter("acoustics.data_logging_rate").get_parameter_value().double_value
        timer_period = 1.0/DATA_LOGING_RATE
        self.logger_timer = self.create_timer(timer_period, self.logger)

        # Debuging ----------
        self.get_logger().info(
            "Started logging data for topics: \n"
            "/acoustics/hydrophone1 [Int32MultiArray] \n"
            "/acoustics/hydrophone2 [Int32MultiArray] \n"
            "/acoustics/hydrophone3 [Int32MultiArray] \n"
            "/acoustics/hydrophone4 [Int32MultiArray] \n"
            "/acoustics/hydrophone5 [Int32MultiArray] \n"
            "/acoustics/filter_response [Int32MultiArray] \n"
            "/acoustics/fft [Int32MultiArray] \n"
            "/acoustics/peaks [Int32MultiArray] \n"
            "/acoustics/time_difference_of_arrival [Float32MultiArray] \n"
            "/acoustics/position [Float32MultiArray] \n"
        )

    # Callback methods for diffrenet topics
    def hydrophone1_callback(self, msg):
        self.hydropone1Data = msg.data
    
    def hydrophone2_callback(self, msg):
        self.hydropone2Data = msg.data

    def hydrophone3_callback(self, msg):
        self.hydropone3Data = msg.data

    def hydrophone4_callback(self, msg):
        self.hydropone4Data = msg.data

    def hydrophone5_callback(self, msg):
        self.hydropone5Data = msg.data

    def filter_response_callback(self, msg):
        self.filterResponseData = msg.data

    def fft_callback(self, msg):
        self.FFTData = msg.data

    def peaks_callback(self, msg):
        self.peaksData = msg.data

    def tdoa_callback(self, msg):
        self.TDOAData = msg.data

    def position_callback(self, msg):
        self.positionData = msg.data
    
    # The logger that logs all the data
    def logger(self):
        self.acoustics_data_record.log_data_to_csv_file(
            hydrophone1=self.hydropone1Data,
            hydrophone2=self.hydropone2Data,
            hydrophone3=self.hydropone3Data,
            hydrophone4=self.hydropone4Data,
            hydrophone5=self.hydropone5Data,

            filter_response=self.filterResponseData,
            fft=self.FFTData,
            peaks=self.peaksData,

            tdoa=self.TDOAData,
            position=self.positionData,
        )



def main():
    # Initialize ROS2
    rclpy.init()

    # Start the ROS2 node and continue forever until exit
    acoustics_data_record_node = AcousticsDataRecordNode()
    rclpy.spin(acoustics_data_record_node)

    # Destroy the node explicitly once ROS2 stops runing
    acoustics_data_record_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
