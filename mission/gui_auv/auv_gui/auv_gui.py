import math
import sys
from threading import Thread
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from nav_msgs.msg import Odometry
from PyQt6.QtCore import QTimer
from PyQt6.QtWidgets import (
    QApplication,
    QGridLayout,
    QLabel,
    QMainWindow,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32

# --- Quaternion to Euler angles ---


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> List[float]:
    """Convert a quaternion to Euler angles (roll, pitch, yaw).

    Args:
        x (float): The x component of the quaternion.
        y (float): The y component of the quaternion.
        z (float): The z component of the quaternion.
        w (float): The w component of the quaternion.

    Returns:
        List[float]: A list of Euler angles [roll, pitch, yaw].
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.degrees(math.asin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.degrees(math.atan2(siny_cosp, cosy_cosp))

    return [roll, pitch, yaw]


# --- GUI Node ---


class GuiNode(Node):
    """ROS2 Node that subscribes to odometry data and stores x, y positions."""

    def __init__(self) -> None:
        """Initialize the GuiNode and set up the odometry subscriber."""
        super().__init__("auv_gui_node")

        # ROS2 parameters
        self.declare_parameter("odom_topic", "/nucleus/odom")
        self.declare_parameter("current_topic", "/auv/power_sense_module/current")
        self.declare_parameter("voltage_topic", "/auv/power_sense_module/voltage")
        self.declare_parameter("temperature_topic", "/auv/temperature")
        self.declare_parameter("pressure_topic", "/auv/pressure")
        self.declare_parameter("history_length", 30)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        current_topic = (
            self.get_parameter("current_topic").get_parameter_value().string_value
        )
        voltage_topic = (
            self.get_parameter("voltage_topic").get_parameter_value().string_value
        )
        temperature_topic = (
            self.get_parameter("temperature_topic").get_parameter_value().string_value
        )
        pressure_topic = (
            self.get_parameter("pressure_topic").get_parameter_value().string_value
        )

        # Subscriber to the /nucleus/odom topic
        self.subscription = self.create_subscription(
            Odometry, odom_topic, self.odom_callback, 10
        )

        # Variables to store odometry data
        self.xpos_data: List[float] = []  # x position
        self.ypos_data: List[float] = []  # y position
        self.zpos_data: List[float] = []  # z position

        self.w_data: List[float] = []  # w component of the quaternion
        self.x_data: List[float] = []  # x component of the quaternion
        self.y_data: List[float] = []  # y component of the quaternion
        self.z_data: List[float] = []  # z component of the quaternion

        self.roll: Optional[float] = None
        self.pitch: Optional[float] = None
        self.yaw: Optional[float] = None

        # Subscribe to internal status topics
        self.current_subscriber = self.create_subscription(
            Float32, current_topic, self.current_callback, 5
        )
        self.voltage_subscriber = self.create_subscription(
            Float32, voltage_topic, self.voltage_callback, 5
        )
        self.temperature_subscriber = self.create_subscription(
            Float32, temperature_topic, self.temperature_callback, 5
        )
        self.pressure_subscriber = self.create_subscription(
            Float32, pressure_topic, self.pressure_callback, 5
        )

        # Variables for internal status
        self.current = 0.0
        self.voltage = 0.0
        self.temperature = 0.0
        self.pressure = 0.0

    # --- Callback functions ---

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x, y, z positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = -(msg.pose.pose.position.z)
        self.xpos_data.append(x)
        self.ypos_data.append(y)
        self.zpos_data.append(z)

        # Extract the quaternion components from the odometry message
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        self.roll, self.pitch, self.yaw = quaternion_to_euler(x, y, z, w)

        # Limit the stored data for real-time plotting (avoid memory overflow)
        max_data_points = (
            self.get_parameter("history_length").get_parameter_value().integer_value
        )
        if len(self.x_data) > max_data_points:
            self.xpos_data.pop(0)
            self.ypos_data.pop(0)
            self.zpos_data.pop(0)

    def current_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a current message is received."""
        self.current = msg.data

    def voltage_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a voltage message is received."""
        self.voltage = msg.data

    def temperature_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a temperature message is received."""
        self.temperature = msg.data

    def pressure_callback(self, msg: Float32) -> None:
        """Callback function that is triggered when a pressure message is received."""
        self.pressure = msg.data


# --- Plotting ---


class PlotCanvas(FigureCanvas):
    """A canvas widget for plotting odometry data using matplotlib."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """Initialize the PlotCanvas."""
        # Set up the 3D plot
        self.gui_node = gui_node
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Initialize a red dot for the current position
        (self.current_position_dot,) = self.ax.plot([], [], [], 'ro')

        super().__init__(self.fig)
        self.setParent(parent)

        # Set labels and title for the 3D plot
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Position")

        # Initialize data lists for 3D plot
        self.x_data: List[float] = []
        self.y_data: List[float] = []
        self.z_data: List[float] = []
        (self.line,) = self.ax.plot([], [], [], 'b-')

    def update_plot(
        self, x_data: List[float], y_data: List[float], z_data: List[float]
    ) -> None:
        """Update the 3D plot with the latest odometry data."""
        # Convert lists to numpy arrays to ensure compatibility with the plot functions
        x_data = np.array(x_data, dtype=float)
        y_data = np.array(y_data, dtype=float)
        z_data = np.array(z_data, dtype=float)

        # Check if the arrays are non-empty before updating the plot
        if len(x_data) > 0 and len(y_data) > 0 and len(z_data) > 0:
            self.line.set_data(x_data, y_data)
            self.line.set_3d_properties(z_data)

            # Update the current position dot
            self.current_position_dot.set_data(x_data[-1:], y_data[-1:])
            self.current_position_dot.set_3d_properties(z_data[-1:])

            # Update the limits for the 3D plot around the latest data point
            x_latest = x_data[-1]
            y_latest = y_data[-1]
            z_latest = z_data[-1]
            margin = 2.5  # Define a margin around the latest point

            self.ax.set_xlim(x_latest - margin, x_latest + margin)
            self.ax.set_ylim(y_latest - margin, y_latest + margin)
            self.ax.set_zlim(z_latest - margin, z_latest + margin)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()


def run_ros_node(ros_node: GuiNode, executor: MultiThreadedExecutor) -> None:
    """Run the ROS2 node in a separate thread using a MultiThreadedExecutor."""
    rclpy.spin(ros_node, executor)


def main(args: Optional[List[str]] = None) -> None:
    """The main function to initialize ROS2 and the GUI application."""
    rclpy.init(args=args)
    ros_node = GuiNode()
    executor = MultiThreadedExecutor()

    # Run ROS in a separate thread
    ros_thread = Thread(target=run_ros_node, args=(ros_node, executor), daemon=True)
    ros_thread.start()

    # Setup the PyQt5 application and window
    app = QApplication(sys.argv)
    gui = QMainWindow()
    gui.setWindowTitle("Vortex GUI")
    gui.setGeometry(100, 100, 600, 400)

    # Create the tab widget
    tabs = QTabWidget()
    tabs.setTabPosition(QTabWidget.TabPosition.North)
    tabs.setMovable(True)

    # --- Position Tab ---
    position_widget = QWidget()
    layout = QGridLayout(position_widget)  # grid layout

    plot_canvas = PlotCanvas(ros_node, position_widget)
    layout.addWidget(plot_canvas, 0, 0)

    current_pos = QLabel(parent=position_widget)
    layout.addWidget(current_pos, 0, 1)

    tabs.addTab(position_widget, "Position")

    # --- Internal Status Tab ---
    internal_widget = QWidget()
    internal_layout = QVBoxLayout(internal_widget)

    internal_status_label = QLabel(parent=internal_widget)
    internal_layout.addWidget(internal_status_label)
    tabs.addTab(internal_widget, "Internal")

    gui.setCentralWidget(tabs)
    gui.showMaximized()

    # Use a QTimer to update plot, current position, and internal status in the main thread
    def update_gui() -> None:
        plot_canvas.update_plot(
            ros_node.xpos_data, ros_node.ypos_data, ros_node.zpos_data
        )
        if len(ros_node.xpos_data) > 0 and ros_node.roll is not None:
            position_text = f"Current Position:\nX: {ros_node.xpos_data[-1]:.2f}\nY: {ros_node.ypos_data[-1]:.2f}\nZ: {ros_node.zpos_data[-1]:.2f}"
            orientation_text = f"Current Orientation:\nRoll: {ros_node.roll:.2f}\nPitch: {ros_node.pitch:.2f}\nYaw: {ros_node.yaw:.2f}"
            current_pos.setText(position_text + "\n\n\n" + orientation_text)

        # Update internal status
        internal_status_label.setText(
            f"Internal Status:\n"
            f"Current: {ros_node.current:.2f}\n"
            f"Voltage: {ros_node.voltage:.2f}\n"
            f"Temperature: {ros_node.temperature:.2f}\n"
            f"Pressure: {ros_node.pressure:.2f}"
        )

    # Set up the timer to call update_gui every 100ms
    timer = QTimer()
    timer.timeout.connect(update_gui)
    timer.start(100)

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()
