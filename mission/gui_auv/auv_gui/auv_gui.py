import math
import sys
from threading import Thread
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
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
    """
    Convert a quaternion to Euler angles (roll, pitch, yaw).

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

        # Subscriber to the /nucleus/odom topic
        self.subscription = self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

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
        self.internal_status_subscriber = self.create_subscription(Float32, "/auv/power_sense_module/current", self.current_callback, 5)
        self.internal_status_subscriber = self.create_subscription(Float32, "/auv/power_sense_module/voltage", self.voltage_callback, 5)

        # Variables for internal status
        self.current = 0.0
        self.voltage = 0.0

    # --- Callback functions ---

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x, y, z positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = -(msg.pose.pose.position.z)  # Inverted to match reality...
        self.xpos_data.append(x)
        self.ypos_data.append(y)
        self.zpos_data.append(z)

        # Extract the quaternion components from the odometry message
        w = msg.pose.pose.orientation.w
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        self.roll = quaternion_to_euler(x, y, z, w)[0]
        self.pitch = quaternion_to_euler(x, y, z, w)[1]
        self.yaw = quaternion_to_euler(x, y, z, w)[2]

        # Limit the stored data for real-time plotting (avoid memory overflow)
        max_data_points = 30 * 100  # 30 seconds * 100 Hz
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


# --- Plotting ---


class PlotCanvas(FigureCanvas):
    """A canvas widget for plotting odometry data using matplotlib."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """Initialize the PlotCanvas with a reference to the ROS node and configure the plot."""
        # Set up the 3D plot
        self.gui_node = gui_node  # Store a reference to the ROS node
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  # Set up 3D projection

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
        (self.line,) = self.ax.plot([], [], [], 'b-')  # 3D line plot

    def update_plot(self, x_data: List[float], y_data: List[float], z_data: List[float]) -> None:
        """
        Update the 3D plot with the latest odometry data.

        Args:
            x_data (List[float]): The list of x positions.
            y_data (List[float]): The list of y positions.
            z_data (List[float]): The list of z positions.
        """
        # Convert lists to numpy arrays to ensure compatibility with the plot functions
        x_data = np.array(x_data, dtype=float)
        y_data = np.array(y_data, dtype=float)
        z_data = np.array(z_data, dtype=float)

        # Check if the arrays are non-empty before updating the plot
        if len(x_data) > 0 and len(y_data) > 0 and len(z_data) > 0:
            self.line.set_data(x_data, y_data)  # Update the 2D projection data
            self.line.set_3d_properties(z_data)  # Update the z-data for the 3D line

            # Update the current position dot
            self.current_position_dot.set_data(x_data[-1:], y_data[-1:])  # Update the 2D projection data
            self.current_position_dot.set_3d_properties(z_data[-1:])  # Update the z-data for the 3D dot

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
    tabs.setTabPosition(QTabWidget.North)
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
    gui.show()

    # Use a QTimer to update plot, current position, and internal status in the main thread
    def update_gui() -> None:
        plot_canvas.update_plot(ros_node.xpos_data, ros_node.ypos_data, ros_node.zpos_data)
        if len(ros_node.xpos_data) > 0 and ros_node.roll is not None:
            position_text = f"Current Position:\nX: {ros_node.xpos_data[-1]:.2f}\nY: {ros_node.ypos_data[-1]:.2f}\nZ: {ros_node.zpos_data[-1]:.2f}"
            orientation_text = f"Current Orientation:\nRoll: {ros_node.roll:.2f}\nPitch: {ros_node.pitch:.2f}\nYaw: {ros_node.yaw:.2f}"
            current_pos.setText(position_text + "\n\n\n" + orientation_text)

        # Update internal status
        internal_status_label.setText(f"Internal Status:\nCurrent: {ros_node.current:.2f}\nVoltage: {ros_node.voltage:.2f}")

    # Set up the timer to call update_gui every 100ms
    timer = QTimer()
    timer.timeout.connect(update_gui)
    timer.start(100)  # 100 ms interval

    app.exec()

    # Cleanup ROS resources
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()
