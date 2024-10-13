import sys
from threading import Thread
from typing import List, Optional

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from nav_msgs.msg import Odometry
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class GuiNode(Node):
    """ROS2 Node that subscribes to odometry data and stores x, y positions."""

    def __init__(self) -> None:
        """Initialize the GuiNode and set up the odometry subscriber."""
        super().__init__("simple_gui_node")

        # Subscriber to the /nucleus/odom topic
        self.subscription = self.create_subscription(Odometry, '/nucleus/odom', self.odom_callback, 10)

        # Variables to store odometry data
        self.x_data: List[float] = []
        self.y_data: List[float] = []
        self.z_data: List[float] = []

        self.counter_ = 0
        self.get_logger().info("Subscribed to /nucleus/odom")

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x and y positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = -(msg.pose.pose.position.z)
        self.x_data.append(x)
        self.y_data.append(y)
        self.z_data.append(z)

        # Limit the stored data for real-time plotting (avoid memory overflow)
        # Store 30 seconds worth of data at 100Hz
        max_data_points = 30 * 100  # 30 seconds * 100 Hz
        if len(self.x_data) > max_data_points:
            self.x_data.pop(0)
            self.y_data.pop(0)
            self.z_data.pop(0)


class PlotCanvas(FigureCanvas):
    """A canvas widget for plotting odometry data using matplotlib."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """Initialize the PlotCanvas with a reference to the ROS node and configure the plot."""
        self.gui_node = gui_node  # Store a reference to the ROS node
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')  # Set up 3D projection
        super().__init__(self.fig)
        self.setParent(parent)

        # Set labels and title for the 3D plot
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Odometry")

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

            # Update the limits for the 3D plot around the latest data point
            x_latest = x_data[-1]
            y_latest = y_data[-1]
            z_latest = z_data[-1]
            margin = 1  # Define a margin around the latest point

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
    gui.setWindowTitle("AUV Odometry GUI")
    gui.setGeometry(100, 100, 600, 400)

    # Create a central widget and layout for the GUI
    central_widget = QWidget()
    layout = QVBoxLayout(central_widget)

    plot_canvas = PlotCanvas(ros_node, central_widget)
    layout.addWidget(plot_canvas)

    # Add buttons or other GUI elements if needed (optional)
    button1 = QPushButton("Send a command", parent=central_widget)
    button1.clicked.connect(lambda: ros_node.get_logger().info("Command sent"))
    layout.addWidget(button1)

    gui.setCentralWidget(central_widget)
    gui.show()

    # Use a QTimer to update the plot in the main thread
    def update_plot() -> None:
        plot_canvas.update_plot(ros_node.x_data, ros_node.y_data, ros_node.z_data)

    # Set up the timer to call update_plot every 100ms
    timer = QTimer()
    timer.timeout.connect(update_plot)
    timer.start(100)  # 100 ms interval

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
