import sys
from threading import Thread
from typing import List, Optional

import matplotlib.pyplot as plt
import rclpy
from matplotlib.backend_bases import MouseEvent
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

        self.counter_ = 0
        self.get_logger().info("Subscribed to /nucleus/odom")

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x and y positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.x_data.append(x)
        self.y_data.append(y)

        # Limit the stored data for real-time plotting (avoid memory overflow)
        # Store 30 seconds worth of data at 100Hz
        max_data_points = 30 * 100  # 30 seconds * 100 Hz
        if len(self.x_data) > max_data_points:
            self.x_data.pop(0)
            self.y_data.pop(0)


class PlotCanvas(FigureCanvas):
    """A canvas widget for plotting odometry data using matplotlib."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """
        Initialize the PlotCanvas with a reference to the ROS node and configure the plot.

        Args:
            gui_node (GuiNode): A reference to the ROS node.
            parent (Optional[QWidget]): The parent widget (default: None).
        """
        self.gui_node = gui_node  # Store a reference to the ROS node
        self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)
        self.setParent(parent)

        self.ax.set_xlabel("X Position")
        self.ax.set_ylabel("Y Position")
        self.ax.set_title("Odometry")

        self.x_data: List[float] = []
        self.y_data: List[float] = []
        (self.line,) = self.ax.plot([], [], 'b-')

        # Mouse click event handler
        self.mpl_connect("button_press_event", self.on_click)

    def on_click(self, event: MouseEvent) -> None:
        """
        Handle mouse click event on the plot and log the clicked coordinates.

        Args:
            event: The mouse click event from matplotlib.
        """
        if event.inaxes is not None:
            x_click = event.xdata
            y_click = event.ydata
            # Use the logger from the ROS node to log the click event
            self.gui_node.get_logger().info(f"Clicked at: x={x_click}, y={y_click}")

    def update_plot(self, x_data: List[float], y_data: List[float]) -> None:
        """
        Update the plot with the latest odometry data.

        Args:
            x_data (List[float]): The list of x positions.
            y_data (List[float]): The list of y positions.
        """
        self.line.set_data(x_data, y_data)  # Update the data of the line object

        if x_data and y_data:
            x_latest = x_data[-1]
            y_latest = y_data[-1]
            margin = 1  # Define a margin around the latest point

            self.ax.set_xlim(x_latest - margin, x_latest + margin)
            self.ax.set_ylim(y_latest - margin, y_latest + margin)

        self.draw()


def run_ros_node(ros_node: GuiNode, executor: MultiThreadedExecutor) -> None:
    """
    Run the ROS2 node in a separate thread using a MultiThreadedExecutor.

    Args:
        ros_node (GuiNode): The ROS2 node to run.
        executor (MultiThreadedExecutor): The executor to use for spinning the node.
    """
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
    button1 = QPushButton("Send command", parent=central_widget)
    button1.clicked.connect(lambda: ros_node.get_logger().info("Command sent"))
    layout.addWidget(button1)

    gui.setCentralWidget(central_widget)
    gui.show()

    # Use a QTimer to update the plot in the main thread
    def update_plot() -> None:
        plot_canvas.update_plot(ros_node.x_data, ros_node.y_data)

    # Set up the timer to call update_plot every 100ms
    timer = QTimer()
    timer.timeout.connect(update_plot)
    timer.start(100)  # 100 ms interval

    sys.exit(app.exec())


if __name__ == '__main__':
    main()
