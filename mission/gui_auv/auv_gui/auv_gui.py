# auv_gui.py
# location: vortex-auv/mission/gui_auv/auv_gui/auv_gui/auv_gui.py

import math
import sys
from threading import Thread
from typing import List, Optional
from rclpy.time import Time
from std_msgs.msg import String

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
from geometry_msgs.msg import PoseStamped
from vortex_msgs.msg import LOSGuidance
from vortex_msgs.action import NavigateWaypoints
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension

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

        # Initialize empty waypoints list
        self.waypoints = []
        
        # Subscribe to waypoint list
        self.create_subscription(
            Float32MultiArray,
            '/guidance/waypoint_list',
            self.waypoint_list_callback,
            10
        )

        # Add new subscriptions for guidance
        self.create_subscription(LOSGuidance, '/guidance/los', self.los_callback, 10)
        self.create_subscription(PoseStamped, '/guidance/reference', self.reference_callback, 10)
        self.create_subscription(NavigateWaypoints.Feedback, '/navigate_waypoints/_action/feedback', self.waypoint_feedback_callback, 10)
        
        # Add new data storage variables
        self.los_data = None  # Store latest LOS guidance commands
        self.reference_pose = None  # Store latest reference pose
        self.waypoints = []  # Store list of waypoints
        self.current_waypoint_index = 0  # Store current waypoint index

        # ROS2 parameters
        self.declare_parameter("odom_topic", "/nucleus/odom")
        self.declare_parameter("current_topic", "/auv/power_sense_module/current")
        self.declare_parameter("voltage_topic", "/auv/power_sense_module/voltage")
        self.declare_parameter("temperature_topic", "/auv/temperature")
        self.declare_parameter("pressure_topic", "/auv/pressure")
        self.declare_parameter("history_length", 30)

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        current_topic = self.get_parameter("current_topic").get_parameter_value().string_value
        voltage_topic = self.get_parameter("voltage_topic").get_parameter_value().string_value
        temperature_topic = self.get_parameter("temperature_topic").get_parameter_value().string_value
        pressure_topic = self.get_parameter("pressure_topic").get_parameter_value().string_value

        # Subscriber to the /nucleus/odom topic
        self.subscription = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

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
        self.current_subscriber = self.create_subscription(Float32, current_topic, self.current_callback, 5)
        self.voltage_subscriber = self.create_subscription(Float32, voltage_topic, self.voltage_callback, 5)
        self.temperature_subscriber = self.create_subscription(Float32, temperature_topic, self.temperature_callback, 5)
        self.pressure_subscriber = self.create_subscription(Float32, pressure_topic, self.pressure_callback, 5)

        # Variables for internal status
        self.current = 0.0
        self.voltage = 0.0
        self.temperature = 0.0
        self.pressure = 0.0
    
    # Add these callback methods to the GuiNode class

    def los_callback(self, msg: LOSGuidance) -> None:
        """Callback function for LOS guidance commands."""
        self.los_data = {
        'surge': msg.surge,
        'pitch': msg.pitch,
        'yaw': msg.yaw
    }

    def waypoint_feedback_callback(self, msg: NavigateWaypoints.Feedback) -> None:
        """Callback function for waypoint navigation feedback."""
        self.current_waypoint_index = int(msg.current_waypoint_index)

    # --- Callback functions ---

    def odom_callback(self, msg: Odometry) -> None:
        """Callback function that is triggered when an odometry message is received."""
        # Extract x, y, z positions from the odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
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
        max_data_points = self.get_parameter("history_length").get_parameter_value().integer_value
        if len(self.xpos_data) > max_data_points:
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

    def waypoint_list_callback(self, msg: Float32MultiArray) -> None:
        """Callback for receiving the complete waypoint list."""
        num_waypoints = msg.layout.dim[0].size
        waypoint_data = msg.data
        
        # Clear existing waypoints and add new ones
        self.waypoints = []
        for i in range(num_waypoints):
            wp = np.array([
                waypoint_data[i*3],     # x
                waypoint_data[i*3 + 1], # y
                -waypoint_data[i*3 + 2] # Negate z value
            ])
            self.waypoints.append(wp)
        
        self.get_logger().info(f"Received complete waypoint list. Total waypoints: {len(self.waypoints)}")

    def reference_callback(self, msg: PoseStamped) -> None:
        """Callback function for reference pose."""
        self.reference_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
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

    def update_plot(self, x_data: List[float], y_data: List[float], z_data: List[float]) -> None:
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

class GuidancePlotCanvas(FigureCanvas):
    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        """Initialize the GuidancePlotCanvas."""
        self.gui_node = gui_node
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # Initialize plot elements
        (self.vehicle_path,) = self.ax.plot([], [], [], 'b-', label='Vehicle Path')
        (self.current_position,) = self.ax.plot([], [], [], 'ro', label='Current Position')
        (self.waypoints,) = self.ax.plot([], [], [], 'g^', label='Waypoints')
        # Add this new line for desired path
        (self.desired_path,) = self.ax.plot([], [], [], 'g--', label='Desired Path')
        (self.target_waypoint,) = self.ax.plot([], [], [], 'y*', markersize=15, label='Target Waypoint')
        (self.los_vector,) = self.ax.plot([], [], [], 'r--', label='LOS Vector')

        super().__init__(self.fig)
        self.setParent(parent)

        # Set labels and title
        self.ax.set_xlabel("X [m]")
        self.ax.set_ylabel("Y [m]")
        self.ax.set_zlabel("Depth [m]")
        self.ax.set_title("Guidance Visualization")
        self.ax.legend()

        # Invert the Z axis so positive values go down
        # self.ax.invert_zaxis()

    def update_plot(self) -> None:
        """Update the guidance visualization plot."""
        # Update vehicle path
        if len(self.gui_node.xpos_data) > 0:
            x_data = np.array(self.gui_node.xpos_data)
            y_data = np.array(self.gui_node.ypos_data)
            z_data = np.array(self.gui_node.zpos_data)
            
            self.vehicle_path.set_data(x_data, y_data)
            self.vehicle_path.set_3d_properties(z_data)
            
            # Update current position
            self.current_position.set_data([x_data[-1]], [y_data[-1]])
            self.current_position.set_3d_properties([z_data[-1]])

            # Update waypoints visualization
            if hasattr(self.gui_node, 'waypoints') and len(self.gui_node.waypoints) > 0:
                waypoints = np.array(self.gui_node.waypoints)
                
                # Plot all waypoints
                self.waypoints.set_data(waypoints[:, 0], waypoints[:, 1])
                self.waypoints.set_3d_properties(waypoints[:, 2])

                # Add this section to plot the desired path
                self.desired_path.set_data(waypoints[:, 0], waypoints[:, 1])
                self.desired_path.set_3d_properties(waypoints[:, 2])
                
                # Plot current target waypoint
                if self.gui_node.current_waypoint_index < len(self.gui_node.waypoints):
                    target = self.gui_node.waypoints[self.gui_node.current_waypoint_index]
                    self.target_waypoint.set_data([target[0]], [target[1]])
                    self.target_waypoint.set_3d_properties([target[2]])
                    
                    # Update LOS vector from current position to target
                    self.los_vector.set_data([x_data[-1], target[0]], [y_data[-1], target[1]])
                    self.los_vector.set_3d_properties([z_data[-1], target[2]])
                    
                    # Make sure the plot shows all waypoints
                    self._update_plot_limits(x_data, y_data, z_data)

                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

    def _update_plot_limits(self, x_data: np.ndarray, y_data: np.ndarray, z_data: np.ndarray) -> None:
        """Update the plot limits to keep the vehicle and waypoints in view."""
        margin = 2.5
        if len(self.gui_node.waypoints) > 0:
            waypoints = np.array(self.gui_node.waypoints)
            x_min = min(np.min(x_data), np.min(waypoints[:, 0])) - margin
            x_max = max(np.max(x_data), np.max(waypoints[:, 0])) + margin
            y_min = min(np.min(y_data), np.min(waypoints[:, 1])) - margin
            y_max = max(np.max(y_data), np.max(waypoints[:, 1])) + margin
            z_min = min(np.min(z_data), np.min(waypoints[:, 2])) - margin
            z_max = max(np.max(z_data), np.max(waypoints[:, 2])) + margin
        else:
            x_min, x_max = np.min(x_data) - margin, np.max(x_data) + margin
            y_min, y_max = np.min(y_data) - margin, np.max(y_data) + margin
            z_min, z_max = np.min(z_data) - margin, np.max(z_data) + margin

        self.ax.set_xlim(x_min, x_max)
        self.ax.set_ylim(y_min, y_max)
        self.ax.set_zlim(z_min, z_max)


class GuidanceInfoPanel(QWidget):
    """Widget displaying guidance-related information."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.gui_node = gui_node
        
        # Create layout
        layout = QVBoxLayout(self)
        
        # Create labels for different information sections
        self.waypoint_info = QLabel("Waypoint Information:")
        self.guidance_commands = QLabel("Guidance Commands:")
        self.navigation_status = QLabel("Navigation Status:")
        
        # Add labels to layout
        layout.addWidget(self.waypoint_info)
        layout.addWidget(self.guidance_commands)
        layout.addWidget(self.navigation_status)
        
        # Add stretch to keep widgets at the top
        layout.addStretch()

class GuidanceInfoPanel(QWidget):
    """Widget displaying guidance-related information."""

    def __init__(self, gui_node: GuiNode, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.gui_node = gui_node
        
        # Create layout
        layout = QVBoxLayout(self)
        
        # Create labels for different information sections
        self.waypoint_info = QLabel("Waypoint Information:")
        self.current_position_info = QLabel("Current Position:")  # New label
        self.guidance_commands = QLabel("Guidance Commands:")
        self.navigation_status = QLabel("Navigation Status:")
        
        # Add labels to layout
        layout.addWidget(self.waypoint_info)
        layout.addWidget(self.current_position_info)  # Add new label
        layout.addWidget(self.guidance_commands)
        layout.addWidget(self.navigation_status)
        
        # Add stretch to keep widgets at the top
        layout.addStretch()

    def update_info(self) -> None:
        """Update the displayed information."""
        # Update waypoint information
        wp_text = "Waypoint Information:\n"
        if hasattr(self.gui_node, 'waypoints') and len(self.gui_node.waypoints) > 0:
            current_idx = self.gui_node.current_waypoint_index
            total_waypoints = len(self.gui_node.waypoints)
            wp_text += f"Current Waypoint: {current_idx + 1}/{total_waypoints}\n"
            if current_idx < total_waypoints:
                wp = self.gui_node.waypoints[current_idx]
                wp_text += f"Target Position: [{wp[0]:.2f}, {wp[1]:.2f}, {wp[2]:.2f}]"
        else:
            wp_text += "No waypoints available"
        self.waypoint_info.setText(wp_text)

        # Update current position information
        pos_text = "Current Position:\n"
        if len(self.gui_node.xpos_data) > 0:
            current_x = self.gui_node.xpos_data[-1]
            current_y = self.gui_node.ypos_data[-1]
            current_z = self.gui_node.zpos_data[-1]
            pos_text += f"X: {current_x:.2f} m\n"
            pos_text += f"Y: {current_y:.2f} m\n"
            pos_text += f"Z: {current_z:.2f} m\n"
            if self.gui_node.roll is not None:
                pos_text += f"Roll: {self.gui_node.roll:.2f}°\n"
                pos_text += f"Pitch: {self.gui_node.pitch:.2f}°\n"
                pos_text += f"Yaw: {self.gui_node.yaw:.2f}°"
        else:
            pos_text += "No position data available"
        self.current_position_info.setText(pos_text)

        # Update guidance commands
        cmd_text = "Guidance Commands:\n"
        if self.gui_node.los_data is not None:
            cmd_text += f"Surge: {self.gui_node.los_data['surge']:.2f} m/s\n"
            cmd_text += f"Pitch: {self.gui_node.los_data['pitch']:.2f} rad\n"
            cmd_text += f"Yaw: {self.gui_node.los_data['yaw']:.2f} rad"
        else:
            cmd_text += "No guidance commands available"
        self.guidance_commands.setText(cmd_text)

        # Update navigation status with more details
        status_text = "Navigation Status:\n"
        if len(self.gui_node.xpos_data) > 0 and self.gui_node.current_waypoint_index < len(self.gui_node.waypoints):
            current_pos = np.array([self.gui_node.xpos_data[-1], 
                                  self.gui_node.ypos_data[-1], 
                                  self.gui_node.zpos_data[-1]])
            target_pos = self.gui_node.waypoints[self.gui_node.current_waypoint_index]
            distance = np.linalg.norm(current_pos - target_pos)
            
            # Calculate individual axis errors
            x_error = target_pos[0] - current_pos[0]
            y_error = target_pos[1] - current_pos[1]
            z_error = target_pos[2] - current_pos[2]
            
            status_text += f"Distance to target: {distance:.2f} m\n"
            status_text += f"Position errors:\n"
            status_text += f"  X error: {x_error:.2f} m\n"
            status_text += f"  Y error: {y_error:.2f} m\n"
            status_text += f"  Z error: {z_error:.2f} m"
        else:
            status_text += "No navigation status available"
        self.navigation_status.setText(status_text)

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

    # Add the new Guidance Tab here:
    # --- Guidance Tab ---
    guidance_widget = QWidget()
    guidance_layout = QGridLayout(guidance_widget)

    # Add guidance plot
    guidance_plot = GuidancePlotCanvas(ros_node, guidance_widget)
    guidance_layout.addWidget(guidance_plot, 0, 0)

    # Add guidance information panel
    guidance_info = GuidanceInfoPanel(ros_node, guidance_widget)
    guidance_layout.addWidget(guidance_info, 0, 1)

    tabs.addTab(guidance_widget, "Guidance")

    # Update your existing update_gui function
    def update_gui() -> None:
        # Existing updates
        plot_canvas.update_plot(ros_node.xpos_data, ros_node.ypos_data, ros_node.zpos_data)
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

        # Add these lines for guidance updates
        guidance_plot.update_plot()
        guidance_info.update_info()

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
