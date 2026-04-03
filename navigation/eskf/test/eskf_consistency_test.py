import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation

# --- IMPORT YOUR EXISTING QOS FUNCTION ---
try:
    from vortex.utils.qos_profiles import sensor_data_profile
except ImportError:
    # Fallback for testing without the vortex library
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    def sensor_data_profile(depth=5):
        return QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=depth)

class EskfValidator(Node):
    def __init__(self):
        super().__init__('eskf_validator')

        self.est_topic = '/nautilus/odom/eskf'
        self.gt_topic = '/nautilus/odom'

        # --- Publishers (Foxglove Visualizers) ---
        # Overall 3D RMSE
        self.pub_rmse_pos_total = self.create_publisher(Float64, 'eskf_metrics/rmse/position/total', 10)
        self.pub_rmse_vel_total = self.create_publisher(Float64, 'eskf_metrics/rmse/velocity/total', 10)
        self.pub_rmse_ori = self.create_publisher(Float64, 'eskf_metrics/rmse/orientation_rad', 10)

        # Axis-Specific Position RMSE
        self.pub_rmse_pos_x = self.create_publisher(Float64, 'eskf_metrics/rmse/position/x', 10)
        self.pub_rmse_pos_y = self.create_publisher(Float64, 'eskf_metrics/rmse/position/y', 10)
        self.pub_rmse_pos_z = self.create_publisher(Float64, 'eskf_metrics/rmse/position/z', 10)

        # Axis-Specific Velocity RMSE
        self.pub_rmse_vel_x = self.create_publisher(Float64, 'eskf_metrics/rmse/velocity/x', 10)
        self.pub_rmse_vel_y = self.create_publisher(Float64, 'eskf_metrics/rmse/velocity/y', 10)
        self.pub_rmse_vel_z = self.create_publisher(Float64, 'eskf_metrics/rmse/velocity/z', 10)

        # Euler Angles & NEES
        self.pub_euler_est = self.create_publisher(Float64MultiArray, 'debug/euler/est', 10)
        self.pub_euler_gt = self.create_publisher(Float64MultiArray, 'debug/euler/gt', 10)
        self.pub_nees_pos = self.create_publisher(Float64, 'eskf_metrics/nees/position', 10)
        self.pub_nees_ori = self.create_publisher(Float64, 'eskf_metrics/nees/orientation', 10)

        # --- State for Running RMSE ---
        self.n_samples = 0
        
        # NumPy arrays make axis-by-axis tracking incredibly easy
        self.sse_pos_xyz = np.array([0.0, 0.0, 0.0])
        self.sse_vel_xyz = np.array([0.0, 0.0, 0.0])
        
        # Total Euclidean SSE
        self.sse_pos_total = 0.0 
        self.sse_vel_total = 0.0
        self.sse_ori = 0.0

        # --- Subscribers ---
        self.get_logger().info(f"Subscribing to {self.est_topic} and {self.gt_topic}...")
        qos = sensor_data_profile(depth=5)
        
        self.sub_est = message_filters.Subscriber(self, Odometry, self.est_topic, qos_profile=qos)
        self.sub_gt = message_filters.Subscriber(self, Odometry, self.gt_topic, qos_profile=qos)

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_est, self.sub_gt], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.callback)

    def callback(self, est_msg, gt_msg):
        self.n_samples += 1
        
        # ===========================
        # 1. POSITION (Euclidean & Axis)
        # ===========================
        p_est = np.array([est_msg.pose.pose.position.x, est_msg.pose.pose.position.y, est_msg.pose.pose.position.z])
        p_gt = np.array([gt_msg.pose.pose.position.x, gt_msg.pose.pose.position.y, gt_msg.pose.pose.position.z])
        
        err_pos_vec = p_est - p_gt
        
        # Update Axis-Specific RMSE (Squares elements individually)
        self.sse_pos_xyz += err_pos_vec**2
        rmse_pos_xyz = np.sqrt(self.sse_pos_xyz / self.n_samples)
        
        self.publish_float(self.pub_rmse_pos_x, rmse_pos_xyz[0])
        self.publish_float(self.pub_rmse_pos_y, rmse_pos_xyz[1])
        self.publish_float(self.pub_rmse_pos_z, rmse_pos_xyz[2])

        # Update Total 3D RMSE
        self.sse_pos_total += np.linalg.norm(err_pos_vec)**2
        self.publish_float(self.pub_rmse_pos_total, np.sqrt(self.sse_pos_total / self.n_samples))

        # ===========================
        # 2. ORIENTATION (Quaternion)
        # ===========================
        q_est = [est_msg.pose.pose.orientation.x, est_msg.pose.pose.orientation.y, est_msg.pose.pose.orientation.z, est_msg.pose.pose.orientation.w]
        q_gt = [gt_msg.pose.pose.orientation.x, gt_msg.pose.pose.orientation.y, gt_msg.pose.pose.orientation.z, gt_msg.pose.pose.orientation.w]

        r_est = Rotation.from_quat(q_est)
        r_gt = Rotation.from_quat(q_gt)

        euler_est = r_est.as_euler('xyz', degrees=True)
        euler_gt = r_gt.as_euler('xyz', degrees=True)

        msg_euler_est = Float64MultiArray()
        msg_euler_est.data = euler_est.tolist()
        self.pub_euler_est.publish(msg_euler_est)

        msg_euler_gt = Float64MultiArray()
        msg_euler_gt.data = euler_gt.tolist()
        self.pub_euler_gt.publish(msg_euler_gt)
        
        r_err = r_gt.inv() * r_est
        err_ori_vec = r_err.as_rotvec()
        
        self.sse_ori += np.linalg.norm(err_ori_vec)**2
        self.publish_float(self.pub_rmse_ori, np.sqrt(self.sse_ori / self.n_samples))

        # ===========================
        # 3. VELOCITY (Euclidean & Axis)
        # ===========================
        v_est = np.array([est_msg.twist.twist.linear.x, est_msg.twist.twist.linear.y, est_msg.twist.twist.linear.z])
        v_gt = np.array([gt_msg.twist.twist.linear.x, gt_msg.twist.twist.linear.y, gt_msg.twist.twist.linear.z])
        
        err_vel_vec = v_est - v_gt
        
        # Update Axis-Specific RMSE
        self.sse_vel_xyz += err_vel_vec**2
        rmse_vel_xyz = np.sqrt(self.sse_vel_xyz / self.n_samples)
        
        self.publish_float(self.pub_rmse_vel_x, rmse_vel_xyz[0])
        self.publish_float(self.pub_rmse_vel_y, rmse_vel_xyz[1])
        self.publish_float(self.pub_rmse_vel_z, rmse_vel_xyz[2])

        # Update Total 3D RMSE
        self.sse_vel_total += np.linalg.norm(err_vel_vec)**2
        self.publish_float(self.pub_rmse_vel_total, np.sqrt(self.sse_vel_total / self.n_samples))

        # ===========================
        # 4. NEES CALCULATION
        # ===========================
        cov_pose = np.array(est_msg.pose.covariance).reshape(6, 6)
        
        # --- Position NEES ---
        cov_pos = cov_pose[0:3, 0:3]
        try:
            cov_pos_inv = np.linalg.inv(cov_pos)
            nees_pos = err_pos_vec.T @ cov_pos_inv @ err_pos_vec
            self.publish_float(self.pub_nees_pos, nees_pos)
        except np.linalg.LinAlgError:
            pass 

        # --- Orientation NEES ---
        cov_ori = cov_pose[3:6, 3:6]
        try:
            cov_ori_inv = np.linalg.inv(cov_ori)
            nees_ori = err_ori_vec.T @ cov_ori_inv @ err_ori_vec
            self.publish_float(self.pub_nees_ori, nees_ori)
        except np.linalg.LinAlgError:
            pass

    def publish_float(self, publisher, value):
        msg = Float64()
        msg.data = float(value)
        publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = EskfValidator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
