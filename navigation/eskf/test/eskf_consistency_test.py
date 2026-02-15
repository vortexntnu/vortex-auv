import message_filters
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation as RotationClass
from std_msgs.msg import Float64, Float64MultiArray

# --- IMPORT YOUR EXISTING QOS FUNCTION ---
try:
    from vortex.utils.qos_profiles import sensor_data_profile
except ImportError:
    # Fallback for testing without the vortex library
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

    def sensor_data_profile(depth=5):
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=depth,
        )


class EskfValidator(Node):
    def __init__(self):
        super().__init__('eskf_validator')

        self.est_topic = '/odom_ESKF'
        self.gt_topic = '/orca/odom'

        # --- Publishers (Foxglove Visualizers) ---
        # RMSE (Running Root Mean Square Error)
        self.pub_rmse_pos = self.create_publisher(
            Float64, 'eskf_metrics/rmse/position', 10
        )
        self.pub_rmse_ori = self.create_publisher(
            Float64, 'eskf_metrics/rmse/orientation_rad', 10
        )
        self.pub_rmse_vel = self.create_publisher(
            Float64, 'eskf_metrics/rmse/velocity', 10
        )

        # Euler Angles (for visual comparison)
        self.pub_euler_est = self.create_publisher(
            Float64MultiArray, 'debug/euler/est', 10
        )
        self.pub_euler_gt = self.create_publisher(
            Float64MultiArray, 'debug/euler/gt', 10
        )

        # NEEDS (Normalized Estimation Error Squared)
        self.pub_nees_pos = self.create_publisher(
            Float64, 'eskf_metrics/needs/position', 10
        )
        self.pub_nees_ori = self.create_publisher(
            Float64, 'eskf_metrics/needs/orientation', 10
        )

        # --- State for Running RMSE ---
        self.n_samples = 0
        self.sse_pos = 0.0  # Sum of Squared Errors
        self.sse_ori = 0.0
        self.sse_vel = 0.0

        # --- Subscribers ---
        self.get_logger().info(
            f"Subscribing to {self.est_topic} and {self.gt_topic}..."
        )
        qos = sensor_data_profile(depth=5)

        self.sub_est = message_filters.Subscriber(
            self, Odometry, self.est_topic, qos_profile=qos
        )
        self.sub_gt = message_filters.Subscriber(
            self, Odometry, self.gt_topic, qos_profile=qos
        )

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub_est, self.sub_gt], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.callback)

    def callback(self, est_msg, gt_msg):
        self.n_samples += 1

        # ===========================
        # 1. POSITION (Euclidean)
        # ===========================
        p_est = np.array(
            [
                est_msg.pose.pose.position.x,
                est_msg.pose.pose.position.y,
                est_msg.pose.pose.position.z,
            ]
        )
        p_gt = np.array(
            [
                gt_msg.pose.pose.position.x,
                gt_msg.pose.pose.position.y,
                gt_msg.pose.pose.position.z,
            ]
        )

        err_pos_vec = p_est - p_gt
        err_pos_norm = np.linalg.norm(err_pos_vec)

        # Update RMSE
        self.sse_pos += err_pos_norm**2
        self.publish_float(self.pub_rmse_pos, np.sqrt(self.sse_pos / self.n_samples))

        # ===========================
        # 2. ORIENTATION (Quaternion)
        # ===========================
        # Extract Quaternions (x, y, z, w)
        q_est = [
            est_msg.pose.pose.orientation.x,
            est_msg.pose.pose.orientation.y,
            est_msg.pose.pose.orientation.z,
            est_msg.pose.pose.orientation.w,
        ]
        q_gt = [
            gt_msg.pose.pose.orientation.x,
            gt_msg.pose.pose.orientation.y,
            gt_msg.pose.pose.orientation.z,
            gt_msg.pose.pose.orientation.w,
        ]

        # Convert to Rotation objects
        r_est = RotationClass.from_quat(q_est)
        r_gt = RotationClass.from_quat(q_gt)

        # ---------------------------------------------------------
        # Extract Euler Angles (Roll, Pitch, Yaw)
        # Using 'xyz' sequence (standard for ROS aerospace/underwater)
        # degrees=True makes it easier to read in plots (0-360 or +/-180)
        # ---------------------------------------------------------
        euler_est = r_est.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]
        euler_gt = r_gt.as_euler('xyz', degrees=True)  # [roll, pitch, yaw]

        # Publish Estimated Euler
        msg_euler_est = Float64MultiArray()
        msg_euler_est.data = euler_est.tolist()
        self.pub_euler_est.publish(msg_euler_est)

        # Publish GT Euler
        msg_euler_gt = Float64MultiArray()
        msg_euler_gt.data = euler_gt.tolist()
        self.pub_euler_gt.publish(msg_euler_gt)
        # Calculate Error Rotation: R_err = R_gt^T * R_est
        # This gives the relative rotation needed to go from GT to Est
        r_err = r_gt.inv() * r_est

        # Convert to Rotation Vector (magnitude is the angle error in radians)
        err_ori_vec = r_err.as_rotvec()
        err_ori_norm = np.linalg.norm(err_ori_vec)

        # Update RMSE
        self.sse_ori += err_ori_norm**2
        self.publish_float(self.pub_rmse_ori, np.sqrt(self.sse_ori / self.n_samples))

        # ===========================
        # 3. VELOCITY (Euclidean)
        # ===========================
        v_est = np.array(
            [
                est_msg.twist.twist.linear.x,
                est_msg.twist.twist.linear.y,
                est_msg.twist.twist.linear.z,
            ]
        )
        v_gt = np.array(
            [
                gt_msg.twist.twist.linear.x,
                gt_msg.twist.twist.linear.y,
                gt_msg.twist.twist.linear.z,
            ]
        )

        err_vel_vec = v_est - v_gt
        self.sse_vel += np.linalg.norm(err_vel_vec) ** 2
        self.publish_float(self.pub_rmse_vel, np.sqrt(self.sse_vel / self.n_samples))

        # Note: Standard Odometry does NOT contain Linear Acceleration.
        # If you need Accel RMSE, you must subscribe to the IMU topic separately.

        # ===========================
        # 4. NEEDS CALCULATION
        # ===========================
        # NEEDS = error^T * Covariance^-1 * error

        # Reshape the 36-float array into 6x6 matrix
        cov_pose = np.array(est_msg.pose.covariance).reshape(6, 6)

        # --- Position NEEDS (Top-Left 3x3) ---
        cov_pos = cov_pose[0:3, 0:3]
        try:
            cov_pos_inv = np.linalg.inv(cov_pos)
            nees_pos = err_pos_vec.T @ cov_pos_inv @ err_pos_vec
            self.publish_float(self.pub_nees_pos, nees_pos)
        except np.linalg.LinAlgError:
            pass  # Singular matrix, skip

        # --- Orientation NEEDS (Bottom-Right 3x3) ---
        # Note: We use the rotation vector error (err_ori_vec) calculated earlier
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
