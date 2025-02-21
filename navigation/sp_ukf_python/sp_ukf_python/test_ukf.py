import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # for 3D plotting

# (Assuming the following have been imported from your modules)
from sp_ukf_python_class import StateVector_quaternion, StateVector_euler
from sp_ukf_python_utils import skew_symmetric, quaternion_super_product
from sp_ukf_python import ErrorStateUnscentedKalmanFilter

def quat_to_yaw(q: np.ndarray) -> float:
    """
    Convert a quaternion (assumed [w, x, y, z]) with zero roll and pitch
    into a yaw angle.
    """
    return 2 * np.arctan2(q[3], q[0])

def run_ESUKF_simulation():
    # Simulation parameters
    dt = 0.01               # time step [s]
    T = 60.0               # total simulation time [s]
    num_steps = int(T/dt)
    g_val = 9.81           # gravitational acceleration

    # Define noise and covariance matrices
    Q = np.diag([0.1]*15)                  # Process noise covariance (15x15)
    R_meas = np.diag([0.08]*3)              # DVL measurement noise (velocity noise)
    P_ab = 0.005                             # Accelerometer bias dynamics matrix
    P_wb = 0.005                             # Gyro bias dynamics matrix
    lever_arm = np.array([0.0, 0.0, 0.0])   # Assume sensor is at the center of mass

    # Create ESUKF instance
    esukf = ErrorStateUnscentedKalmanFilter(P_ab, P_wb, Q, lever_arm, R_meas, g_val, dt)

    # Initialize true state (StateVector_quaternion) with no biases.
    true_state = StateVector_quaternion()
    true_state.position = np.array([20.0, 0.0, 0.0])
    true_state.velocity = np.array([0.0, 1.0, 0.0])
    true_state.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # No initial rotation
    true_state.acceleration_bias = np.zeros(3)
    true_state.gyro_bias = np.zeros(3)

    # Initialize estimated (nominal) state with a small offset.
    est_state_nom = StateVector_quaternion()
    est_state_nom.position = true_state.position + np.array([0.1, -0.1, 0.05])
    est_state_nom.velocity = true_state.velocity + np.array([0.05, 0.05, -0.05])
    est_state_nom.orientation = true_state.orientation.copy()
    est_state_nom.acceleration_bias = np.zeros(3)
    est_state_nom.gyro_bias = np.zeros(3)

    # Initialize error state (StateVector_euler) as zero with some initial covariance.
    est_state_error = StateVector_euler()
    est_state_error.fill_states(np.zeros(15))
    est_state_error.covariance = 0.1 * np.eye(15)

    # Prepare histories for plotting
    time_hist = []
    true_pos_hist = []
    est_pos_hist = []
    true_vel_hist = []
    est_vel_hist = []
    true_yaw_hist = []
    est_yaw_hist = []

    # For the true trajectory, we simulate a circle in the horizontal plane.
    R_circle = 20.0      # circle radius [m]
    omega = 0.05         # angular speed [rad/s]

    t = 0.0
    for step in range(num_steps):
        # --- True State Generation ---
        # Circular trajectory: position = [R*cos(omega*t), R*sin(omega*t), 0]
        pos_true = np.array([R_circle * np.cos(omega * t),
                             R_circle * np.sin(omega * t),
                             0.0])
        # Velocity is the derivative of position.
        vel_true = np.array([-R_circle * omega * np.sin(omega * t),
                              R_circle * omega * np.cos(omega * t),
                              0.0])
        # Acceleration is the second derivative.
        acc_true = np.array([-R_circle * omega**2 * np.cos(omega * t),
                             -R_circle * omega**2 * np.sin(omega * t),
                             0.0])
        # Update the true state.
        true_state.position = pos_true
        true_state.velocity = vel_true
        # Compute heading (yaw) tangent to the path.
        yaw_true = np.arctan2(vel_true[1], vel_true[0])
        # For simplicity, assume roll and pitch are zero.
        true_state.orientation = np.array([np.cos(yaw_true/2), 0.0, 0.0, np.sin(yaw_true/2)])
        # Biases remain zero for the true state.

        # --- Simulated IMU Measurements ---
        # The nominal state propagation uses:
        #   velocity_dot = R_q() @ (imu_acc - bias) + g
        # Therefore, the ideal accelerometer measurement is:
        #   imu_acc = R_true.T @ (acc_true - g_vector)
        R_true = true_state.R_q()  # rotation matrix from quaternion
        imu_acc_ideal = np.dot(R_true.T, (acc_true - np.array([0.0, 0.0, g_val])))
        # Add noise (e.g., 0.1 m/s^2 std dev).
        imu_acc_noise = np.random.normal(0.0, 0.1, 3)
        imu_acc_meas = imu_acc_ideal + imu_acc_noise

        # For the gyro: the true angular velocity in body frame.
        # For a circular path with constant yaw rate, the ideal gyro reading is:
        imu_gyro_ideal = np.array([0.0, 0.0, omega])
        # Add noise (e.g., 0.01 rad/s std dev).
        imu_gyro_noise = np.random.normal(0.0, 0.01, 3)
        imu_gyro_meas = imu_gyro_ideal + imu_gyro_noise

        # Combine to form the complete IMU measurement vector.
        imu_meas = np.hstack((imu_acc_meas, imu_gyro_meas))

        # --- Simulated DVL Measurement ---
        # DVL measures velocity (here assumed in the inertial frame).
        dvl_noise = np.random.normal(0.0, 0.05, 3)
        dvl_meas = vel_true + dvl_noise

        # --- Filter Updates ---
        # 1. Propagate the nominal state with IMU data.
        est_state_nom, est_state_error = esukf.imu_update_states(est_state_nom, est_state_error, imu_meas)
        # 2. Incorporate DVL measurement.
        est_state_nom, est_state_error = esukf.dvl_update_states(est_state_nom, est_state_error, dvl_meas)
        # 3. Inject the error state into the nominal state and reset the error state.
        est_state_nom, est_state_error = esukf.inject_and_reset(est_state_nom, est_state_error)

        # --- Store Histories ---
        time_hist.append(t)
        true_pos_hist.append(pos_true)
        est_pos_hist.append(est_state_nom.position.copy())
        true_vel_hist.append(vel_true)
        est_vel_hist.append(est_state_nom.velocity.copy())
        true_yaw_hist.append(yaw_true)
        est_yaw_hist.append(quat_to_yaw(est_state_nom.orientation))

        t += dt

    # Convert histories to NumPy arrays.
    true_pos_hist = np.array(true_pos_hist)
    est_pos_hist = np.array(est_pos_hist)
    true_vel_hist = np.array(true_vel_hist)
    est_vel_hist = np.array(est_vel_hist)
    true_yaw_hist = np.array(true_yaw_hist)
    est_yaw_hist = np.array(est_yaw_hist)
    time_hist = np.array(time_hist)

    # --- Plotting Results ---

    # Plot positions (each axis separately)
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_hist, true_pos_hist[:, 0], label='True X')
    plt.plot(time_hist, est_pos_hist[:, 0], '--', label='Estimated X')
    plt.ylabel('X Position (m)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_hist, true_pos_hist[:, 1], label='True Y')
    plt.plot(time_hist, est_pos_hist[:, 1], '--', label='Estimated Y')
    plt.ylabel('Y Position (m)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_hist, true_pos_hist[:, 2], label='True Z')
    plt.plot(time_hist, est_pos_hist[:, 2], '--', label='Estimated Z')
    plt.xlabel('Time (s)')
    plt.ylabel('Z Position (m)')
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Plot velocities
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_hist, true_vel_hist[:, 0], label='True Vx')
    plt.plot(time_hist, est_vel_hist[:, 0], '--', label='Estimated Vx')
    plt.ylabel('Vx (m/s)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_hist, true_vel_hist[:, 1], label='True Vy')
    plt.plot(time_hist, est_vel_hist[:, 1], '--', label='Estimated Vy')
    plt.ylabel('Vy (m/s)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_hist, true_vel_hist[:, 2], label='True Vz')
    plt.plot(time_hist, est_vel_hist[:, 2], '--', label='Estimated Vz')
    plt.xlabel('Time (s)')
    plt.ylabel('Vz (m/s)')
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Plot heading (yaw)
    plt.figure(figsize=(10, 4))
    plt.plot(time_hist, np.degrees(true_yaw_hist), label='True Yaw')
    plt.plot(time_hist, np.degrees(est_yaw_hist), '--', label='Estimated Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.legend()
    plt.title('Heading Comparison')
    plt.tight_layout()
    plt.show()

    # Plot 3D Trajectory
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(true_pos_hist[:, 0], true_pos_hist[:, 1], true_pos_hist[:, 2], label='True Trajectory', linewidth=2)
    ax.plot(est_pos_hist[:, 0], est_pos_hist[:, 1], est_pos_hist[:, 2], '--', label='Estimated Trajectory', linewidth=2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.legend()
    plt.title('3D Trajectory')
    plt.show()

if __name__ == '__main__':
    run_ESUKF_simulation()
