import matplotlib.pyplot as plt
import numpy as np

# (Assuming the following have been imported from your modules)
from sp_ukf_python_class import StateVector_euler, StateVector_quaternion

from sp_ukf_python import ErrorStateUnscentedKalmanFilter


def quat_to_yaw(q: np.ndarray) -> float:
    """Convert a quaternion (assumed [w, x, y, z]) into a yaw angle.
    In NED, yaw is typically around the z-down axis.
    """
    return 2 * np.arctan2(q[3], q[0])


def run_ESUKF_simulation():
    # -------------------------------------------------------------------------
    # Simulation parameters
    # -------------------------------------------------------------------------
    dt = 0.01  # time step [s]
    T = 60.0  # total simulation time [s]
    num_steps = int(T / dt)

    # In an NED frame, gravity is +9.81 in the z (down) direction.
    g_val = 9.81

    # -------------------------------------------------------------------------
    # Define noise and covariance matrices
    # -------------------------------------------------------------------------
    Q = np.diag(
        [
            0.06,
            0.06,
            0.06,  # position error
            0.04,
            0.04,
            0.04,  # velocity error
            0.003,
            0.003,
            0.003,  # orientation error
            0.02,
            0.02,
            0.02,  # accelerometer bias error
            0.02,
            0.02,
            0.02,  # gyro bias error
        ]
    )

    R_meas = np.diag([0.52, 0.52, 0.52])  # Increased DVL measurement noise

    # Bias dynamics tuning remains the same here:
    P_ab = 0.002
    P_wb = 0.002
    lever_arm = np.array([0.0, 0.0, 0.0])  # Sensor at the vehicle CG

    # Create the Error-State UKF instance (NED convention)
    esukf = ErrorStateUnscentedKalmanFilter(P_ab, P_wb, Q, lever_arm, R_meas, g_val, dt)

    # -------------------------------------------------------------------------
    # Initialize the true state in NED
    # -------------------------------------------------------------------------
    # We treat x as North, y as East, z as Down.
    # We'll do a circular path in the horizontal plane (z=0).
    true_state = StateVector_quaternion()
    true_state.position = np.array([20.0, 0.0, 0.0])  # [N, E, D]=[20, 0, 0]
    true_state.velocity = np.array([0.0, 1.0, 0.0])  # 1 m/s in the East direction
    true_state.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # No initial rotation
    true_state.acceleration_bias = np.zeros(3)
    true_state.gyro_bias = np.zeros(3)

    # -------------------------------------------------------------------------
    # Initialize the estimated state
    # -------------------------------------------------------------------------
    est_state_nom = StateVector_quaternion()
    est_state_nom.position = true_state.position + np.array([0.1, -0.1, 0.05])
    est_state_nom.velocity = true_state.velocity + np.array([0.05, 0.05, -0.05])
    est_state_nom.orientation = true_state.orientation.copy()
    est_state_nom.acceleration_bias = np.zeros(3)
    est_state_nom.gyro_bias = np.zeros(3)

    # Initialize error state (Euler) with some covariance
    est_state_error = StateVector_euler()
    est_state_error.fill_states(np.zeros(15))
    est_state_error.covariance = 0.5 * np.eye(15)

    # -------------------------------------------------------------------------
    # Prepare histories for plotting
    # -------------------------------------------------------------------------
    time_hist = []
    true_pos_hist = []
    est_pos_hist = []
    true_vel_hist = []
    est_vel_hist = []
    true_yaw_hist = []
    est_yaw_hist = []

    # -------------------------------------------------------------------------
    # Define the "circular" trajectory in the horizontal plane (z=0)
    # in NED: x=North, y=East, z=Down
    # We'll revolve in the XY-plane, at D=0, with radius=20 m, angular speed=0.05 rad/s
    # -------------------------------------------------------------------------
    R_circle = 20.0
    omega = 0.05

    # -------------------------------------------------------------------------
    # Main simulation loop
    # -------------------------------------------------------------------------
    t = 0.0
    for step in range(num_steps):
        # --- True State Generation (NED) ---
        # Position: circle in x-y plane at z=0
        pos_true = np.array(
            [
                R_circle * np.cos(omega * t),  # N
                R_circle * np.sin(omega * t),  # E
                0.0,  # D
            ]
        )
        # Velocity: derivative of pos
        vel_true = np.array(
            [
                -R_circle * omega * np.sin(omega * t),  # d/dt of cos => -sin
                R_circle * omega * np.cos(omega * t),  # d/dt of sin => cos
                0.0,
            ]
        )
        # Acceleration: second derivative
        acc_true = np.array(
            [
                -R_circle * omega**2 * np.cos(omega * t),
                -R_circle * omega**2 * np.sin(omega * t),
                0.0,
            ]
        )

        # Update the "true" state in NED
        true_state.position = pos_true
        true_state.velocity = vel_true

        # Compute full quaternion from Euler angles (roll, pitch, yaw)
        roll_true = 0.0
        pitch_true = 0.0
        yaw_true = np.arctan2(vel_true[1], vel_true[0])
        cy = np.cos(yaw_true * 0.5)
        sy = np.sin(yaw_true * 0.5)
        cp = np.cos(pitch_true * 0.5)
        sp = np.sin(pitch_true * 0.5)
        cr = np.cos(roll_true * 0.5)
        sr = np.sin(roll_true * 0.5)
        true_state.orientation = np.array(
            [
                cr * cp * cy + sr * sp * sy,  # w
                sr * cp * cy - cr * sp * sy,  # x
                cr * sp * cy + sr * cp * sy,  # y
                cr * cp * sy - sr * sp * cy,  # z
            ]
        )

        # --- Simulated IMU Measurements (NED) ---
        # Gravity is +9.81 in the down (z) direction in NED
        R_true = true_state.R_q()  # rotation from body to inertial
        # The "specific force" in body frame is (acc_inertial - gravity_inertial) rotated to body
        imu_acc_ideal = R_true.T @ (
            acc_true - np.array([0.0, 0.0, g_val])
        ) + np.random.normal(0.01, 0.01, 3)  # [rad/s]

        # Add small noise
        imu_acc_noise = np.random.normal(0.0, 0.05, 3)  # [m/s^2]
        imu_acc_meas = imu_acc_ideal + imu_acc_noise

        # Gyro: angular velocity about body axes. Yaw rate is ~omega for a flat circle
        imu_gyro_ideal = np.array([0.0, 0.0, omega]) + np.random.normal(
            0.01, 0.01, 3
        )  # [rad/s]
        imu_gyro_noise = np.random.normal(0.0, 0.05, 3)  # [rad/s]
        imu_gyro_meas = imu_gyro_ideal + imu_gyro_noise

        # Combine
        imu_meas = np.hstack((imu_acc_meas, imu_gyro_meas))

        # --- Simulated DVL Measurement ---
        # Velocity in inertial frame (NED) with zero noise for this test
        dvl_noise = np.random.normal(0.0, 0.05, 3)
        dvl_meas = vel_true + dvl_noise

        # ---------------------------------------------------------------------
        # Filter Updates
        # ---------------------------------------------------------------------
        # 1. IMU update (prediction)
        est_state_nom, est_state_error = esukf.imu_update_states(
            est_state_nom, est_state_error, imu_meas
        )
        # 2. DVL update (measurement)
        est_state_nom, est_state_error = esukf.dvl_update_states(
            est_state_nom, est_state_error, dvl_meas
        )
        # 3. Inject error state
        est_state_nom, est_state_error = esukf.inject_and_reset(
            est_state_nom, est_state_error
        )

        # --- Store Histories ---
        time_hist.append(t)
        true_pos_hist.append(pos_true)
        est_pos_hist.append(est_state_nom.position.copy())
        true_vel_hist.append(vel_true)
        est_vel_hist.append(est_state_nom.velocity.copy())
        true_yaw_hist.append(yaw_true)
        est_yaw_hist.append(quat_to_yaw(est_state_nom.orientation))

        t += dt

    # -------------------------------------------------------------------------
    # Convert histories to arrays
    # -------------------------------------------------------------------------
    true_pos_hist = np.array(true_pos_hist)
    est_pos_hist = np.array(est_pos_hist)
    true_vel_hist = np.array(true_vel_hist)
    est_vel_hist = np.array(est_vel_hist)
    true_yaw_hist = np.array(true_yaw_hist)
    est_yaw_hist = np.array(est_yaw_hist)
    time_hist = np.array(time_hist)

    # -------------------------------------------------------------------------
    # Plotting
    # -------------------------------------------------------------------------
    # Positions
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_hist, true_pos_hist[:, 0], label='True N')
    plt.plot(time_hist, est_pos_hist[:, 0], '--', label='Estimated N')
    plt.ylabel('N (m)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_hist, true_pos_hist[:, 1], label='True E')
    plt.plot(time_hist, est_pos_hist[:, 1], '--', label='Estimated E')
    plt.ylabel('E (m)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_hist, true_pos_hist[:, 2], label='True D')
    plt.plot(time_hist, est_pos_hist[:, 2], '--', label='Estimated D')
    plt.xlabel('Time (s)')
    plt.ylabel('D (m)')
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Velocities
    plt.figure(figsize=(10, 8))
    plt.subplot(3, 1, 1)
    plt.plot(time_hist, true_vel_hist[:, 0], label='True Vn')
    plt.plot(time_hist, est_vel_hist[:, 0], '--', label='Estimated Vn')
    plt.ylabel('Vn (m/s)')
    plt.legend()

    plt.subplot(3, 1, 2)
    plt.plot(time_hist, true_vel_hist[:, 1], label='True Ve')
    plt.plot(time_hist, est_vel_hist[:, 1], '--', label='Estimated Ve')
    plt.ylabel('Ve (m/s)')
    plt.legend()

    plt.subplot(3, 1, 3)
    plt.plot(time_hist, true_vel_hist[:, 2], label='True Vd')
    plt.plot(time_hist, est_vel_hist[:, 2], '--', label='Estimated Vd')
    plt.xlabel('Time (s)')
    plt.ylabel('Vd (m/s)')
    plt.legend()
    plt.tight_layout()
    plt.show()

    # Heading (Yaw)
    plt.figure(figsize=(10, 4))
    plt.plot(time_hist, np.degrees(true_yaw_hist), label='True Yaw')
    plt.plot(time_hist, np.degrees(est_yaw_hist), '--', label='Estimated Yaw')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw (deg)')
    plt.legend()
    plt.title('Heading Comparison (NED)')
    plt.tight_layout()
    plt.show()

    # 3D Trajectory
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(
        true_pos_hist[:, 0],
        true_pos_hist[:, 1],
        true_pos_hist[:, 2],
        label='True Trajectory',
        linewidth=2,
    )
    ax.plot(
        est_pos_hist[:, 0],
        est_pos_hist[:, 1],
        est_pos_hist[:, 2],
        '--',
        label='Estimated Trajectory',
        linewidth=2,
    )
    ax.set_xlabel('North (m)')
    ax.set_ylabel('East (m)')
    ax.set_zlabel('Down (m)')
    ax.legend()
    plt.title('3D Trajectory (NED Frame)')
    plt.show()


if __name__ == '__main__':
    run_ESUKF_simulation()
