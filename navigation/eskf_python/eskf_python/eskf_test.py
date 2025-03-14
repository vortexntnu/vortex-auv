
from eskf_python_class import StateEuler, StateQuat, MeasurementModel, Measurement
import numpy as np 
from eskf_python_utils import skew_matrix, quaternion_product, R_from_angle_axis, angle_axis_to_quaternion
from ukf_okid_class import process_model, quat_to_euler, euler_to_quat
from ukf_okid_class import StateQuat as StateQuatmodel
from scipy.linalg import block_diag
import matplotlib.pyplot as plt
from eskf_python_filter import ESKF


def fancy_print_state_quat(state: StateQuat) -> None:
    print("Nominal State (Quaternion):")
    print(f"  Position          : {np.array2string(state.position, precision=3, separator=', ')}")
    print(f"  Velocity          : {np.array2string(state.velocity, precision=3, separator=', ')}")
    print(f"  Orientation (Quat): {np.array2string(state.orientation, precision=3, separator=', ')}")
    print(f"  Acceleration Bias : {np.array2string(state.acceleration_bias, precision=3, separator=', ')}")
    print(f"  Gyro Bias         : {np.array2string(state.gyro_bias, precision=3, separator=', ')}")
    print(f"  Gravity           : {np.array2string(state.g, precision=3, separator=', ')}\n")


def fancy_print_state_euler(state: StateEuler) -> None:
    print("Error State (Euler):")
    print(f"  Position Error         : {np.array2string(state.position, precision=3, separator=', ')}")
    print(f"  Velocity Error         : {np.array2string(state.velocity, precision=3, separator=', ')}")
    print(f"  Orientation Error      : {np.array2string(state.orientation, precision=3, separator=', ')}")
    print(f"  Acceleration Bias Error: {np.array2string(state.acceleration_bias, precision=3, separator=', ')}")
    print(f"  Gyro Bias Error        : {np.array2string(state.gyro_bias, precision=3, separator=', ')}")
    print(f"  Gravity Error          : {np.array2string(state.g, precision=3, separator=', ')}\n")

def fancy_print_matrix(matrix: np.ndarray) -> None:
    print(f"Matrix shape: {matrix.shape}")
    print("========== Matrix ==========")
    for row in matrix:
        print("  ".join(f"{value:8.3f}" for value in row))
    print("======== End Matrix ========")

if __name__ == "__main__":

    # Simulation parameters
    simulation_time = 20.0  # seconds
    dt = 0.01
    num_steps = int(simulation_time / dt)
    time = np.linspace(0, simulation_time, num_steps)

    # ----------------------- Setup Initial States, Filter & Model -----------------------
    # True initial state
    true_state_init = StateQuat()
    true_state_init.position = np.array([0.1, 0.0, 0.0])
    true_state_init.velocity = np.array([0.1, 0.0, 0.0])
    P0 = np.diag([
        1.0, 1.0, 1.0,        # Position
        0.2, 0.2, 0.2,        # Velocity
        0.01, 0.01, 0.01,        # Orientation
        0.00001, 0.00001, 0.00001,        # Acceleration bias
        0.00001, 0.00001, 0.00001,        # Gyro bias
        0.00001, 0.00001, 0.00001         # Gravity
    ])

    # Estimated initial state (for filter)
    est_state_init = StateQuat()
    est_state_init.position = np.array([0.1, 0.0, 0.0])
    est_state_init.velocity = np.array([0.1, 0.0, 0.0])

    # Noise parameters
    Q = np.diag([
        (0.02**2) / dt, (0.02**2) / dt, (0.02**2) / dt,        # Accelerometer noise
        (0.001**2) / dt, (0.001**2) / dt, (0.001**2) / dt,        # Gyroscope noise
        0.0001, 0.0001, 0.0001,                                # Acceleration bias random walk
        0.00001, 0.00001, 0.00001                                 # Gyro bias random walk
    ])
    
    Hx = np.zeros((3, 19))
    Hx[0:3, 6:9] = np.eye(3)

    # Create filter object
    eskf = ESKF(Q, P0, Hx, true_state_init, 1e-13, 1e-13, dt)

    imu_data = Measurement()
    dvl_data = Measurement()
    dvl_data.aiding_covariance = np.eye(3) * 0.2

    # Setup the process model
    model = process_model()
    model.dt = dt
    model.mass_interia_matrix = np.array([
        [30.0, 0.0, 0.0, 0.0, 0.0, 0.6],
        [0.0, 30.0, 0.0, 0.0, -0.6, 0.3],
        [0.0, 0.0, 30.0, 0.6, 0.3, 0.0],
        [0.0, 0.0, 0.6, 0.68, 0.0, 0.0],
        [0.0, -0.6, 0.3, 0.0, 3.32, 0.0],
        [0.6, 0.3, 0.0, 0.0, 0.0, 3.34]
    ])
    model.m = 30.0
    model.r_b_bg = np.array([0.01, 0.0, 0.02])
    model.inertia = np.diag([0.68, 3.32, 3.34])
    model.damping_linear = np.diag([0.03, 0.03, 0.03, 0.03, 0.03, 0.03])

    # Initialize a dummy state for simulation dynamics.
    new_state = StateQuatmodel()
    new_state.position = np.array([0.1, 0.0, 0.0])
    new_state.velocity = np.array([0.1, 0.0, 0.0])
    new_state_prev = StateQuatmodel()
    new_state_prev.position = np.array([0.1, 0.0, 0.0])
    new_state_prev.velocity = np.array([0.1, 0.0, 0.0])

    model.state_vector = new_state
    model.state_vector_prev = new_state_prev

    # ----------------------- Data Storage Arrays -----------------------
    true_positions = np.zeros((num_steps, 3))
    true_orientations = np.zeros((num_steps, 3))
    true_velocities = np.zeros((num_steps, 3))

    est_positions = np.zeros((num_steps, 3))
    est_orientations = np.zeros((num_steps, 3))
    est_velocities = np.zeros((num_steps, 3))

    # We'll record the filter’s covariance diagonal for each state component.
    pos_cov = np.zeros((num_steps, 3))   # covariance for position (indices 0:3)
    vel_cov = np.zeros((num_steps, 3))   # covariance for velocity (indices 3:6)
    ori_cov = np.zeros((num_steps, 3))   # covariance for orientation (indices 6:9)

    prev_velocity = np.zeros(3)
    u = lambda t: np.array([
        0.5 * np.sin(0.1 * t),
        0.5 * np.sin(0.1 * t + 0.3),
        0.5 * np.sin(0.1 * t + 0.6),
        0.05 * np.cos(0.1 * t),
        0.05 * np.cos(0.1 * t + 0.3),
        0.05 * np.cos(0.1 * t + 0.6)
    ])

    # ----------------------- Simulation Loop -----------------------
    for step in range(num_steps):
        t = step * dt

        model.Control_input = u(t)
        model.model_prediction(new_state)
        new_state = model.euler_forward()

        # Simulate IMU measurements (with noise)
        imu_data.acceleration = ((new_state.velocity - prev_velocity) / dt) + np.random.normal(0, 0.13, 3)
        imu_data.angular_velocity = new_state.angular_velocity + np.random.normal(0, 0.13, 3)

        eskf.imu_update(imu_data)

        # DVL update every 20 time-steps
        if step % 20 == 0:
            dvl_data.aiding = new_state.velocity + np.random.normal(0, 0.01, 3)
            eskf.dvl_update(dvl_data)

        # Store True data (from the simulated dynamics)
        true_positions[step, :] = np.copy(new_state.position)
        true_orientations[step, :] = quat_to_euler(np.copy(new_state.orientation))
        true_velocities[step, :] = np.copy(new_state.velocity)

        # Store estimated state (from the filter)
        est_positions[step, :] = np.copy(eskf.nom_state.position)
        est_orientations[step, :] = quat_to_euler(np.copy(eskf.nom_state.orientation))
        est_velocities[step, :] = np.copy(eskf.nom_state.velocity)

        # Record covariance diagonal (assumed ordering: pos (0:3), vel (3:6), orientation (6:9))
        P_diag = np.diag(eskf.error_state.covariance)
        pos_cov[step, :] = P_diag[0:3]
        vel_cov[step, :] = P_diag[3:6]
        ori_cov[step, :] = P_diag[6:9]

        prev_velocity = new_state.velocity
        model.state_vector_prev = new_state

    # ----------------------- New Plotting Scheme -----------------------
    # Create 3 separate figures, each corresponding to one degree of freedom:
    # For position and velocity: X, Y, Z.
    # For orientation: Roll, Pitch, Yaw.
    axis_labels_pos = ["X", "Y", "Z"]
    axis_labels_vel = ["X", "Y", "Z"]
    axis_labels_ori = ["Roll", "Pitch", "Yaw"]

    # Plot Position
    fig_pos, axs_pos = plt.subplots(3, 1, figsize=(10, 12))
    fig_pos.suptitle("True Data vs Filter Estimates for Position")
    for i in range(3):
        ax_pos = axs_pos[i]
        ax_pos.plot(time, true_positions[:, i], label=f"True Pos {axis_labels_pos[i]}", color=f"C{i}", linestyle='-')
        ax_pos.plot(time, est_positions[:, i], label=f"Est Pos {axis_labels_pos[i]}", color=f"C{i}", linestyle='--')
        sigma_pos = np.sqrt(pos_cov[:, i])
        ax_pos.fill_between(time, est_positions[:, i] - sigma_pos, est_positions[:, i] + sigma_pos, 
                            color=f"C{i}", alpha=0.2)
        ax_pos.set_title(f"Position [{axis_labels_pos[i]}] [m]")
        ax_pos.set_xlabel("Time [s]")
        ax_pos.set_ylabel("Position")
        ax_pos.grid(True)
        ax_pos.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

    # Plot Velocity
    fig_vel, axs_vel = plt.subplots(3, 1, figsize=(10, 12))
    fig_vel.suptitle("True Data vs Filter Estimates for Velocity")
    for i in range(3):
        ax_vel = axs_vel[i]
        ax_vel.plot(time, true_velocities[:, i], label=f"True Vel {axis_labels_vel[i]}", color=f"C{i}", linestyle='-')
        ax_vel.plot(time, est_velocities[:, i], label=f"Est Vel {axis_labels_vel[i]}", color=f"C{i}", linestyle='--')
        sigma_vel = np.sqrt(vel_cov[:, i])
        ax_vel.fill_between(time, est_velocities[:, i] - sigma_vel, est_velocities[:, i] + sigma_vel, 
                            color=f"C{i}", alpha=0.2)
        ax_vel.set_title(f"Velocity [{axis_labels_vel[i]}] [m/s]")
        ax_vel.set_xlabel("Time [s]")
        ax_vel.set_ylabel("Velocity")
        ax_vel.grid(True)
        ax_vel.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

    # Plot Orientation
    fig_ori, axs_ori = plt.subplots(3, 1, figsize=(10, 12))
    fig_ori.suptitle("True Data vs Filter Estimates for Orientation")
    for i in range(3):
        ax_ori = axs_ori[i]
        ax_ori.plot(time, true_orientations[:, i], label=f"True Ori {axis_labels_ori[i]}", color=f"C{i}", linestyle='-')
        ax_ori.plot(time, est_orientations[:, i], label=f"Est Ori {axis_labels_ori[i]}", color=f"C{i}", linestyle='--')
        sigma_ori = np.sqrt(ori_cov[:, i])
        ax_ori.fill_between(time, est_orientations[:, i] - sigma_ori, est_orientations[:, i] + sigma_ori, 
                            color=f"C{i}", alpha=0.2)
        ax_ori.set_title(f"Orientation [{axis_labels_ori[i]}] [rad]")
        ax_ori.set_xlabel("Time [s]")
        ax_ori.set_ylabel("Orientation")
        ax_ori.grid(True)
        ax_ori.legend()

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()
