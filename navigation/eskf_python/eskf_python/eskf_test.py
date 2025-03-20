
from eskf_python_class import StateEuler, StateQuat, MeasurementModel, Measurement
from eskf_python_utils import quat_to_euler
from eskf_test_utils import process_model, StateQuatModel
import numpy as np 
import matplotlib.pyplot as plt
from eskf_python_filter import ESKF
from scipy.stats import chi2

def simulate_eskf():

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
        0.5, 0.5, 0.5,        # Position
        0.2, 0.2, 0.2,        # Velocity
        0.2, 0.2, 0.2,        # Orientation
        0.00001, 0.00001, 0.00001,        # Acceleration bias
        0.00001, 0.00001, 0.00001,        # Gyro bias
        0.00001, 0.00001, 0.00001         # Gravity
    ])

    # Noise parameters
    Q = np.diag([
        (0.034**2) / dt, (0.034**2) / dt, (0.034**2) / dt,                 # Accelerometer noise
        (0.002**2) / dt, (0.002**2) / dt, (0.002**2) / dt,               # Gyroscope noise
        0.00001, 0.00001, 0.00001,                                   # Acceleration bias random walk
        0.00001, 0.00001, 0.00001                                       # Gyro bias random walk
    ])
    
    Hx = np.zeros((3, 19))
    Hx[0:3, 3:6] = np.eye(3)

    # Create filter object
    eskf = ESKF(Q, P0, Hx, true_state_init, 1e-13, 1e-13, dt)

    # Create measurement objects
    imu_data = Measurement()
    dvl_data = Measurement()

    # R matrix for DVL aiding
    dvl_data.aiding_covariance = np.diag([(0.01)**2, (0.01)**2, (0.01)**2])

    # Setup the process model for simulation of AUV
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
    # Two where made since there seems to be an issue with declaring two identical objects.
    new_state = StateQuatModel()
    new_state.position = np.array([0.1, 0.0, 0.0])
    new_state.velocity = np.array([0.1, 0.0, 0.0])

    new_state_prev = StateQuatModel()
    new_state_prev.position = np.array([0.1, 0.0, 0.0])
    new_state_prev.velocity = np.array([0.1, 0.0, 0.0])

    model.state_vector = new_state
    model.state_vector_prev = new_state_prev

    # Initialize arrays to store true and estimated states
    true_positions = np.zeros((num_steps, 3))
    true_orientations = np.zeros((num_steps, 3))
    true_velocities = np.zeros((num_steps, 3))

    est_positions = np.zeros((num_steps, 3))
    est_orientations = np.zeros((num_steps, 3))
    est_velocities = np.zeros((num_steps, 3))

    # covariance arrays
    pos_cov = np.zeros((num_steps, 3))   
    vel_cov = np.zeros((num_steps, 3))   
    ori_cov = np.zeros((num_steps, 3))   

    prev_velocity = np.zeros(3)
    u = lambda t: np.array([
        0.5 * np.sin(0.1 * t),
        0.5 * np.sin(0.1 * t + 0.3),
        0.5 * np.sin(0.1 * t + 0.6),
        0.05 * np.cos(0.1 * t),
        0.05 * np.cos(0.1 * t + 0.3),
        0.05 * np.cos(0.1 * t + 0.6)
    ])

    NIS_list = []
    NIS_value = 0.0

    # Sim
    for step in range(num_steps):
        t = step * dt

        model.Control_input = u(t)
        model.model_prediction(new_state)
        new_state = model.euler_forward()

        imu_data.acceleration = ((new_state.velocity - prev_velocity) / dt) + np.random.normal(0, 0.13, 3)
        imu_data.angular_velocity = new_state.angular_velocity + np.random.normal(0, 0.13, 3)

        eskf.imu_update(imu_data)

        if step % 20 == 0:
            dvl_data.aiding = new_state.velocity + np.random.normal(0, 0.01, 3)
            NIS_value = eskf.dvl_update(dvl_data)
            NIS_list.append(NIS_value)

        true_positions[step, :] = np.copy(new_state.position)
        true_orientations[step, :] = quat_to_euler(np.copy(new_state.orientation))
        true_velocities[step, :] = np.copy(new_state.velocity)

        est_positions[step, :] = np.copy(eskf.nom_state.position)
        est_orientations[step, :] = quat_to_euler(np.copy(eskf.nom_state.orientation))
        est_velocities[step, :] = np.copy(eskf.nom_state.velocity)

        P_diag = np.diag(eskf.error_state.covariance)
        pos_cov[step, :] = P_diag[0:3]
        vel_cov[step, :] = P_diag[3:6]
        ori_cov[step, :] = P_diag[6:9]

        prev_velocity = new_state.velocity
        model.state_vector_prev = new_state

    return time, true_positions, true_orientations, true_velocities, est_positions, est_orientations, est_velocities, pos_cov, vel_cov, ori_cov, NIS_list

time, true_positions, true_orientations, true_velocities, est_positions, est_orientations, est_velocities, pos_cov, vel_cov, ori_cov, _ = simulate_eskf()

# Plotting
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


### _______ NIS AND NEES _______

num_simulations = 10 
NIS_runs = []

for sim in range(num_simulations):
    time, true_positions, true_orientations, true_velocities, \
    est_positions, est_orientations, est_velocities, \
    pos_cov, vel_cov, ori_cov, NIS_list = simulate_eskf()

    NIS_runs.append(np.array(NIS_list))

NIS_runs = np.vstack(NIS_runs)  
ANIS = np.mean(NIS_runs, axis=0)

measurement_dimension = 3

chi2_lower = chi2.ppf(0.025, measurement_dimension) / num_simulations
chi2_upper = chi2.ppf(0.975, measurement_dimension) / num_simulations

time_steps = np.arange(len(ANIS)) * 0.01 * 20 

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(time_steps, ANIS, label="ANIS", color="C0")
ax.axhline(chi2_lower, color="C1", linestyle="--", label="95% CI Lower")
ax.axhline(chi2_upper, color="C2", linestyle="--", label="95% CI Upper")
ax.set_title("Average Normalized Innovation Squared (ANIS)")
ax.set_xlabel("Time [s]")
ax.set_ylabel("ANIS")
ax.grid(True)
ax.legend()

plt.tight_layout()
plt.show()