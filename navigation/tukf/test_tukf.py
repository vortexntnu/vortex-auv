import numpy as np
from tukf import TUKF
import tukf_class as ukf
from mpl_toolkits.mplot3d import Axes3D
import time
import math

import matplotlib.pyplot as plt

# Initialize UKF with StateQuat
initial_position = np.array([0.0, 0.0, 0.0])  # x, y, z
initial_velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
initial_quaternion = np.array([0.0, 0.0, 0.0])  # w, x, y, z (identity quaternion)
initial_angular_velocity = np.array([0.0, 0.0, 0.0])  # wx, wy, wz
initial_g_eta = np.array([1.2, 0.3, 0.3, 0.3])  # g_eta parameters
initial_intertia = np.array([0.68, 0.2, 0.1, 
                            0.2, 3.32, 0.2, 
                            0.1, 0.2, 3.34])
initial_damping = np.array([0.01, 0.01, 0.01,
                            0.01, 0.01, 0.01])
initla_added_mass = np.array([0.02, 0.02, 0.02,
                             0.02, 0.02, 0.02])

p_diag = np.concatenate([
    2*np.ones(3),  # x position
    2*np.ones(3),  # orientation
    2*np.ones(3),  # velocity
    2*np.ones(3),  # angular velocity
    2*np.ones(9),             # inertia
    2*np.ones(6),             # added mass
    2*np.ones(6),             # damping
    2*np.ones(4)             # g_eta
])

initial_covariance = np.diag(p_diag) 

state = ukf.AUVState(initial_position.copy(), initial_quaternion.copy(), initial_velocity.copy(), initial_angular_velocity.copy())
state.covariance = initial_covariance.copy()
state.inertia = np.array([0.58, 0.1, 0.05, 
                                0.1, 2.32, 0.1, 
                                0.01, 0.1, 2.34])
state.g_eta = np.array([1.1, 0.1, 0.1, 0.1])
state.damping = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
state.added_mass = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

real_state = ukf.AUVState(initial_position.copy(), initial_quaternion.copy(), initial_velocity.copy(), initial_angular_velocity.copy())
real_state.inertia = initial_intertia.copy()
real_state.g_eta = initial_g_eta.copy()
real_state.damping = initial_damping.copy()
real_state.added_mass = initla_added_mass.copy()

Q_diag = np.concatenate([
    0.01*np.ones(3),      # position
    0.2*np.ones(9),     # kinematic (η & ν)
    0.8*np.ones(9),      # inertia
    0.8*np.ones(6),      # added mass
    0.8*np.ones(6),      # damping
    0.8*np.ones(4),        # g_eta
])

UKF_model = TUKF(state, np.diag(Q_diag))  # Process noise covariance

def dvl_h(state: ukf.AUVState) -> 'ukf.MeasModel':
    H_matrix = np.zeros((3, 12))
    H_matrix[:, 6:9] = np.eye(3)
    z_i = ukf.MeasModel()
    z_i.measurement = np.dot(H_matrix, state.dynamic_part())
    return z_i

dvl_measurement = ukf.MeasModel(H=dvl_h)

def ang_h(state: ukf.AUVState) -> 'ukf.MeasModel':
    H_matrix = np.zeros((3, 12))
    H_matrix[:, 9:12] = np.eye(3)
    z_i = ukf.MeasModel()
    z_i.measurement = np.dot(H_matrix, state.dynamic_part())
    return z_i

ang_measurement = ukf.MeasModel(H=ang_h) 

# UKF parameters
dt = 0.01  # time step
sim_time = 50.0  # total simulation time
steps = int(sim_time / dt)

# Storage for trajectory
positions = np.zeros((steps, 3))
velocities = np.zeros((steps, 3))
quaternions = np.zeros((steps, 3))
angular_velocities = np.zeros((steps, 3))
okid_params = np.zeros((steps, 25))

# Storage for trajectory
positions_est = np.zeros((steps, 3))
velocities_est = np.zeros((steps, 3))
quaternions_est = np.zeros((steps, 3))
angular_velocities_est = np.zeros((steps, 3))
okid_params_est = np.zeros((steps, 25))

# ---------- user‑tunable manoeuvre parameters -----------------------------
SEG_DUR         = 10.0        # [s] duration of each phase
A_F_TRANSL      =  2.0        # [N]  translational force amplitude
A_T_ROT         =  1.0        # [N·m] rotational torque amplitude
# --------------------------------------------------------------------------

#  Helper: build the scripted sequence as (kind, axis_idx, sign)
#  kind = 'F' for force, 'T' for torque
#  axes: 0‑x (surge/roll), 1‑y (sway/pitch), 2‑z (heave/yaw)
sequence = [
    ('F', 2, +1),   # +z  (up)
    ('F', 2, -1),   # –z  (down)
    ('F', 1, +1),   # +y  (right)
    ('F', 1, -1),   # –y  (left / “back” sideways)
    ('F', 0, +1),   # +x  (forward)
    ('F', 0, -1),   # –x  (backward)
    ('T', 2, +1),   # +yaw (turn right)
    ('T', 2, -1),   # –yaw (turn left)
    ('T', 1, +1),   # +pitch (nose up)
    ('T', 1, -1),   # –pitch (nose down)
    ('T', 0, +1),   # +roll (starboard roll)
    ('T', 0, -1)    # –roll (port roll)
]

TOTAL_TIME = len(sequence) * SEG_DUR          # handy if you need it

def _half_sine(local_t: float, duration: float) -> float:
    """Smooth window: 0 → 1 → 0 over `duration` (half‑sine)."""
    return np.sin(np.pi * local_t / duration)

def control_inputs(t: float) -> tuple[np.ndarray, np.ndarray]:
    """
    Piecewise scripted test signal:
      – translations along z, y, x
      – rotations about z (yaw), y (pitch), x (roll)
    """
    # Default: no actuation
    F = np.zeros(3)
    T = np.zeros(3)

    # Past the last segment? keep everything zero
    idx = int(t // SEG_DUR)
    if idx >= len(sequence):
        return F, T

    # Time inside current segment
    tau = t - idx * SEG_DUR
    window = _half_sine(tau, SEG_DUR)          # 0‑to‑1‑to‑0 shape

    kind, axis, sgn = sequence[idx]

    if kind == 'F':
        F[axis] = sgn * A_F_TRANSL * window
    else:  # 'T'
        T[axis] = sgn * A_T_ROT * window

    return F, T

# Simulation loop
for i in range(steps):
    t = i * dt
    
    control_force, control_torque = control_inputs(t)
    
    control_input = np.concatenate((control_force, control_torque))

    # Propagate state using UKF prediction
    real_state = ukf.F_dynamics(real_state, dt, control_input)
    state = UKF_model.unscented_transform(state, control_input)
    
    if UKF_model.filter_failed:
        print("Filter failed, stopping simulation.")
        break

    if i % 5 == 0:
        # Simulate measurement update every 5 steps
        ang_measurement.measurement = np.array([
            real_state.angular_velocity[0],
            real_state.angular_velocity[1],
            real_state.angular_velocity[2]
        ]) + np.random.normal(0, 0.04, 3)

        ang_measurement.covariance = np.eye(3) * (0.03**2)  # Measurement noise covariance

        UKF_model.measurement_update(state, ang_measurement)
        state = UKF_model.posteriori_estimate(state, ang_measurement)

    if i % 10 == 0:
        # Simulate measurement update every 10 steps
        dvl_measurement.measurement = np.array([
            real_state.velocity[0],
            real_state.velocity[1],
            real_state.velocity[2]
        ]) + np.random.normal(0, 0.04, 3)  # Simulated measurement with noise
        
        # Simulate measurement covariance
        dvl_measurement.covariance = np.eye(3) * (0.03**2)  # Measurement noise covariance
         
        # Update UKF with measurement
        UKF_model.measurement_update(state, dvl_measurement)
        state = UKF_model.posteriori_estimate(state, dvl_measurement)

    # Store state for plotting
    positions[i] = real_state.position
    velocities[i] = real_state.velocity
    quaternions[i] = real_state.orientation
    angular_velocities[i] = real_state.angular_velocity
    okid_params[i] = real_state.okid_part()

    # Store estimated state for plotting
    positions_est[i] = state.position
    velocities_est[i] = state.velocity
    quaternions_est[i] = state.orientation
    angular_velocities_est[i] = state.angular_velocity
    okid_params_est[i] = state.okid_part()
    
    # Add small delay to simulate real-time execution
    time.sleep(0.001)
print(state.as_vector())
# Plotting
time_points = np.arange(0, sim_time, dt)

# 3D trajectory plot
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')
ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], 'b-', label='True Trajectory')
ax.plot(positions_est[:, 0], positions_est[:, 1], positions_est[:, 2], 'r--', label='Estimated Trajectory')
ax.scatter(positions[0, 0], positions[0, 1], positions[0, 2], c='g', marker='o', s=100, label='Start')
ax.scatter(positions[-1, 0], positions[-1, 1], positions[-1, 2], c='r', marker='o', s=100, label='End')
ax.set_xlabel('X Position')
ax.set_ylabel('Y Position')
ax.set_zlabel('Z Position')
ax.set_title('3D Trajectory')
ax.legend()

# Position plot
plt.figure(figsize=(12, 6))
plt.subplot(311)
plt.plot(time_points, positions[:, 0], 'b-', label='True')
plt.plot(time_points, positions_est[:, 0], 'r--', label='Estimated')
plt.ylabel('X Position')
plt.legend()
plt.subplot(312)
plt.plot(time_points, positions[:, 1], 'b-', label='True')
plt.plot(time_points, positions_est[:, 1], 'r--', label='Estimated')
plt.ylabel('Y Position')
plt.legend()
plt.subplot(313)
plt.plot(time_points, positions[:, 2], 'b-', label='True')
plt.plot(time_points, positions_est[:, 2], 'r--', label='Estimated')
plt.ylabel('Z Position')
plt.xlabel('Time (s)')
plt.legend()
plt.tight_layout()

# Velocity plot
plt.figure(figsize=(12, 6))
plt.subplot(311)
plt.plot(time_points, velocities[:, 0], 'b-', label='True')
plt.plot(time_points, velocities_est[:, 0], 'r--', label='Estimated')
plt.ylabel('X Velocity')
plt.legend()
plt.subplot(312)
plt.plot(time_points, velocities[:, 1], 'b-', label='True')
plt.plot(time_points, velocities_est[:, 1], 'r--', label='Estimated')
plt.ylabel('Y Velocity')
plt.legend()
plt.subplot(313)
plt.plot(time_points, velocities[:, 2], 'b-', label='True')
plt.plot(time_points, velocities_est[:, 2], 'r--', label='Estimated')
plt.ylabel('Z Velocity')
plt.xlabel('Time (s)')
plt.legend()
plt.tight_layout()

# Angular velocity plot
plt.figure(figsize=(12, 6))
plt.subplot(311)
plt.plot(time_points, angular_velocities[:, 0], 'b-', label='True')
plt.plot(time_points, angular_velocities_est[:, 0], 'r--', label='Estimated')
plt.ylabel('Roll Rate')
plt.legend()
plt.subplot(312)
plt.plot(time_points, angular_velocities[:, 1], 'b-', label='True')
plt.plot(time_points, angular_velocities_est[:, 1], 'r--', label='Estimated')
plt.ylabel('Pitch Rate')
plt.legend()
plt.subplot(313)
plt.plot(time_points, angular_velocities[:, 2], 'b-', label='True')
plt.plot(time_points, angular_velocities_est[:, 2], 'r--', label='Estimated')
plt.ylabel('Yaw Rate')
plt.xlabel('Time (s)')
plt.legend()
plt.tight_layout()

# OKID Inertia parameters plot (9 parameters)
plt.figure(figsize=(15, 10))
plt.suptitle('Inertia Parameters', fontsize=16)
for i in range(9):
    plt.subplot(3, 3, i+1)
    plt.plot(time_points, okid_params[:, i], 'b-', label='True')
    plt.plot(time_points, okid_params_est[:, i], 'r--', label='Estimated')
    plt.ylabel(f'Inertia[{i}]')
    if i >= 6:  # Add x-label only to bottom row
        plt.xlabel('Time (s)')
    plt.legend()
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust for suptitle

# OKID Added Mass parameters plot (6 parameters)
plt.figure(figsize=(15, 8))
plt.suptitle('Added Mass Parameters', fontsize=16)
for i in range(6):
    plt.subplot(2, 3, i+1)
    plt.plot(time_points, okid_params[:, i+9], 'b-', label='True')
    plt.plot(time_points, okid_params_est[:, i+9], 'r--', label='Estimated')
    plt.ylabel(f'Added Mass[{i}]')
    if i >= 3:  # Add x-label only to bottom row
        plt.xlabel('Time (s)')
    plt.legend()
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust for suptitle

# OKID Damping parameters plot (6 parameters)
plt.figure(figsize=(15, 8))
plt.suptitle('Damping Parameters', fontsize=16)
for i in range(6):
    plt.subplot(2, 3, i+1)
    plt.plot(time_points, okid_params[:, i+15], 'b-', label='True')
    plt.plot(time_points, okid_params_est[:, i+15], 'r--', label='Estimated')
    plt.ylabel(f'Damping[{i}]')
    if i >= 3:  # Add x-label only to bottom row
        plt.xlabel('Time (s)')
    plt.legend()
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust for suptitle

# OKID g_eta parameters plot (4 parameters)
plt.figure(figsize=(12, 8))
plt.suptitle('g_eta Parameters', fontsize=16)
for i in range(4):
    plt.subplot(2, 2, i+1)
    plt.plot(time_points, okid_params[:, i+21], 'b-', label='True')
    plt.plot(time_points, okid_params_est[:, i+21], 'r--', label='Estimated')
    plt.ylabel(f'g_eta[{i}]')
    plt.xlabel('Time (s)')
    plt.legend()
plt.tight_layout(rect=[0, 0, 1, 0.96])  # Adjust for suptitle

plt.show()

