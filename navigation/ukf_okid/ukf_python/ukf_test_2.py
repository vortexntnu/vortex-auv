import numpy as np
import ukf_okid_class as ukf
from ukf_okid import UKF
from mpl_toolkits.mplot3d import Axes3D
import time
import math
from ukf_utils import print_StateQuat, print_matrix

import matplotlib.pyplot as plt

# Initialize UKF with StateQuat
initial_position = np.array([0.0, 0.0, 0.0])  # x, y, z
initial_velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
initial_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # w, x, y, z (identity quaternion)
initial_angular_velocity = np.array([0.0, 0.0, 0.0])  # wx, wy, wz
initial_g_eta = np.array([1.2, 0.3, 0.3, 0.3])  # g_eta parameters
initial_covariance = np.eye(37) * 20.0  # Initial covariance matrix

# Create StateQuat object
state = ukf.StateQuat(initial_position, initial_quaternion, initial_velocity, initial_angular_velocity)
state.okid_params.g_eta = initial_g_eta
state.covariance = initial_covariance

real_state = ukf.StateQuat(initial_position, initial_quaternion, initial_velocity, initial_angular_velocity)
real_state.okid_params.g_eta = np.array([1.2, 0.3, 0.3, 0.3])

UKF_model = UKF(state, Q=10.0 * np.eye(37))  # Process noise covariance
dvl_measurement = ukf.MeasModel()

def ang_h(state: ukf.StateQuat) -> 'ukf.MeasModel':
    H_matrix = np.zeros((3, 13))
    H_matrix[:, 10:13] = np.eye(3)
    z_i = ukf.MeasModel()
    z_i.measurement = np.dot(H_matrix, state.dynamic_part())
    return z_i

ang_measurement = ukf.MeasModel(H=ang_h) 

# UKF parameters
dt = 0.01  # time step
sim_time = 1.0  # total simulation time
steps = int(sim_time / dt)

# Storage for trajectory
positions = np.zeros((steps, 3))
velocities = np.zeros((steps, 3))
quaternions = np.zeros((steps, 4))
angular_velocities = np.zeros((steps, 3))
okid_params = np.zeros((steps, 4))

# Storage for trajectory
positions_est = np.zeros((steps, 3))
velocities_est = np.zeros((steps, 3))
quaternions_est = np.zeros((steps, 4))
angular_velocities_est = np.zeros((steps, 3))
okid_params_est = np.zeros((steps, 4))

# Simulation loop
for i in range(steps):
    t = i * dt
    
    # Generate control input (slow sinusoidal signals)
    control_force = np.array([
        5 * np.sin(4.0 * t),
        10 * np.sin(3.0 * t),
        10 * np.sin(2.0 * t)
    ])
    
    control_torque = np.array([
        10 * np.sin(4.0 * t),
        10 * np.sin(4.0 * t),
        10 * np.sin(4.0 * t)
    ])
    
    control_input = np.concatenate((control_force, control_torque))

    # Propagate state using UKF prediction
    real_state = ukf.F_dynamics(real_state, dt, control_input)
    state = UKF_model.unscented_transform(state, control_input)
    if i % 5 == 0:
        # Simulate measurement update every 5 steps
        ang_measurement.measurement = np.array([
            real_state.angular_velocity[0],
            real_state.angular_velocity[1],
            real_state.angular_velocity[2]
        ]) + np.random.normal(0, 0.03, 3)
        # Simulate measurement covariance
        ang_measurement.covariance = np.eye(3) * 0.04  # Measurement noise covariance
        # Update UKF with measurement
        UKF_model.measurement_update(state, ang_measurement)
        state = UKF_model.posteriori_estimate(state, ang_measurement)

    if i % 10 == 0:
        # Simulate measurement update every 10 steps
        dvl_measurement.measurement = np.array([
            real_state.velocity[0],
            real_state.velocity[1],
            real_state.velocity[2]
        ]) + np.random.normal(0, 0.03, 3)  # Simulated measurement with noise
        
        # Simulate measurement covariance
        dvl_measurement.covariance = np.eye(3) * 0.04  # Measurement noise covariance
         
        # Update UKF with measurement
        UKF_model.measurement_update(state, dvl_measurement)
        state = UKF_model.posteriori_estimate(state, dvl_measurement)
    print_matrix(state.covariance)
    
    print("Determinant of covariance matrix:", np.linalg.det(state.covariance))
    # Store state for plotting
    positions[i] = real_state.position
    velocities[i] = real_state.velocity
    quaternions[i] = real_state.orientation
    angular_velocities[i] = real_state.angular_velocity
    okid_params[i] = real_state.okid_params.g_eta

    # Store estimated state for plotting
    positions_est[i] = state.position
    velocities_est[i] = state.velocity
    quaternions_est[i] = state.orientation
    angular_velocities_est[i] = state.angular_velocity
    okid_params_est[i] = state.okid_params.g_eta
    
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

# OKID g_eta plot
plt.figure(figsize=(12, 6))
plt.subplot(411)
plt.plot(time_points, okid_params[:, 0], 'b-', label='True')
plt.plot(time_points, okid_params_est[:, 0], 'r--', label='Estimated')
plt.ylabel('g_eta[0]')
plt.legend()
plt.subplot(412)
plt.plot(time_points, okid_params[:, 1], 'b-', label='True')
plt.plot(time_points, okid_params_est[:, 1], 'r--', label='Estimated')
plt.ylabel('g_eta[1]')
plt.legend()
plt.subplot(413)
plt.plot(time_points, okid_params[:, 2], 'b-', label='True')
plt.plot(time_points, okid_params_est[:, 2], 'r--', label='Estimated')
plt.ylabel('g_eta[2]')
plt.legend()
plt.subplot(414)
plt.plot(time_points, okid_params[:, 3], 'b-', label='True')
plt.plot(time_points, okid_params_est[:, 3], 'r--', label='Estimated')
plt.ylabel('g_eta[3]')
plt.xlabel('Time (s)')
plt.legend()
plt.tight_layout()

plt.show()

