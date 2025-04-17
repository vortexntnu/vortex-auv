import numpy as np
import matplotlib.pyplot as plt
from ukf_utils import print_StateQuat
# Import your classes and functions.
# Adjust the import paths as necessary based on your module organization.
from ukf_okid_class import (
    StateQuat,
    MeasModel,
    iterative_quaternion_mean_statequat,
    mean_set,
    mean_measurement,
    covariance_set,
    covariance_measurement,
    cross_covariance,
    quaternion_super_product,
    quaternion_error,
    quat_to_euler,
    quat_norm,
)

# For testing, define a function to create a StateQuat with small perturbations.
def create_statequat(base_vector, position_perturbation, orientation_perturbation, velocity_perturbation, angular_velocity_perturbation):
    """
    Creates a StateQuat object.
      - base_vector: 1D numpy array for base state (13 elements:
          position (3), quaternion (4), velocity (3), angular_velocity (3))
      - ..._perturbation: small perturbation vector to be added to each respective component.
    Returns a StateQuat.
    """
    state = StateQuat()
    # Base state
    state.position = base_vector[0:3] + position_perturbation
    # For orientation, perturb by adding a small rotation:
    base_quat = base_vector[3:7]
    noise_angle = np.linalg.norm(orientation_perturbation)
    if noise_angle < 1e-8:
        noise_quat = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        noise_axis = orientation_perturbation / noise_angle
        noise_quat = np.concatenate(([np.cos(noise_angle/2)], np.sin(noise_angle/2) * noise_axis))
    state.orientation = quat_norm(quaternion_super_product(base_quat, noise_quat))
    state.velocity = base_vector[7:10] + velocity_perturbation
    state.angular_velocity = base_vector[10:13] + angular_velocity_perturbation
    
    # For the augmented parameters (OKID parameters), set a 21-element vector: 
    # 9 for inertia, 6 for added_mass, and 6 for damping_linear.
    state.okid_params.fill(np.concatenate((np.zeros(9), np.zeros(6), np.zeros(6))))
    
    # Set a default covariance (33x33 for the extended state)
    state.covariance = np.eye(33) * 0.01
    return state

# Test functions for state statistics
def test_state_statistics():
    # Define a base state vector (13 elements: position, quaternion, velocity, angular_velocity)
    base_vector = np.zeros(13)
    base_vector[0:3] = np.array([1.0, 2.0, 3.0])
    base_vector[3:7] = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion
    base_vector[7:10] = np.array([0.1, 0.2, 0.3])
    base_vector[10:13] = np.array([0.01, 0.02, 0.03])
    
    # Create a list of StateQuat objects with small random perturbations.
    np.random.seed(42)
    state_list = []
    num_states = 10
    for _ in range(num_states):
        pos_noise = np.random.normal(0, 0.05, 3)
        ori_noise = np.random.normal(0, 0.01, 3)
        vel_noise = np.random.normal(0, 0.02, 3)
        ang_vel_noise = np.random.normal(0, 0.005, 3)
        state_list.append(create_statequat(base_vector, pos_noise, ori_noise, vel_noise, ang_vel_noise))
    
    # Compute the state mean using mean_set.
    mean_state_vec = mean_set(state_list)
    print("Computed mean state vector:")
    print(mean_state_vec)
    
    # Compute the covariance of the states.
    cov_state = covariance_set(state_list, mean_state_vec)
    print("Computed state covariance matrix:")
    print(cov_state)
    
    # Check symmetry of the covariance:
    asym_error = np.linalg.norm(cov_state - cov_state.T)
    print("Covariance symmetry error (should be near 0):", asym_error)
    
    # Check eigenvalues for positive semidefiniteness:
    eigvals = np.linalg.eigvals(cov_state)
    print("Eigenvalues of state covariance:")
    print(eigvals)
    
def test_measurement_statistics():
    # Create a list of measurement objects (MeasModel) with measurements in R^3.
    np.random.seed(24)
    meas_list = []
    num_meas = 10
    base_meas = np.array([1.0, 2.0, 3.0])
    for _ in range(num_meas):
        noise = np.random.normal(0, 0.1, 3)
        meas = MeasModel()
        meas.measurement = base_meas + noise
        meas_list.append(meas)
    
    # Compute the measurement mean.
    mean_meas = mean_measurement(meas_list)
    print("Computed measurement mean:")
    print(mean_meas)
    
    # Compute the measurement covariance.
    cov_meas = covariance_measurement(meas_list, mean_meas)
    print("Computed measurement covariance:")
    print(cov_meas)
    
    # Check symmetry and eigenvalues.
    asym_error = np.linalg.norm(cov_meas - cov_meas.T)
    print("Measurement covariance symmetry error:", asym_error)
    eigvals = np.linalg.eigvals(cov_meas)
    print("Eigenvalues of measurement covariance:")
    print(eigvals)

def test_cross_covariance():
    # Create a set of StateQuat and corresponding MeasModel objects.
    np.random.seed(99)
    num = 10
    state_list = []
    meas_list = []
    base_vector = np.zeros(13)
    base_vector[0:3] = np.array([0.5, 1.0, -0.5])
    base_vector[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    base_vector[7:10] = np.array([0.05, 0.1, 0.15])
    base_vector[10:13] = np.array([0.005, 0.01, 0.015])
    
    for _ in range(num):
        pos_noise = np.random.normal(0, 0.02, 3)
        ori_noise = np.random.normal(0, 0.005, 3)
        vel_noise = np.random.normal(0, 0.01, 3)
        ang_vel_noise = np.random.normal(0, 0.002, 3)
        state = create_statequat(base_vector, pos_noise, ori_noise, vel_noise, ang_vel_noise)
        state_list.append(state)
        
        # Generate a measurement from each state (e.g., state velocity plus noise).
        meas = MeasModel()
        meas.measurement = state.velocity + np.random.normal(0, 0.01, 3)
        meas_list.append(meas)
    
    # Compute the state mean and measurement mean as vectors.
    mean_state_vec = mean_set(state_list)
    mean_meas = mean_measurement(meas_list)
    
    cross_cov = cross_covariance(state_list, mean_state_vec, meas_list, mean_meas)
    print("Computed cross-covariance between state and measurement:")
    print(cross_cov)

import time
import numpy as np
import matplotlib.pyplot as plt

# Import your classes and functions.
from ukf_okid_class import (
    StateQuat,
    MeasModel,
    iterative_quaternion_mean_statequat,
    mean_set,
    mean_measurement,
    covariance_set,
    covariance_measurement,
    cross_covariance,
    quaternion_super_product,
    quaternion_error,
    quat_norm,
)
from ukf_okid import UKF
from ukf_okid_class import process_model, okid_process_model  # Your process model classes

############################################
# Helper function to create a StateQuat with perturbations.
############################################
def create_statequat(base_vector, pos_noise, ori_noise, vel_noise, ang_vel_noise):
    """
    Create a StateQuat object from a base vector (13 elements:
      position (3), quaternion (4), velocity (3), angular_velocity (3))
    plus additive noise on each component.
    
    For the OKID parameters, we assume a 21-element vector:
      - first 9: inertia,
      - next 6: added_mass,
      - last 6: damping_linear.
    """
    state = StateQuat()
    state.position = base_vector[0:3] + pos_noise
    base_quat = base_vector[3:7]
    noise_angle = np.linalg.norm(ori_noise)
    if noise_angle < 1e-8:
        noise_quat = np.array([1.0, 0.0, 0.0, 0.0])
    else:
        noise_axis = ori_noise / noise_angle
        noise_quat = np.concatenate(([np.cos(noise_angle/2)],
                                     np.sin(noise_angle/2) * noise_axis))
    state.orientation = quat_norm(quaternion_super_product(base_quat, noise_quat))
    state.velocity = base_vector[7:10] + vel_noise
    state.angular_velocity = base_vector[10:13] + ang_vel_noise

    # Set OKID parameters to exactly 21 elements (9,6,6)
    state.okid_params.fill(np.concatenate((np.array([0.0, 0.0, 0.3, 0.0, 0.0, 3.3, 0.0, 0.0, 3.3]), np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]), np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]))))
    # Set an initial covariance (33x33) for the full augmented state.
    state.covariance = np.eye(33) * 0.02
    return state

############################################
# Full Filter Simulation Test
############################################
def run_ukf_simulation():
    dt = 0.01               # Time step for simulation [s]
    simulation_time = 10    # Total simulation time in seconds
    num_steps = int(simulation_time / dt)
    
    # Define a base state vector (13 elements: pos, quat, vel, ang_vel)
    base_vector = np.zeros(13)
    base_vector[0:3] = np.array([0.0, 0.0, 0.0]) 
    base_vector[3:7] = np.array([1.0, 0.0, 0.0, 0.0])  # identity quaternion
    base_vector[7:10] = np.array([0.1, 0.0, 0.0])       # small velocity in x
    base_vector[10:13] = np.array([0.0, 0.0, 0.0])
    
    # Define initial covariance for state (33x33)
    P0 = np.eye(33)
    P0[0:3, 0:3] = np.eye(3) * 0.01  # position
    P0[3:6, 3:6] = np.eye(3) * 0.01  # orientation error (quaternion)
    P0[6:9, 6:9] = np.eye(3) * 0.01  # velocity
    P0[9:12, 9:12] = np.eye(3) * 0.01 # angular velocity
    P0[12:33, 12:33] = np.eye(21) * 0.001 # OKID parameters

    # Define process noise covariance Q (33x33)
    Q = np.zeros((33, 33))
    Q[0:3, 0:3] = np.eye(3)*0.001     # for position
    Q[3:6, 3:6] = np.eye(3)*0.001     # for orientation error (represented with Euler angles)
    Q[6:9, 6:9] = np.eye(3)*0.001     # for velocity
    Q[9:12, 9:12] = np.eye(3)*0.001    # for angular velocity
    Q[12:33, 12:33] = np.eye(21) * 0.001 # OKID parameters


    G = np.zeros((33, 12))
    G[0:3, 0:3] = np.eye(3)
    G[3:6, 3:6] = np.eye(3)
    G[6:9, 6:9] = np.eye(3)
    G[9:12, 9:12] = np.eye(3)

    # Measurement noise covariance R (3x3), assume measurement is velocity
    R = np.eye(3) * 0.01

    # Create a simulation process model and an independent UKF process model.
    sim_model = process_model()
    sim_model.dt = dt
    sim_model.mass_interia_matrix = np.array([
        [30.0, 0.0, 0.0, 0.0, 0.0, 0.6],
        [0.0, 30.0, 0.0, 0.0, -0.6, 0.3],
        [0.0, 0.0, 30.0, 0.6, 0.3, 0.0],
        [0.0, 0.0, 0.6, 0.68, 0.0, 0.0],
        [0.0, -0.6, 0.3, 0.0, 3.32, 0.0],
        [0.6, 0.3, 0.0, 0.0, 0.0, 3.34],
    ])
    sim_model.m = 30.0
    sim_model.r_b_bg = np.array([0.01, 0.0, 0.02])
    sim_model.inertia = np.diag([0.68, 3.32, 3.34])
    sim_model.damping_linear = np.array([0.1]*6)
    sim_model.added_mass = np.array([1.0,1.0,1.0,2.0,2.0,2.0])

    # UKF process model copy:
    ukf_model = okid_process_model()
    ukf_model.dt = dt
    ukf_model.mass_interia_matrix = sim_model.mass_interia_matrix.copy()
    ukf_model.m = sim_model.m
    ukf_model.r_b_bg = sim_model.r_b_bg.copy()
    ukf_model.inertia = sim_model.inertia.copy()
    ukf_model.damping_linear = sim_model.damping_linear.copy()
    ukf_model.added_mass = sim_model.added_mass.copy()

    # Initialize true state and filter state.
    true_state = create_statequat(base_vector,
                                  np.zeros(3),
                                  np.zeros(3),
                                  np.zeros(3),
                                  np.zeros(3))
    true_state.covariance = P0.copy()
    
    filter_state = create_statequat(base_vector,
                                    np.zeros(3),
                                    np.zeros(3),
                                    np.zeros(3),
                                    np.zeros(3))
    filter_state.covariance = P0.copy()

    # Initialize measurement model (for example, measuring velocity only)
    meas_model = MeasModel()
    meas_model.covariance = R.copy()

    # Initialize UKF.
    ukf = UKF(ukf_model, true_state, P0.copy(), Q.copy(), G.copy())

    # Arrays to store time histories.
    pos_true_hist = np.zeros((num_steps, 3))
    pos_est_hist = np.zeros((num_steps, 3))
    vel_true_hist = np.zeros((num_steps, 3))
    vel_est_hist = np.zeros((num_steps, 3))
    euler_true_hist = np.zeros((num_steps, 3))
    euler_est_hist = np.zeros((num_steps, 3))
    time_array = np.linspace(0, simulation_time, num_steps)

    # Control input function (example: oscillatory in all directions)
    def control_input(t):
        return np.array([
            2*np.sin(t),
            2*np.sin(t+0.5),
            2*np.sin(t+1.0),
            0.2*np.cos(t),
            0.2*np.cos(t+0.5),
            0.2*np.cos(t+1.0)
        ])

    # Set previous states.
    sim_model.state_vector_prev = true_state
    sim_model.state_vector = true_state
    ukf_model.state_vector_prev = filter_state
    ukf_model.state_vector = filter_state

    # Lists for timing diagnostics.
    ukf_transform_times = []
    ukf_update_times = []
    
    # Simulation loop.
    for i in range(num_steps):
        t_current = i*dt
        
        # Update control inputs.
        sim_model.Control_input = control_input(t_current)
        ukf_model.Control_input = control_input(t_current)
        
        # Propagate true state using the simulation model.
        sim_model.model_prediction(true_state)
        true_state = sim_model.euler_forward()
        
        # Create a measurement from true state.
        # Here we assume we measure velocity plus noise.
        meas_noise = np.random.normal(0, 0.01, 3)
        meas_model.measurement = true_state.velocity + meas_noise

        # UKF prediction: unscented transform.
        start = time.time()
        filter_state = ukf.unscented_transform(filter_state)
        ukf_transform_times.append(time.time() - start)
        
        # UKF measurement update every few steps.
        if i % 5 == 0:
            try:
                start = time.time()
                ukf.measurement_update(filter_state, meas_model)
                filter_state = ukf.posteriori_estimate(filter_state, meas_model)
                ukf_update_times.append(time.time() - start)
            except np.linalg.LinAlgError:
                # If matrix is not PD, add jitter.
                filter_state.covariance += np.eye(filter_state.covariance.shape[0])*1e-6
        
        # Store true and estimated state for diagnostics.
        pos_true_hist[i, :] = true_state.position
        pos_est_hist[i, :] = filter_state.position
        vel_true_hist[i, :] = true_state.velocity
        vel_est_hist[i, :] = filter_state.velocity
        # Convert quaternion to Euler angles for visualization.
        # Assumes you have a function quat_to_euler.
        euler_true_hist[i, :] = quat_to_euler(true_state.orientation)
        euler_est_hist[i, :] = quat_to_euler(filter_state.orientation)
        
        # Update previous states.
        sim_model.state_vector_prev = true_state
        ukf_model.state_vector_prev = filter_state

    # Print timing diagnostics.
    print("Average unscented transform time:", np.mean(ukf_transform_times))
    print("Average measurement update time:", np.mean(ukf_update_times))
    
    # Compute error metrics.
    pos_error = np.linalg.norm(pos_true_hist - pos_est_hist, axis=1)
    vel_error = np.linalg.norm(vel_true_hist - vel_est_hist, axis=1)
    euler_error = np.linalg.norm(euler_true_hist - euler_est_hist, axis=1)
    print("Average position error:", np.mean(pos_error))
    print("Average velocity error:", np.mean(vel_error))
    print("Average orientation (Euler) error:", np.mean(euler_error))
    
    # Plot estimated vs true trajectory (positions).
    plt.figure(figsize=(10,8))
    plt.subplot(3,1,1)
    plt.plot(time_array, pos_true_hist[:,0], label="True X")
    plt.plot(time_array, pos_est_hist[:,0], label="Est X", linestyle="--")
    plt.legend()
    plt.title("Position X")
    
    plt.subplot(3,1,2)
    plt.plot(time_array, pos_true_hist[:,1], label="True Y")
    plt.plot(time_array, pos_est_hist[:,1], label="Est Y", linestyle="--")
    plt.legend()
    plt.title("Position Y")
    
    plt.subplot(3,1,3)
    plt.plot(time_array, pos_true_hist[:,2], label="True Z")
    plt.plot(time_array, pos_est_hist[:,2], label="Est Z", linestyle="--")
    plt.legend()
    plt.title("Position Z")
    plt.tight_layout()
    plt.show()
    
    # Plot errors.
    plt.figure(figsize=(10,4))
    plt.plot(time_array, pos_error, label="Position Error")
    plt.plot(time_array, vel_error, label="Velocity Error")
    plt.plot(time_array, euler_error, label="Euler Angle Error")
    plt.legend()
    plt.title("Error Metrics over Time")
    plt.xlabel("Time (s)")
    plt.ylabel("Error magnitude")
    plt.show()

# You can also test the individual statistics functions separately:
def run_diagnostics():
    print("Testing state mean and covariance computation:")
    # Call your pre-written tests:
    # (Assuming these functions—test_state_statistics, test_measurement_statistics, test_cross_covariance—are defined above)
    test_state_statistics()
    print("\nTesting measurement mean and covariance computation:")
    test_measurement_statistics()
    print("\nTesting cross-covariance computation:")
    test_cross_covariance()

if __name__ == '__main__':
    # First, run the diagnostics on the mean/covariance functions.
    run_diagnostics()
    
    # Then run the full UKF simulation test.
    print("\nRunning full UKF simulation test:")
    run_ukf_simulation()


