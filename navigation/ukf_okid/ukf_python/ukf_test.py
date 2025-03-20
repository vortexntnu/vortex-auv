from ukf_okid import UKF
from ukf_okid_class import StateQuat, process_model, MeasModel
import numpy as np
import time
import matplotlib.pyplot as plt
from ukf_utils import print_StateQuat_list, print_StateQuat
from ukf_okid_class import quaternion_super_product, quat_to_euler, mean_set, covariance_set



def add_quaternion_noise(q, noise_std):

    noise = np.random.normal(0, noise_std, 3)

    theta = np.linalg.norm(noise)

    if theta > 0:

        axis = noise / theta

        q_noise = np.hstack((np.cos(theta/2), np.sin(theta/2) * axis))

    else:

        q_noise = np.array([1.0, 0.0, 0.0, 0.0])

    q_new = quaternion_super_product(q, q_noise)

    return q_new / np.linalg.norm(q_new)


if __name__ == '__main__':

    # Define a mean StateQuat
    mean_state = StateQuat()
    mean_state.position = np.array([1.0, 2.0, 3.0])
    mean_state.orientation = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion
    mean_state.velocity = np.array([0.5, 0.5, 0.5])
    mean_state.angular_velocity = np.array([0.1, 0.1, 0.1])

    test_state = StateQuat()
    test_state.position = np.array([1.0, 1.0, 1.0])
    test_state.orientation = np.array([0.0, 1.0, 0.0, 0.0])  # Quaternion
    test_state.velocity = np.array([0.2, 0.2, 0.2])
    test_state.angular_velocity = np.array([0.2, 0.2, 0.2])

    # Create a set with only one element
    state_set = list()
    state_set.append(test_state)
    print(len(state_set))

    # Compute the mean
    mean = mean_set(state_set)

    # Compute the covariance
    mean_state.covariance = covariance_set(state_set, mean_state)

    # Print the results
    print("Mean State:")
    print_StateQuat(mean_state)

    # # Create initial state vector and covariance matrix.
    # x0 = np.zeros(13)
    # x0[0:3] = [0.3, 0.3, 0.3]
    # x0[3] = 1
    # x0[7:10] = [0.2, 0.2, 0.2]
    # dt = 0.01
    # R = (0.01) * np.eye(3)
    
    # Q = 0.00015 * np.eye(12)
    # P0 = np.eye(12) * 0.0001

    # model = process_model()
    # model.dt = 0.01
    # model.mass_interia_matrix = np.array([
    #     [30.0, 0.0, 0.0, 0.0, 0.0, 0.6],
    #     [0.0, 30.0, 0.0, 0.0, -0.6, 0.3],
    #     [0.0, 0.0, 30.0, 0.6, 0.3, 0.0],
    #     [0.0, 0.0, 0.6, 0.68, 0.0, 0.0],
    #     [0.0, -0.6, 0.3, 0.0, 3.32, 0.0],
    #     [0.6, 0.3, 0.0, 0.0, 0.0, 3.34]
    # ])
    # model.m = 30.0
    # model.r_b_bg = np.array([0.01, 0.0, 0.02])
    # model.inertia = np.diag([0.68, 3.32, 3.34])
    # model.damping_linear = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    # model.damping_nonlinear = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    # model.added_mass = np.diag([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])

    # model_ukf = process_model()
    # model_ukf.dt = 0.01
    # model_ukf.mass_interia_matrix = np.array([
    #     [30.0, 0.0, 0.0, 0.0, 0.0, 0.6],
    #     [0.0, 30.0, 0.0, 0.0, -0.6, 0.3],
    #     [0.0, 0.0, 30.0, 0.6, 0.3, 0.0],
    #     [0.0, 0.0, 0.6, 0.68, 0.0, 0.0],
    #     [0.0, -0.6, 0.3, 0.0, 3.32, 0.0],
    #     [0.6, 0.3, 0.0, 0.0, 0.0, 3.34]
    # ])
    # model_ukf.m = 30.0
    # model_ukf.r_b_bg = np.array([0.01, 0.0, 0.02])
    # model_ukf.inertia = np.diag([0.68, 3.32, 3.34])
    # model_ukf.damping_linear = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])
    # model_ukf.damping_nonlinear = np.array([0.3, 0.3, 0.3, 0.3, 0.3, 0.3])
    # model_ukf.added_mass = np.diag([1.0, 1.0, 1.0, 2.0, 2.0, 2.0])

    # # Simulation parameters
    # simulation_time = 20  # seconds
    # num_steps = int(simulation_time / dt)

    # # Initialize a dummy StateQuat.
    # new_state = StateQuat()
    # new_state.fill_states(x0)
    # new_state.covariance = P0

    # test_state_x = StateQuat()
    # test_state_x.fill_states(x0)
    # test_state_x.covariance = P0

    # # Initialize a estimated state
    # estimated_state = StateQuat()
    # estimated_state.fill_states(x0)
    # estimated_state.covariance = P0

    # # Initialize a estimated state
    # noisy_state = StateQuat()
    # noisy_state.fill_states(x0)
    # noisy_state.covariance = P0

    # measurment_model = MeasModel()
    # measurment_model.measurement = np.array([0.0, 0.0, 0.0])
    # measurment_model.covariance = R 

    # # Initialize arrays to store the results
    # positions = np.zeros((num_steps, 3))
    # orientations = np.zeros((num_steps, 3))
    # velocities = np.zeros((num_steps, 3))
    # angular_velocities = np.zeros((num_steps, 3))

    # # Initialize arrays to store the estimates
    # positions_est = np.zeros((num_steps, 3))
    # orientations_est = np.zeros((num_steps, 3))
    # velocities_est = np.zeros((num_steps, 3))
    # angular_velocities_est = np.zeros((num_steps, 3))

    # # Initialize the okid params
    # okid_params = np.zeros((num_steps, 21))

    # model.state_vector_prev = new_state
    # model.state_vector = new_state

    # model_ukf.state_vector_prev = test_state_x
    # model_ukf.state_vector = test_state_x

    # # initialize the ukf
    # ukf = UKF(model_ukf, x0, P0, Q, R)

    # elapsed_times = []

    # u = lambda t: np.array([2 * np.sin(1 * t), 2 * np.sin(1 * t), 2 * np.sin(1 * t), 0.2 * np.cos(1 * t), 0.2 * np.cos(1 * t), 0.2 * np.cos(1 * t)])

    # # Run the simulation
    # for step in range(num_steps):
    #     # Insert control input
    #     model.Control_input = u(step * dt)
    #     model_ukf.Control_input = u(step * dt)

    #     # Perform the unscented transform
    #     model.model_prediction(new_state)
    #     new_state = model.euler_forward()

    #     # Adding noise in the state vector
    #     estimated_state.position = estimated_state.position # + np.random.normal(0, 0.01, 3)
    #     estimated_state.orientation = estimated_state.orientation #add_quaternion_noise(estimated_state.orientation, 0.01)
    #     estimated_state.velocity = estimated_state.velocity # + np.random.normal(0, 0.01, 3)
    #     estimated_state.angular_velocity = estimated_state.angular_velocity # + np.random.normal(0, 0.01, 3)

    #     start_time = time.time()
    #     estimated_state = ukf.unscented_transform(estimated_state)
    #     elapsed_time = time.time() - start_time
    #     elapsed_times.append(elapsed_time)

    #     if step % 10 == 0:
    #         measurment_model.measurement = new_state.velocity # + np.random.normal(0, 0.01, 3)
    #         meas_update, covariance_matrix = ukf.measurement_update(estimated_state, measurment_model)
    #         estimated_state = ukf.posteriori_estimate(estimated_state, covariance_matrix, measurment_model, meas_update)


    #     positions[step, :] = new_state.position
    #     orientations[step, :] = quat_to_euler(new_state.orientation)
    #     velocities[step, :] = new_state.velocity
    #     angular_velocities[step, :] = new_state.angular_velocity

    #     positions_est[step, :] = estimated_state.position
    #     orientations_est[step, :] = quat_to_euler(estimated_state.orientation)
    #     velocities_est[step, :] = estimated_state.velocity
    #     angular_velocities_est[step, :] = estimated_state.angular_velocity
        
    #     # Update the state for the next iteration
    #     model.state_vector_prev = new_state

    # print('Average elapsed time: ', np.mean(elapsed_times))
    # print('Max elapsed time: ', np.max(elapsed_times))
    # print('Min elapsed time: ', np.min(elapsed_times))
    # print('median elapsed time: ', np.median(elapsed_times))
    # # Plot the results
    # time = np.linspace(0, simulation_time, num_steps)

    # # Plot positions
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(time, positions[:, 0], label='True')
    # plt.plot(time, positions_est[:, 0], label='Estimated')
    # plt.title('Position X')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Position X [m]')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(time, positions[:, 1], label='True')
    # plt.plot(time, positions_est[:, 1], label='Estimated')
    # plt.title('Position Y')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Position Y [m]')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(time, positions[:, 2], label='True')
    # plt.plot(time, positions_est[:, 2], label='Estimated')
    # plt.title('Position Z')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Position Z [m]')
    # plt.legend()

    # plt.tight_layout()
    # plt.show()

    # # Plot orientations (Euler angles)
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(time, orientations[:, 0], label='True')
    # plt.plot(time, orientations_est[:, 0], label='Estimated')
    # plt.title('Orientation Roll')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Roll [rad]')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(time, orientations[:, 1], label='True')
    # plt.plot(time, orientations_est[:, 1], label='Estimated')
    # plt.title('Orientation Pitch')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Pitch [rad]')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(time, orientations[:, 2], label='True')
    # plt.plot(time, orientations_est[:, 2], label='Estimated')
    # plt.title('Orientation Yaw')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Yaw [rad]')
    # plt.legend()

    # plt.tight_layout()
    # plt.show()

    # # Plot velocities
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(time, velocities[:, 0], label='True')
    # plt.plot(time, velocities_est[:, 0], label='Estimated')
    # plt.title('Velocity X')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Velocity X [m/s]')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(time, velocities[:, 1], label='True')
    # plt.plot(time, velocities_est[:, 1], label='Estimated')
    # plt.title('Velocity Y')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Velocity Y [m/s]')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(time, velocities[:, 2], label='True')
    # plt.plot(time, velocities_est[:, 2], label='Estimated')
    # plt.title('Velocity Z')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Velocity Z [m/s]')
    # plt.legend()

    # plt.tight_layout()
    # plt.show()

    # # Plot angular velocities
    # plt.figure()
    # plt.subplot(3, 1, 1)
    # plt.plot(time, angular_velocities[:, 0], label='True')
    # plt.plot(time, angular_velocities_est[:, 0], label='Estimated')
    # plt.title('Angular Velocity X')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Angular Velocity X [rad/s]')
    # plt.legend()

    # plt.subplot(3, 1, 2)
    # plt.plot(time, angular_velocities[:, 1], label='True')
    # plt.plot(time, angular_velocities_est[:, 1], label='Estimated')
    # plt.title('Angular Velocity Y')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Angular Velocity Y [rad/s]')
    # plt.legend()

    # plt.subplot(3, 1, 3)
    # plt.plot(time, angular_velocities[:, 2], label='True')
    # plt.plot(time, angular_velocities_est[:, 2], label='Estimated')
    # plt.title('Angular Velocity Z')
    # plt.xlabel('Time [s]')
    # plt.ylabel('Angular Velocity Z [rad/s]')
    # plt.legend()

    # plt.tight_layout()
    # plt.show()