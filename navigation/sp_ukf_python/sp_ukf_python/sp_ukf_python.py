
import numpy as np
from scipy.linalg import expm
from sp_ukf_python_class import StateVector_euler, StateVector_quaternion
from sp_ukf_python_utils import quaternion_super_product, skew_symmetric


class ErrorStateUnscentedKalmanFilter:
    def __init__(
        self,
        P_ab: float,
        P_wb: float,
        Q: np.ndarray,
        lever_arm: np.array,
        R: np.ndarray,
        g: float,
        dt: float,
    ) -> None:
        self.P_ab = P_ab
        self.P_wb = P_wb
        self.Q_process_noise = Q
        self.lever_arm = lever_arm
        self.R = R
        self.g = np.array([0, 0, g])
        self.dt = dt
        self.y_i = np.zeros((15, 2 * 15))
        self.W = np.zeros(2 * 15 + 1)

    def mean_set(self, set: np.ndarray) -> np.ndarray:
        """Calculates the mean of a set of values.

        Args:
            set (np.ndarray): The set of values.

        Returns:
            np.ndarray: The mean of the set.
        """
        # Define the number of sigma points based on columns
        n = set.shape[0]
        mean_value = np.zeros(n)

        for i in range(2 * n + 1):
            mean_value += (1/(2 * n + 1)) * set[:, i]

        return mean_value

    def weighted_mean_set(self, set: np.ndarray, weight: np.ndarray) -> np.ndarray:
        """Calculates the mean of a set of values.

        Args:
            set (np.ndarray): The set of values.

        Returns:
            np.ndarray: The mean of the set.
        """
        # Define the number of columns
        n = set.shape[0]
        mean_value = np.zeros(n)

        for i in range(2 * n + 1):
            mean_value += weight[i] * set[:, i]

        return mean_value

    def covariance_set(self, mean: np.ndarray, set: np.ndarray) -> np.ndarray:
        """Calculate the covarince of a set of sigmapoints

        Args:
            mean (np.ndarray): The mean of the set.
            set (np.ndarray): The set of values.

        Returns:
            np.ndarray: The covariance of the set.
        """
        n = set.shape[0]
        covariance_set = np.zeros((n, n))

        for i in range(2 * n + 1):
            vector = StateVector_euler()
            vector.position = set[:, i][:3]
            vector.velocity = set[:, i][3:6]
            vector.orientation = set[:, i][6:9]
            vector.acceleration_bias = set[:, i][9:12]
            vector.gyro_bias = set[:, i][12:]

            W_i = vector - mean

            covariance_set += (1 / (2 * n + 1)) * np.outer(W_i, W_i)

        return covariance_set

    def cross_covariance_set(
        self,
        mean: np.ndarray,
        set: np.ndarray,
        mean_2: np.ndarray,
        set_2: np.ndarray,
        weight: np.ndarray,
    ) -> np.ndarray:
        """Calculate the cross covariance of a set of sigmapoints

        Args:
            mean (np.ndarray): The mean of the set.
            set (np.ndarray): The set of values.
            mean_2 (np.ndarray): The mean of the second set.
            set_2 (np.ndarray): The second set of values.

        Returns:
            np.ndarray: The cross covariance of the set.
        """
        n_x = set.shape[0]
        n_z = set_2.shape[0]
        covariance_mat = np.zeros((n_x, n_z))

        for i in range(2 * n_x + 1):
            # parse the 15-dim error state
            err_vec = set[:, i]  # shape (15,)
            W_i = err_vec - mean  # shape (15,)

            # parse the 3-dim measurement
            meas_vec = set_2[:, i]  # shape (3,)
            W_i_2 = meas_vec - mean_2  # shape (3,)

            # outer product -> shape (15,3)
            covariance_mat += weight[i] * np.outer(W_i, W_i_2)

        return covariance_mat

    def weighted_covariance_set(
        self, mean: np.ndarray, set: np.ndarray, weight: np.ndarray
    ) -> np.ndarray:
        """Calculate the covarince of a set of sigmapoints

        Args:
            mean (np.ndarray): The mean of the set.
            set (np.ndarray): The set of values.

        Returns:
            np.ndarray: The covariance of the set.
        """
        n = set.shape[0]
        covariance_set = np.zeros((n, n))

        for i in range(2 * n + 1):
            vector = StateVector_euler()
            vector.position = set[:, i][:3]
            vector.velocity = set[:, i][3:6]
            vector.orientation = set[:, i][6:9]
            vector.acceleration_bias = set[:, i][9:12]
            vector.gyro_bias = set[:, i][12:]

            W_i = vector - mean
            covariance_set += weight[i] * np.outer(W_i, W_i)

        return covariance_set

    def generate_sigma_points(
        self, error_state: StateVector_euler, Q_process_noise
    ) -> tuple[list[StateVector_euler], np.ndarray]:
        """Generates the sigma points for the UKF
        This is done using the Cholesky decomposition method
        """
        n = len(error_state.covariance)
        kappa = 3 - n

        S = np.linalg.cholesky(error_state.covariance + Q_process_noise)

        S_scaled = np.sqrt(n + kappa) * S

        weighted_points = np.concatenate((S_scaled, -S_scaled), axis=1)

        sigma_points = [StateVector_euler() for _ in range(2 * n + 1)]
        W = np.zeros(2 * n + 1)

        sigma_points[0].fill_states(error_state.as_vector())
        W[0] = kappa / (n + kappa)

        for i in range(2 * n):
            sigma_points[i + 1].fill_states(error_state + weighted_points[:, i])
            W[i + 1] = 1 / (2 * (n + kappa))

        self.W = W
        return sigma_points, W

    def nominal_state_update(
        self, current_state: StateVector_quaternion, imu_reading: np.ndarray
    ) -> StateVector_quaternion:
        """Updates the nominal state of the system.

        Args:
            current_state (np.ndarray): The current state of the system.
            imu_reading (np.ndarray): The IMU reading.

        Returns:
            np.ndarray: The updated nominal state.
        """
        # Defining the IMU readings
        imu_acceleration = imu_reading[0:3]
        imu_gyro = imu_reading[3:6]

        # Define the derivative of the state
        current_state_dot = StateVector_quaternion()

        # Define the state derivates
        current_state_dot.position = current_state.velocity
        current_state_dot.velocity = (
            np.dot(
                current_state.R_q(),
                (imu_acceleration - current_state.acceleration_bias),
            )
            + self.g
        )

        # Define the quaternion derivatives
        current_state_dot.orientation = 0.5 * quaternion_super_product(
            current_state.orientation,
            np.array(
                [
                    0,
                    imu_gyro[0] - current_state.gyro_bias[0],
                    imu_gyro[1] - current_state.gyro_bias[1],
                    imu_gyro[2] - current_state.gyro_bias[2],
                ]
            ),
        )

        # Define the bias
        current_state_dot.acceleration_bias = (
            -np.dot(self.P_ab, np.eye(3)) @ current_state.acceleration_bias
        )
        current_state_dot.gyro_bias = (
            -np.dot(self.P_wb, np.eye(3)) @ current_state.gyro_bias
        )

        return current_state_dot.euler_forward(current_state, self.dt)

    def error_state_update(
        self,
        current_error_state: StateVector_euler,
        current_state: StateVector_quaternion,
        imu_reading: np.ndarray,
    ) -> np.ndarray:
        """Updates the error state of the system.

        Args:
            current_error_state (np.ndarray): The current error state of the system.
            current_state (np.ndarray): The current state of the system.
            imu_reading (np.ndarray): The IMU reading.

        Returns:
            np.ndarray: The updated error state.
        """
        # Defining the IMU readings
        imu_acceleration = imu_reading[0:3]
        imu_gyro = imu_reading[3:6]

        A_c = np.zeros((15, 15))
        A_c[0:3, 3:6] = np.eye(3)
        A_c[3:6, 6:9] = -np.dot(
            current_state.R_q(),
            skew_symmetric(imu_acceleration - current_state.acceleration_bias),
        )
        A_c[6:9, 6:9] = -skew_symmetric(imu_gyro - current_state.gyro_bias)
        A_c[3:6, 9:12] = -current_state.R_q()
        A_c[6:9, 12:15] = -np.eye(3)
        A_c[9:12, 9:12] = -self.P_ab * np.eye(3)
        A_c[12:15, 12:15] = -self.P_wb * np.eye(3)

        # Exact matrix exponential
        A_d = expm(A_c * self.dt)

        next_error_state = A_d @ current_error_state.as_vector()

        return next_error_state

    def unscented_transform(
        self,
        sigma_points: list[StateVector_euler],
        current_state: StateVector_quaternion,
        imu_reading: np.ndarray,
    ) -> StateVector_euler:
        """Performs the Unscented Transform
        This is the corresponding to a preditction step in the EKF
        """
        n = len(sigma_points[0].as_vector())

        self.y_i = np.zeros((n, 2 * n + 1))

        for i in range(2 * n + 1):
            self.y_i[:, i] = self.error_state_update(
                sigma_points[i], current_state, imu_reading
            )

        error_state_estimate = StateVector_euler()

        x = self.weighted_mean_set(self.y_i, self.W)

        error_state_estimate.fill_states(x)
        error_state_estimate.covariance = self.weighted_covariance_set(x, self.y_i, self.W)

        return error_state_estimate

    def H(self) -> np.ndarray:
        """Calculates the measurement matrix.

        Returns:
            np.ndarray: The measurement matrix.
        """
        # Define the measurement matrix (error state is 15-dim)
        H = np.zeros((3, 16))

        # For now assume only velocity is measured (located at indices 3:6)
        H[:, 3:6] = np.eye(3)

        return H

    def injection(
        self,
        current_state_nom: StateVector_quaternion,
        current_state_error: StateVector_euler,
    ) -> StateVector_quaternion:
        """Injects the error state into the nominal state

        Args:
            current_state_nom (StateVector_quaternion): The current nominal state
            current_state_error (StateVector_euler): The current error state

        Returns:
            StateVector_quaternion: The updated nominal state
        """
        inj_state = StateVector_quaternion()

        inj_state.position = current_state_nom.position + current_state_error.position
        inj_state.velocity = current_state_nom.velocity + current_state_error.velocity
        inj_state.orientation = quaternion_super_product(
            current_state_nom.orientation,
            0.5
            * np.array(
                [
                    2,
                    current_state_error.orientation[0],
                    current_state_error.orientation[1],
                    current_state_error.orientation[2],
                ]
            ),
        )
        inj_state.acceleration_bias = (
            current_state_nom.acceleration_bias + current_state_error.acceleration_bias
        )
        inj_state.gyro_bias = (
            current_state_nom.gyro_bias + current_state_error.gyro_bias
        )

        return inj_state

    def measurement_update(
        self,
        sigma_points: list[StateVector_euler],
        current_nom_state: StateVector_quaternion,
        current_error_state: StateVector_euler,
        dvl_data: np.ndarray,
        Weight: np.ndarray,
    ) -> StateVector_euler:
        """Updates the state vector with the DVL data
        """
        H = self.H()
        R = self.R

        n = len(sigma_points[0].as_vector())

        Z_i = np.zeros((H.shape[0], 2 * n + 1))

        for i in range(2 * n + 1):
            injected_state = self.injection(current_nom_state, sigma_points[i])
            Z_i[:, i] = np.dot(H, injected_state.as_vector())

        z = self.weighted_mean_set(Z_i, Weight)
        S = self.weighted_covariance_set(z, Z_i, Weight)

        x = self.mean_set(self.y_i)

        innovation = dvl_data - z

        P_innovation = S + R

        P_xz = self.cross_covariance_set(x, self.y_i, z, Z_i, Weight)

        # Kalman gain
        K_k = np.dot(P_xz, np.linalg.inv(P_innovation))

        updated_error_state = StateVector_euler()

        # Update the state
        updated_error_state.fill_states(x + np.dot(K_k, innovation))

        # Update the covariance
        updated_error_state.covariance = current_error_state.covariance - np.dot(
            K_k, np.dot(P_innovation, K_k.T)
        )

        return updated_error_state

    def imu_update_states(
        self,
        current_state_nom: StateVector_quaternion,
        current_state_error: StateVector_euler,
        imu_data: np.ndarray,
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Updates the state vector with the IMU data

        Args:
            current_state_nom (StateVector_quaternion): The current nominal state
            current_state_error (StateVector_euler): The current error state
            imu_data (np.ndarray): The IMU data

        Returns:
            tuple[StateVector_quaternion, StateVector_euler]: The updated nominal and error states

        """
        # Update the nominal state
        current_state_nom = self.nominal_state_update(current_state_nom, imu_data)

        # Generate the sigma points
        sigma_points, _ = self.generate_sigma_points(
            current_state_error, self.Q_process_noise
        )

        # Update the error state
        current_state_error = self.unscented_transform(
            sigma_points, current_state_nom, imu_data
        )

        return current_state_nom, current_state_error

    def dvl_update_states(
        self,
        current_state_nom: StateVector_quaternion,
        current_state_error: StateVector_euler,
        dvl_data: np.ndarray,
        imu_data: np.ndarray,
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Update the error state given the DVL data

        Args:
            current_state_nom (StateVector_quaternion): The current nominal state
            current_state_error (StateVector_euler): The current error state
            dvl_data (np.ndarray): The DVL data to update the state with

        Returns:
            tuple[StateVector_quaternion, StateVector_euler]: The updated nominal and error states
        """
        # Generate the sigma points
        sigma_points, weight = self.generate_sigma_points(
            current_state_error, self.Q_process_noise
        )

        # Update the error state
        current_state_error = self.unscented_transform(
            sigma_points, current_state_nom, imu_data
        )

        # Update the error state
        current_state_error = self.measurement_update(
            sigma_points, current_state_nom, current_state_error, dvl_data, weight
        )

        return current_state_nom, current_state_error

    def inject_and_reset(
        self,
        current_state_nom: StateVector_quaternion,
        current_state_error: StateVector_euler,
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Injects the error state into the nominal state and resets the error state

        Args:
            current_state_nom (StateVector_quaternion): The current nominal state
            current_state_error (StateVector_euler): The current error state

        Returns:
            tuple[StateVector_quaternion, StateVector_euler]: The updated nominal and error states
        """
        inj_state = self.injection(current_state_nom, current_state_error)

        G = np.eye(15)
        G[6:9, 6:9] = np.eye(3) - skew_symmetric(0.5 * current_state_error.orientation)

        current_state_error.covariance = np.dot(
            np.dot(G, current_state_error.covariance), G.T
        )
        current_state_error.covariance += np.eye(15)

        current_state_error.fill_states(np.zeros(15))

        return inj_state, current_state_error
