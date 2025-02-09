from dataclasses import dataclass, field
from typing import tuple

import numpy as np


@dataclass
class StateVector_quaternion:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Position vector (x, y, z)
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Velocity vector (u, v, w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.zeros(4)
    )  # Orientation quaternion (w, x, y, z)
    acceleration_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Acceleration bias vector (b_ax, b_ay, b_az)
    gyro_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Gyro bias vector (b_gx, b_gy, b_gz)

    def R_q(self) -> np.ndarray:
        """Calculates the rotation matrix from the orientation quaternion.

        Returns:
            np.ndarray: The rotation matrix.
        """
        q0, q1, q2, q3 = self.orientation
        R = np.array(
            [
                [
                    1 - 2 * q2**2 - 2 * q3**2,
                    2 * (q1 * q2 - q0 * q3),
                    2 * (q0 * q2 + q1 * q3),
                ],
                [
                    2 * (q1 * q2 + q0 * q3),
                    1 - 2 * q1**2 - 2 * q3**2,
                    2 * (q2 * q3 - q0 * q1),
                ],
                [
                    2 * (q1 * q3 - q0 * q2),
                    2 * (q0 * q1 + q2 * q3),
                    1 - 2 * q1**2 - 2 * q2**2,
                ],
            ]
        )

        return R

    def euler_forward(
        self, current_state: 'StateVector_quaternion', dt: float
    ) -> 'StateVector_quaternion':
        # Define the new state
        new_state = StateVector_quaternion()

        # Define the state derivatives
        new_state.position = current_state.position + self.position * dt
        new_state.velocity = current_state.velocity + self.velocity * dt
        new_state.orientation = current_state.orientation + self.orientation * dt
        new_state.acceleration_bias = (
            current_state.acceleration_bias + self.acceleration_bias * dt
        )
        new_state.gyro_bias = current_state.gyro_bias + self.gyro_bias * dt

        # Normalize the orientation quaternion
        new_state.orientation /= np.linalg.norm(new_state.orientation)

        return new_state


@dataclass
class StateVector_euler:
    position: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Position vector (x, y, z)
    velocity: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Velocity vector (u, v, w)
    orientation: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Orientation angles (roll, pitch, yaw)
    acceleration_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Acceleration bias vector (b_ax, b_ay, b_az)
    gyro_bias: np.ndarray = field(
        default_factory=lambda: np.zeros(3)
    )  # Gyro bias vector (b_gx, b_gy, b_gz)
    covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((15, 15))
    )  # Covariance matrix

    def fill_states(self, state: np.ndarray) -> None:
        """Fills the state vector with the values from a numpy array.

        Args:
            state (np.ndarray): The state vector.
        """
        self.position = state[0:3]
        self.velocity = state[3:6]
        self.orientation = state[6:9]
        self.acceleration_bias = state[9:12]
        self.gyro_bias = state[12:15]

    def copy_state(self, wanted_state: 'StateVector_euler') -> None:
        """Copies the state from a StateVector object into the current StateVector object.

        Args:
            wanted_state (StateVector_euler): The quaternion state to copy from.
        """
        self.position = wanted_state.position
        self.velocity = wanted_state.velocity
        self.orientation = wanted_state.orientation
        self.acceleration_bias = wanted_state.acceleration_bias
        self.gyro_bias = wanted_state.gyro_bias


@dataclass
class MeasurementModel:
    measurement: np.ndarray = field(
        default_factory=lambda: np.zeros(6)
    )  # Measurement vector
    measurement_matrix: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 15))
    )  # Measurement matrix
    measurement_covariance: np.ndarray = field(
        default_factory=lambda: np.zeros((6, 6))
    )  # Measurement noise matrix


class ErrorStateKalmanFilter:
    def __init__(
        self,
        P_ab: np.ndarray,
        P_wb: np.ndarray,
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

    def skew_symmetric(self, vector: np.ndarray) -> np.ndarray:
        """Calculates the skew symmetric matrix of a vector.

        Args:
            vector (np.ndarray): The vector.

        Returns:
            np.ndarray: The skew symmetric matrix.
        """
        return np.array(
            [
                [0, -vector[2], vector[1]],
                [vector[2], 0, -vector[0]],
                [-vector[1], vector[0], 0],
            ]
        )

    def quaternion_super_product(self, q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
        """Calculates the quaternion super product of two quaternions.

        Args:
            q1 (np.ndarray): The first quaternion.
            q2 (np.ndarray): The second quaternion.

        Returns:
            np.ndarray: The quaternion super product.
        """
        nu_0, eta_0_x, eta_0_y, eta_0_z = q1
        nu_1, eta_1_x, eta_1_y, eta_1_z = q2

        eta_0 = np.array([[eta_1_x, eta_1_y, eta_1_z]]).T
        eta_1 = np.array([[eta_0_x, eta_0_y, eta_0_z]]).T

        eta_new = (
            nu_1 * eta_0 + nu_0 * eta_1 + np.dot(self.skew_symmetric(eta_0), eta_1)
        )
        nu_new = nu_0 * nu_1 - np.dot(eta_0.T, eta_1)

        q_new = np.array([nu_new, eta_new[0], eta_new[1], eta_new[2]])
        q_new /= np.linalg.norm(q_new)

        return q_new

    def van_loan_discretization(
        self, A_c: np.ndarray, G_c: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Calculates the Van Loan discretization of a continuous-time system.

        Args:
            A_c (np.ndarray): The A matrix.
            G_c (np.ndarray): The G matrix.

        Returns:
            tuple: The A_d and GQG_d matrices.
        """
        GQG_T = np.dot(np.dot(G_c, self.Q_process_noise), G_c.T) * self.dt

        matrix_exp = (
            np.block([[A_c, GQG_T], [np.zeros((A_c.shape[0], A_c.shape[0])), A_c.T]])
            * self.dt
        )

        van_loan_matrix = np.linalg.expm(matrix_exp)

        V1 = van_loan_matrix[A_c.shape[0] :, A_c.shape[0] :]
        V2 = van_loan_matrix[: A_c.shape[0], A_c.shape[0] :]

        A_d = V1.T
        GQG_d = A_d @ V2

        return A_d, GQG_d

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
        current_state_dot.orientation = 0.5 * self.quaternion_super_product(
            current_state.orientation,
            np.array([0, imu_gyro[0], imu_gyro[1], imu_gyro[2]]),
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
    ) -> StateVector_euler:
        """Updates the error state of the system.

        Args:
            current_error_state (np.ndarray): The current error state of the system.
            current_state (np.ndarray): The current state of the system.
            imu_reading (np.ndarray): The IMU reading.

        Returns:
            np.ndarray: The updated error state.
        """
        # Define the derivative of the state
        next_error_state = StateVector_euler()

        # Defining the IMU readings
        imu_acceleration = imu_reading[0:3]
        imu_gyro = imu_reading[3:6]

        A_c = np.zeros((15, 15))
        A_c[0:3, 3:6] = np.eye(3)
        A_c[3:6, 6:9] = -np.dot(
            current_state.R_q(),
            self.skew_symmetric(imu_acceleration - current_state.acceleration_bias),
        )
        A_c[6:9, 6:9] = -self.skew_symmetric(imu_gyro - current_state.gyro_bias)
        A_c[3:6, 9:12] = -current_state.R_q()
        A_c[6:9, 12:15] = -np.eye(3)
        A_c[9:12, 9:12] = -self.P_ab * np.eye(3)
        A_c[12:15, 12:15] = -self.P_wb * np.eye(3)

        G_c = np.zeros((15, 12))
        G_c[3:6, 0:3] = -current_state.R_q()
        G_c[6:9, 3:6] = -np.eye(3)
        G_c[9:12, 6:9] = np.eye(3)
        G_c[12:15, 9:12] = np.eye(3)

        # Van loan discretization
        A_d, GQG_d = self.van_loan_discretization(A_c, G_c, self.dt)

        # Inserting the new state and covariance
        next_error_state.copy_state(current_error_state)
        next_error_state.covariance = (
            np.dot(np.dot(A_d, current_error_state.covariance), A_d.T) + GQG_d
        )

        return next_error_state

    def H(self) -> np.ndarray:
        """Calculates the measurement matrix.

        Returns:
            np.ndarray: The measurement matrix.
        """
        # Define the measurement matrix
        H = np.zeros((3, 15))

        # For now assume only velocity is measured
        H[0:3, 3:6] = np.eye(3)

        return H

    def prediction_from_estimates(
        self,
        current_state: StateVector_quaternion,
        current_error_state: StateVector_euler,
        imu_reading: np.ndarray,
    ) -> StateVector_euler:
        """Predicts the measurement from the current state and error state.

        Args:
            current_state (StateVector_quaternion): The current state of the system.
            current_error_state (StateVector_euler): The current error state of the system.
            imu_reading (np.ndarray): The IMU reading.

        Returns:
            StateVector_euler: The predicted measurement.
        """
        # Define the z_pred matrix
        z_pred = MeasurementModel()

        # Define the z_pred values separately
        z_pred_1 = np.hstack((current_state.position, current_state.velocity))
        z_pred_2 = np.hstack(
            np.dot(current_state.R_q(), self.lever_arm),
            np.dot(
                current_state.R_q,
                np.dot(
                    self.skew_symmetric(current_state.angular_velocity), self.lever_arm
                ),
            ),
        )

        # Combine the z_pred values
        z_pred.measurement = z_pred_1 + z_pred_2

        # Define the H matrix
        z_pred.measurement_matrix = self.H()
        R = self.R
        z_pred.measurement_covariance = (
            np.dot(
                np.dot(z_pred.measurement_matrix, current_error_state.covariance),
                z_pred.measurement_matrix.T,
            )
            + R
        )

        return z_pred

    def measurement_update(
        self,
        error_state_pred: StateVector_euler,
        z_pred: MeasurementModel,
        dvl_measure: np.array,
    ) -> StateVector_euler:
        """Updates the error state of the system.

        Args:
            current_error_state (np.ndarray): The current error state of the system.
            measurement (np.ndarray): The measurement.

        Returns:
            np.ndarray: The updated error state.
        """
        # Define new error state value
        new_error_state = StateVector_euler()

        # Define the measurement matrix
        innovation = dvl_measure - z_pred.measurement
        H = z_pred.measurement_matrix
        R = self.R
        P = error_state_pred.covariance
        S = z_pred.measurement_covariance

        # Kalman gain calculation
        W = np.dot(P, np.linalg.solve(S, H).T)
        new_error_state.fill_states(np.dot(W, innovation))

        I_WH = np.eye(15) - np.dot(W, H)
        new_error_state.covariance = np.dot(np.dot(I_WH, P), I_WH.T) + np.dot(
            np.dot(W, R), W.T
        )

        return new_error_state

    def imu_update_states(
        self,
        current_pred_nom: StateVector_quaternion,
        current_pred_err: StateVector_euler,
        imu_readings: np.array,
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Calculates the predicted state using the IMU readings.

        Args:
            current_pred_nom (StateVector_quaternion): The current nominal state.
            current_pred_err (StateVector_euler): The current error state.
            imu_readings (np.array): The IMU readings.

        Returns:
            tuple: The predicted nominal state and the predicted error state.
        """
        pred_nom_state = self.nominal_state_update(current_pred_nom, imu_readings)
        pred_err_state = self.error_state_update(
            current_pred_err, current_pred_nom, imu_readings
        )

        return pred_nom_state, pred_err_state

    def dvl_update_states(
        self,
        current_pred_nom: StateVector_quaternion,
        current_pred_err: StateVector_euler,
        dvl_measure: np.array,
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Calculates the predicted state using the DVL readings.

        Args:
            current_pred_nom (StateVector_quaternion): The current nominal state.
            current_pred_err (StateVector_euler): The current error state.
            dvl_measure (np.array): The DVL readings.

        Returns:
            tuple: The predicted nominal state and the predicted error state.
        """
        z_pred = self.prediction_from_estimates(
            current_pred_nom, current_pred_err, dvl_measure
        )
        new_error_state = self.measurement_update(current_pred_err, z_pred, dvl_measure)

        return current_pred_nom, new_error_state

    def injection_and_reset(
        self, next_state: StateVector_quaternion, next_error_state: StateVector_euler
    ) -> tuple[StateVector_quaternion, StateVector_euler]:
        """Injects the error state into the nominal state and resets the error state.

        Args:
            next_state (StateVector_quaternion): The next nominal state.
            next_error_state (StateVector_euler): The next error state.

        Returns:
            tuple: The injected nominal state and the reset error state.
        """
        # Define the new state
        inj_state = StateVector_quaternion()

        # Injecting the error state
        inj_state.position = next_state.position + next_error_state.position
        inj_state.velocity = next_state.velocity + next_error_state.velocity
        inj_state.orientation = self.quaternion_super_product(
            next_state.orientation,
            0.5
            * np.array(
                [
                    2,
                    next_error_state.orientation[0],
                    next_error_state.orientation[1],
                    next_error_state.orientation[2],
                ]
            ),
        )
        inj_state.acceleration_bias = (
            next_state.acceleration_bias + next_error_state.acceleration_bias
        )
        inj_state.gyro_bias = next_state.gyro_bias + next_error_state.gyro_bias

        # Resetting the error state
        G = np.eye(15)
        G[6:9, 6:9] = np.eye(3) - self.skew_symmetric(
            0.5 * next_error_state.orientation
        )

        next_error_state.covariance = np.dot(
            np.dot(G, next_error_state.covariance), G.T
        )
        next_error_state.fill_states(np.zeros(15))

        return inj_state, next_error_state
