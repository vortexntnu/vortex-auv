import numpy as np
from ukf_okid_class import *


class UKF:
    def __init__(self, process_model: process_model, x_0, P_0, Q, R):
        self.x = x_0
        self.P = P_0
        self.Q = Q
        self.R = R
        self.process_model = process_model
        self.sigma_points_list = None
        self.y_i = None
        self.weight = None
        self.T = self.generate_T_matrix(len(P_0))

    def generate_T_matrix(self, n: float) -> np.ndarray:
        """Generates the orthonormal transformation matrix T used in the TUKF sigma point generation.

        Parameters:
            n (int): The state dimension.

        Returns:
            T (np.ndarray): An n x 2n orthonormal transformation matrix used to generate TUKF sigma points.
        """
        T = np.zeros((n, n))

        for i in range(n):
            for j in range(n // 2):
                T[2 * j - 2, i - 1] = np.sqrt(2) * np.cos(((2 * j - 1) * i * np.pi) / n)
                T[2 * j - 1, i - 1] = np.sqrt(2) * np.sin(((2 * j - 1) * i * np.pi) / n)

            if n % 2 == 1:  # if n is odd
                T[n - 1, i - 1] = (-1) ** i

        T = T / np.sqrt(2)

        return T

    def sigma_points(self, current_state: StateQuat) -> list[StateQuat]:
        """Functions that generate the sigma points for the UKF"""
        n = len(current_state.covariance)

        I = np.hstack([np.eye(n), -np.eye(n)])
        my = np.sqrt(n) * I
        delta = self.T @ my

        S = np.linalg.cholesky(current_state.covariance + self.Q)

        self.sigma_points_list = [StateQuat() for _ in range(2 * n)]

        for index, state in enumerate(self.sigma_points_list):
            delta_x = S @ delta[:, index]
            state.fill_states_different_dim(current_state.as_vector(), delta_x)

        return self.sigma_points_list

    def unscented_transform(self, current_state: StateQuat) -> StateQuat:
        """The unscented transform function generates the priori state estimate"""
        _ = self.sigma_points(current_state)
        n = len(current_state.covariance)

        self.y_i = [StateQuat() for _ in range(2 * n)]

        for i in range(2 * n):
            self.process_model.model_prediction(self.sigma_points_list[i])
            self.y_i[i] = self.process_model.euler_forward()

        state_estimate = StateQuat()
        x = mean_set(self.y_i)

        state_estimate.fill_states(x)
        state_estimate.covariance = covariance_set(self.y_i, x)
        return state_estimate

    def measurement_update(
        self, current_state: StateQuat, measurement: MeasModel
    ) -> tuple[MeasModel, np.ndarray]:
        """Function that updates the state estimate with a measurement
        Hopefully this is the DVL or GNSS
        """
        n = len(current_state.covariance)
        z_i = [MeasModel() for _ in range(2 * n)]

        for i in range(2 * n):
            z_i[i] = measurement.H(self.sigma_points_list[i])

        meas_update = MeasModel()

        meas_update.measurement = mean_measurement(z_i)

        meas_update.covariance = covariance_measurement(z_i, meas_update.measurement)

        cross_correlation = cross_covariance(
            self.y_i, current_state.as_vector(), z_i, meas_update.measurement
        )

        return meas_update, cross_correlation

    def posteriori_estimate(
        self,
        current_state: StateQuat,
        cross_correlation: np.ndarray,
        measurement: MeasModel,
        ex_measuremnt: MeasModel,
    ) -> StateQuat:
        """Calculates the posteriori estimate using measurement and the prior estimate"""
        nu_k = MeasModel()

        nu_k.measurement = measurement.measurement - ex_measuremnt.measurement
        nu_k.covariance = ex_measuremnt.covariance + measurement.covariance

        K_k = np.dot(cross_correlation, np.linalg.inv(nu_k.covariance))

        posteriori_estimate = StateQuat()

        posteriori_estimate.fill_states_different_dim(
            current_state.as_vector(), np.dot(K_k, nu_k.measurement)
        )
        posteriori_estimate.covariance = current_state.covariance - np.dot(
            K_k, np.dot(nu_k.covariance, np.transpose(K_k))
        )

        self.process_model.state_vector_prev = posteriori_estimate

        return posteriori_estimate
