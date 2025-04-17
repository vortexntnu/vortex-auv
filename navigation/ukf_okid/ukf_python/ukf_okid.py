import numpy as np
from ukf_okid_class import (
    MeasModel,
    StateQuat,
    covariance_measurement,
    covariance_set,
    cross_covariance,
    mean_measurement,
    mean_set,
    okid_process_model,
)


class UKF:
    def __init__(self, process_model: okid_process_model, x_0, P_0, Q, G):
        self.x = x_0
        self.P = P_0
        self.Q = Q
        self.G = G
        self.process_model = process_model
        self.sigma_points_list = None
        self.measurement_updated = MeasModel()
        self.y_i = None
        self.weight = None
        self.delta = self.generate_delta_matrix(len(x_0.as_vector()) - 1)
        self.cross_correlation = None

    def generate_delta_matrix(self, n: float) -> np.ndarray:
        """Generates the weight matrix used in the TUKF sigma point generation.

        Parameters:
            n (int): The state dimension.

        Returns:
            delta (np.ndarray): An n x 2n orthonormal transformation matrix used to generate TUKF sigma points.
        """
        delta = np.zeros((n, 2 * n))
        k = 0.01  # Tuning parameter to ensure pos def

        for i in range(2 * n):
            for j in range(n // 2):
                delta[2 * j + 1, i] = (
                    np.sqrt(2) * np.sin(2 * j - 1) * ((k * np.pi) / n)
                )
                delta[2 * j, i] = np.sqrt(2) * np.cos(2 * j - 1) * ((k * np.pi) / n)

            if (n % 2) == 1:
                delta[n - 1, i] = (-1) ** i
        return delta

    def sigma_points(self, current_state: StateQuat) -> list[StateQuat]:
        """Functions that generate the sigma points for the UKF."""
        n = len(current_state.covariance)

        S = np.linalg.cholesky(current_state.covariance + self.Q)

        self.sigma_points_list = [StateQuat() for _ in range(2 * n)]

        for index, state in enumerate(self.sigma_points_list):
            delta_x = S @ self.delta[:, index]
            state.fill_dynamic_states(current_state.as_vector(), delta_x)

        return self.sigma_points_list

    def unscented_transform(self, current_state: StateQuat) -> StateQuat:
        """The unscented transform function generates the priori state estimate."""
        self.sigma_points(current_state)
        n = len(current_state.covariance)

        self.y_i = [StateQuat() for _ in range(2 * n)]

        for i, state in enumerate(self.sigma_points_list):
            self.process_model.model_prediction(state)
            self.process_model.state_vector_prev = state
            self.y_i[i] = self.process_model.euler_forward()

        state_estimate = StateQuat()
        x = mean_set(self.y_i)

        state_estimate.fill_states(x)
        state_estimate.covariance = covariance_set(self.y_i, x)
        return state_estimate

    def measurement_update(
        self, current_state: StateQuat, measurement: MeasModel
    ) -> None:
        """Function that updates the state estimate with a measurement.

        Hopefully this is the DVL or GNSS
        """
        n = len(current_state.covariance)
        z_i = [MeasModel() for _ in range(2 * n)]

        for i, state in enumerate(self.sigma_points_list):
            z_i[i] = measurement.H(state)

        self.measurement_updated.measurement = mean_measurement(z_i)

        self.measurement_updated.covariance = covariance_measurement(
            z_i, self.measurement_updated.measurement
        )

        self.cross_correlation = cross_covariance(
            self.y_i,
            current_state.as_vector(),
            z_i,
            self.measurement_updated.measurement,
        )

    def posteriori_estimate(
        self,
        current_state: StateQuat,
        measurement: MeasModel,
    ) -> StateQuat:
        """Calculates the posteriori estimate using measurement and the prior estimate."""
        nu_k = MeasModel()
        nu_k.measurement = (
            measurement.measurement - self.measurement_updated.measurement
        )
        nu_k.covariance = self.measurement_updated.covariance + measurement.covariance

        K_k = np.dot(self.cross_correlation, np.linalg.inv(nu_k.covariance))

        posteriori_estimate = StateQuat()

        posteriori_estimate.fill_states_different_dim(
            current_state.as_vector(), np.dot(K_k, nu_k.measurement)
        )
        posteriori_estimate.covariance = current_state.covariance - np.dot(
            K_k, np.dot(nu_k.covariance, np.transpose(K_k))
        )

        return posteriori_estimate
