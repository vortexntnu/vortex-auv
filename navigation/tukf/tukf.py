import numpy as np
from tukf_class import (
    MeasModel,
    AUVState,
    covariance_measurement,
    covariance_set,
    cross_covariance,
    mean_measurement,
    mean_set,
    F_dynamics,
    generate_delta_matrix,
)

def print_matrix(matrix, name="Matrix"):
    """Custom print function to print matrices in a formatted form."""
    print(f"{name}: {matrix.shape}")
    if isinstance(matrix, np.ndarray):
        for row in matrix:
            print(" ".join(f"{val:.2f}" for val in row))
    else:
        print(matrix)

class TUKF:
    def __init__(self, x_0: AUVState, Q):
        self.x = x_0
        self.Q = Q
        self.delta = generate_delta_matrix(len(x_0.as_vector())) / np.sqrt(len(x_0.as_vector()))
        self.sigma_points_list = None
        self.measurement_updated = MeasModel()
        self.dt = 0.01  # Time step for dynamics
        self.flagg = 0
        self.filter_failed = False

    def sigma_points(self, current_state: AUVState) -> list[AUVState]:
        """Functions that generate the sigma points for the UKF."""
        n = len(current_state.covariance)
        self.flagg += 1
        try:
            S = np.linalg.cholesky(current_state.covariance)
        except np.linalg.LinAlgError:
            print("Cholesky decomposition failed!")
            print("flagg", self.flagg)
            print_matrix(current_state.covariance, "Current State Covariance")
            print_matrix(self.Q, "Process Noise Covariance (Q)")
            
            # Set flag to indicate filter has failed
            self.filter_failed = True
            
            # Create a valid but minimal S matrix to avoid crashing
            # This allows the simulation to continue to the next step where it can be checked
            S = np.eye(n) * 1e-6
            
        self.sigma_points_list = [AUVState() for _ in range(2 * n)]

        for index, state in enumerate(self.sigma_points_list):
            state.fill_states(current_state.as_vector() + S @ self.delta[:, index])

        return self.sigma_points_list

    def unscented_transform(self, current_state: AUVState, control_force: np.ndarray) -> AUVState:
        """The unscented transform function generates the priori state estimate."""
        self.sigma_points(current_state)
        n = len(current_state.covariance)

        self.y_i = [AUVState() for _ in range(2 * n)]

        for i, sp in enumerate(self.sigma_points_list):
            self.y_i[i] = F_dynamics(sp, self.dt, control_force)

        state_estimate = AUVState()
        x = mean_set(self.y_i)

        state_estimate.fill_states(x)
        state_estimate.covariance = covariance_set(self.y_i, x) + + self.Q
        return state_estimate

    def measurement_update(
        self, current_state: AUVState, measurement: MeasModel
    ) -> None:
        """Function that updates the state estimate with a measurement.

        Hopefully this is the DVL or GNSS
        """
        n = len(current_state.covariance)
        z_i = [MeasModel() for _ in range(2 * n)]

        for i, state in enumerate(self.y_i):
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
        current_state: AUVState,
        measurement: MeasModel,
    ) -> AUVState:
        """Calculates the posteriori estimate using measurement and the prior estimate."""
        nu_k = MeasModel()
        nu_k.measurement = (
            measurement.measurement - self.measurement_updated.measurement
        )
        nu_k.covariance = self.measurement_updated.covariance + measurement.covariance

        K_k = np.dot(self.cross_correlation, np.linalg.inv(nu_k.covariance))

        posteriori_estimate = AUVState()

        posteriori_estimate.fill_states(
            current_state.as_vector() + np.dot(K_k, nu_k.measurement)
        )
        posteriori_estimate.covariance = current_state.covariance - np.dot(
            K_k, np.dot(nu_k.covariance, np.transpose(K_k))
        )

        return posteriori_estimate
